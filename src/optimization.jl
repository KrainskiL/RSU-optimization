#####################################
## Optimization auxilary functions ##
#####################################

"""
`optimize_RSU_location` function returns dictionary with number of RSUs in given nodes.

**Input parameters**
* `OSMmap` : MapData type from OpenStreetMapX package
* `range` : range of RSUs
* `throughput` : number of agents RSU can serve at once
* `div_coeff` : adjustment factor for calculating number of RSUs in node
"""
function optimize_RSU_location(OSMmap::MapData,
                                inAgents::Vector{Agent},
                                range::Float64,
                                throughput::Int64,
                                div_coeff::Float64 = 0.1)
    RSUs = Vector{RSU}()
    #Count how many times each node was passed by smart agents in base simulation
    passed_nodes = StatsBase.countmap(collect(Iterators.flatten([a.route for a in inAgents if a.smart])))
    while !isempty(passed_nodes)
        #Find node with highest count
        nodeID = findmax(passed_nodes)[2]
        #Gather count from all nodes in range
        rng_nodes = OpenStreetMapX.nodes_within_range(OSMmap.nodes, OSMmap.nodes[nodeID], range)
        #Filter rng_nodes - only include nodes which agents passed
        rng_nodes = filter(n-> haskey(passed_nodes, n), rng_nodes)
        sum_count = sum(map(n-> passed_nodes[n],rng_nodes))
        #Calculate number of RSUs in node
        N = Int(ceil(sum_count/throughput * div_coeff))
        #Create new RSU entry and push it to RSUs list
        new_RSU = RSU(nodeID, OSMmap.nodes[nodeID], N, N * throughput)
        push!(RSUs, new_RSU)
        #Remove served nodes
        for node in rng_nodes delete!(passed_nodes, node) end
    end
        return RSUs
end

"""
`reoptimize_RSU_location` function adjust RSUs location and number to meet service availability and utilization criteria

**Input parameters**
* `OSMmap` : MapData type from OpenStreetMapX package
* `range` : range of RSUs
* `RSUs` : vector with RSUs used in simulation
* `failed_coor` : vector of vectors with agents coordinates missing an update
"""
function reoptimize_RSU_location!(OSMmap::MapData,
                                RSUs::Vector{RSU},
                                failed_coor::Vector{Vector{ENU}},
                                range::Float64)
    flat_ = unique(collect(Iterators.flatten(failed_coor)))
    RSU_ENU = getfield.(RSUs, :ENU)
    #Split set of coordinates according to reason of failure
    failed_throughput = Vector{ENU}()
    failed_range = Vector{ENU}()
    for enu in flat
            if any([OpenStreetMapX.distance(RSU,enu) <= range for RSU in RSU_ENU])
                push!(failed_throughput, enu)
            else
                push!(failed_range, enu)
            end
    end
    #Handle agents who failed due to being out of range
    dict_in_range = Dict{ENU,Array{Int64,1}}()
    for elem in failed_range
        dict_in_range[elem] = OpenStreetMapX.nodes_within_range(OSMmap.nodes, elem, range)
        dict_in_range[elem] = [n for n in dict_in_range[elem] if haskey(OSMmap.v,n)]
        isempty(dict_in_range[elem]) && delete!(dict_in_range, elem)
    end
    #Repeat until failed points are in RSUs range
    while !isempty(dict_in_range)
        nodes_count = StatsBase.countmap(collect(Iterators.flatten(values(dict_in_range))))
        nodeID = findmax(nodes_count)[2]
        #Put RSU in given node
        RSU_Dict[nodeID] = 1
        #Delete all points in new RSU range
        for (k,v) in dict_in_range nodeID in v && delete!(dict_in_range, k) end
    end

    #Handle agents who failed due to reached transfer limit
    nodes_throughput = Vector{Int64}()
    for enu in failed_throughput
        distance_dict = Dict([n=>OpenStreetMapX.distance(enu,OSMmap.nodes[n]) for n in keys(RSU_Dict)])
        nodeID = findmin(distance_dict)[2]
        !(nodeID in nodes_throughput) && push!(nodes_throughput,nodeID)
    end
    for n in nodes_throughput RSU_Dict[n] += 1 end
    #Return updated dictionary
    return RSU_Dict
end

"""
`get_agent_coordinates` return agents coordinates in ENU system

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `inAgent` : agent which current coordinates are requested
"""
function get_agent_coordinates(OSMmap::MapData, inAgent::Agent)
    if inAgent.pos == 0.0 return OSMmap.nodes[inAgent.route[1]] end
    pA = OSMmap.nodes[inAgent.route[1]]
    pB = OSMmap.nodes[inAgent.route[2]]
    rel_pos = inAgent.pos/OSMmap.w[inAgent.edge[1],inAgent.edge[2]]
    Agent_coor = ENU(pA.east+(pB.east-pA.east)*rel_pos,
                    pA.north+(pB.north-pA.north)*rel_pos,
                    pA.up+(pB.up-pA.up)*rel_pos)
    return Agent_coor
end

"""
`ITS_quality_assess` return various measures describing ITS model quality

**Input parameters**
* `smart_ind` : vector with true flags for smart agents
* `times_base` : agents travelling time in base scenario
* `times_ITS` : agents travelling time in ITS scenario
* `srvc_avblty` : vector with percentage service availability (from `simulation_ITS`)
* `RSUs_util` : vector of dictionaries with RSUs utilization (from `simulation_ITS`)
* `RSU_ENU` : vector of RSUs ENU coordinates
* `range` : infrastructure transfer range
* `failures` : array with ENU coordinates of smart agents missing an update (from `simulation_ITS`)
"""
function ITS_quality_assess(smart_ind::BitArray{1},
                            times_base::Vector{Float64},
                            times_ITS::Vector{Float64},
                            srvc_avblty::Vector{Float64},
                            RSUs_util,
                            RSU_ENU::Vector{ENU},
                            range::Float64,
                            failures::Vector{Vector{ENU}})
    perc_time_diff = (times_base - times_ITS)./times_base
    #Means
    mean_time_overall = round(mean(perc_time_diff), digits=3)
    mean_time_smart = round(mean(perc_time_diff[smart_ind]), digits=3)
    mean_time_not_smart = round(mean(perc_time_diff[.!smart_ind]), digits=3)
    mean_srvc_avblty = round(mean(srvc_avblty), digits=3)
    mean_RSUs_util = [mean(values(Dict)) for Dict in RSUs_util]
    #Gather all results in one variable
    mean_tuple = (overall_time = mean_time_overall,
                    smart_time = mean_time_smart,
                    other_time = mean_time_not_smart,
                    service_availability = mean_srvc_avblty,
                    RSUs_utilization = mean_RSUs_util)
    #Categorize failures
    failures_type = Vector{Vector{Bool}}()
    for element in failures
        failures_type_update = Vector{Bool}()
        for e in element
                if any([OpenStreetMapX.distance(RSU,e) <= range for RSU in RSU_ENU])
                    push!(failures_type_update, false)
                else
                    push!(failures_type_update, true)
                end
        end
        failures_type = [failures_type; [failures_type_update]]
    end
    mixed_out_of_range = hcat([sum(e) for e in failures_type],
                                [sum(e)/length(e) for e in failures_type])
    return mean_tuple, failures_type, mixed_out_of_range
end
