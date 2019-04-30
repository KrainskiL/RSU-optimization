#####################################
## Optimization auxilary functions ##
#####################################

"""
`optimize_RSU_location` function returns dictionary with number of RSUs in given nodes.

**Input parameters**
* `OSMmap` : MapData type from OpenStreetMapX package
* `stats` : dictionary with highest traffic density on edges during base simulation run
* `range` : range of RSUs
* `throughput` : number of agents RSU can serve at once
* `α` : required service availabilty/data coverage α ∈ <0,1>
* `ϵ` : service availabilty tolerance - all weights updates must be within α ± ϵ
"""

function optimize_RSU_location(OSMmap::MapData,
                                range::Float64,
                                throughput::Int64,
                                mode::String,
                                mode_input)
    RSUs = Dict{Int64,Int64}()
    if mode == "cover_all_nodes"
        #Count how many times each node was passed by in base simulation
        passed_nodes = StatsBase.countmap(collect(Iterators.flatten([a.route for a in mode_input if a.smart])))
        while !isempty(passed_nodes)
            #Find node with highest count
            how_many, nodeID = findmax(passed_nodes)
            #Gather count from all nodes in range
            rng_nodes = OpenStreetMapX.nodes_within_range(OSMmap.nodes, OSMmap.nodes[nodeID], range)
            #Filter rng_nodes - only include nodes which agents passed
            rng_nodes = [n for n in rng_nodes if haskey(passed_nodes, n)]
            sum_count = sum([passed_nodes[n] for n in rng_nodes])
            #Calculate number of RSUs in node
            N = max(1, ceil(sum_count/throughput*0.1))
            RSUs[nodeID] = N
            #Remove served nodes
            for node in rng_nodes
                delete!(passed_nodes, node)
            end
        end
        return RSUs
    elseif mode == "average_density_based"
        #Create working dictionary
        temp = deepcopy(stats)
        while !isempty(temp)
            #Divide traffic density by roads length - traffic per road unit
            unit_density = Dict{Array{Int64,1},Float64}()
            for k in keys(temp)
                unit_density[k] = temp[k]/OSMmap.w[k[1],k[2]]
            end
            #Gather unit traffic in nodes
            vertices_traffic = Dict{Int,Float64}()
            for (key,val) in unit_density
                map(x-> haskey(vertices_traffic, x) ? vertices_traffic[x]+=val : vertices_traffic[x]=val, key)
            end
            #Node with highest aggregated traffic
            maxvertex = findmax(vertices_traffic)[2]
            maxnode = OSMmap.n[maxvertex]
            #Find nodes within RSU range from chosen node
            rng_nodes = OpenStreetMapX.nodes_within_range(OSMmap.nodes, OSMmap.nodes[maxnode], range)
            rng_vertices = [OSMmap.v[k] for k in rng_nodes if haskey(OSMmap.v,k)]
            #Transform vertices within range into edges
            edges_product = collect.(vec(collect(Iterators.product(rng_vertices, rng_vertices))))
            rng_edges = [p for p in edges_product if haskey(temp, p)]
            if isempty(rng_edges) rng_edges = [k for k in keys(temp) if maxvertex in k] end
            #Sum traffic on edges in range
            N = sum([temp[e] for e in rng_edges])
            #Place new RSUs in node
            RSUs[maxnode] = max(1,ceil(N/(0.2*length(rng_edges)*throughput)))
            #Delete all traffic in covered edges
            for e in rng_edges delete!(temp, e) end
        end
        return RSUs
    else
        error("Wrong optimization mode.")
    end
end



function reoptimize_RSU_location!(OSMmap::MapData,
                                RSU_Dict::Dict{Int64,Int64},
                                failed_coordinates::Vector{Vector{ENU}},
                                range::Float64)
    distinct_failed_ENUs = unique(collect(Iterators.flatten(failed_coordinates)))
    RSU_ENU = [OSMmap.nodes[k] for k in keys(RSU_Dict)]
    #Split set of coordinates according to reason of failure
    failed_throughput = Vector{ENU}()
    failed_range = Vector{ENU}()
    for enu in distinct_failed_ENUs
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
