#####################################
## Optimization auxilary functions ##
#####################################

"""
`calculate_RSU_location` function returns dictionary with number of RSUs in given nodes.

**Input parameters**
* `OSMmap` : MapData type from OpenStreetMapX package
* `range` : range of RSUs
* `throughput` : number of agents RSU can serve at once
* `div_coeff` : adjustment factor for calculating number of RSUs in node
"""
function calculate_RSU_location(OSMmap::MapData,
                                inAgents::Vector{Agent},
                                range::Float64,
                                throughput::Int64,
                                V2V_throughput::Int64;
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
`adjust_RSU_availability!` function adjust RSUs location and number to meet service availability and utilization criteria

**Input parameters**
* `OSMmap` : MapData type from OpenStreetMapX package
* `RSUs` : vector with RSUs used in simulation
* `failed_coor` : vector of vectors with agents coordinates missing an update
* `range` : range of RSUs
* `throughput` : number of agents RSU can serve at once
* `V2V_throughput` : throughput of V2V master to slaves communication
"""
function adjust_RSU_availability!(OSMmap::MapData,
                                RSUs::Vector{RSU},
                                failed_coor::Vector{Vector{ENU}},
                                range::Float64,
                                throughput::Int64,
                                V2V_throughput::Int64)
    RSU_count = sum(getfield.(RSUs,:count))
    RSU_ENU = getfield.(RSUs, :ENU) #Extract RSUs coordinates
    #Split set of coordinates according to reason of failure
    failed_throughput = Vector{Vector{ENU}}()
    failed_range = Vector{Vector{ENU}}()
    for failed_update in failed_coor
        tmp_range = Vector{ENU}()
        tmp_thput = Vector{ENU}()
        for enu in failed_update
            if any([OpenStreetMapX.distance(RSU,enu) <= range for RSU in RSU_ENU])
                push!(tmp_thput, enu)
            else
                push!(tmp_range, enu)
            end
        end
        push!(failed_throughput, tmp_thput)
        push!(failed_range, tmp_range)
    end
    #############################
    #Handle out of range failures
    #############################
    #For every failed coordinate find nodes within RSU range
    failed_range_dicts = Vector{Dict{ENU,Array{Int64,1}}}()
    for vec in failed_range
        new_dict = filter!(n->!isempty(n[2]), Dict([enu => filter(n-> haskey(OSMmap.v, n), nodes_within_range(OSMmap.nodes, enu, range)) for enu in vec]))
        push!(failed_range_dicts, new_dict)
    end
    #Repeat until failed points are in RSUs range
    while !all(isempty.(failed_range_dicts))
        nodes_count = StatsBase.countmap(collect(Iterators.flatten(Iterators.flatten(values.(failed_range_dicts)))))
        nodeID = findmax(nodes_count)[2]
        #Put RSU in given node
        max_to_serve = maximum(count.(n-> n == nodeID, collect.(Iterators.flatten.(values.(failed_range_dicts)))))
        N = ceil(max_to_serve/(throughput*V2V_throughput))
        new_RSU = RSU(nodeID, OSMmap.nodes[nodeID], N, N * throughput)
        push!(RSUs, new_RSU)
        #Delete all points in new RSU range
        for dict in failed_range_dicts filter!(n-> !in(nodeID, n[2]), dict) end
    end
    #############################
    #Handle agents who failed due to reached transfer limit
    #############################
    failed_thput_vecs = Vector{Vector{RSU}}()
    for vec in failed_throughput
        push!(failed_thput_vecs, [findmin(Dict([r => OpenStreetMapX.distance(enu, r.ENU) for r in RSUs]))[2] for enu in vec])
    end
    for rsu in unique(collect(Iterators.flatten(keys.(failed_thput_vecs))))
        if typeof(rsu) == RSU
            max_to_serve = maximum(count.(n-> n == rsu, failed_thput_vecs))
            N = ceil(max_to_serve/(throughput*V2V_throughput))
            rsu.count += N
            rsu.total_thput += N * throughput
        end
    end
    #Check if RSUs parameters were changed
    if RSU_count == sum(getfield.(RSUs,:count)) error("Availability criterion can't be met. Stopping simulation.") end
end

"""
`adjust_RSU_utilization!` function

**Input parameters**
* `RSUs` : vector with RSUs used in simulation
* `RSUs_util` : vector with RSUs percentage utilization in every weigths update
* `throughput` : number of agents RSU can serve at once
"""
function adjust_RSU_utilization!(RSUs::Vector{RSU},
                                RSUs_util::Vector{Dict{Int64, Float64}},
                                throughput::Int64)
    util_failed = false
    for rsu in filter(r-> r.count !=1,RSUs)
        max_util = maximum([dict[rsu.node] for dict in RSUs_util])
        N = floor(Int, (1.0 - max_util)*rsu.total_thput/throughput)
        if N != 0 && rsu.count - N > 0
            util_failed = true
            rsu.count -= N
            rsu.total_thput -= N * throughput
        end
    end
    return util_failed
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
`gather_statistics` return various measures describing ITS model quality

**Input parameters**
* `smart_ind` : vector with true flags for smart agents
* `times_base` : agents travelling time in base scenario
* `times_ITS` : agents travelling time in ITS scenario
* `srvc_avblty` : vector with percentage service availability (from `simulation_ITS`)
* `RSUs_util` : vector of dictionaries with RSUs utilization (from `simulation_ITS`)
* `RSUs` : vector of RSUs
"""
function gather_statistics(smart_ind::BitArray{1},
                            times_base::Vector{Float64},
                            times_ITS::Vector{Float64},
                            srvc_avblty::Vector{Float64},
                            RSUs_util::Vector{Dict{Int64, Float64}},
                            RSUs::Vector{RSU})
    overall_time = (sum(times_base) - sum(times_ITS))/sum(times_base)
    smart_time = (sum(times_base[smart_ind]) - sum(times_ITS[smart_ind]))/sum(times_base[smart_ind])
    other_time = (sum(times_base[.!smart_ind]) - sum(times_ITS[.!smart_ind]))/sum(times_base[.!smart_ind])
    statistics_tuple = (
        overall_time = round(overall_time, digits=3),
        smart_time = round(smart_time, digits=3),
        other_time = round(other_time, digits=3),
        service_availability = round(minimum(srvc_avblty), digits=3),
        RSUs_utilization = round(mean([mean(values(Dict)) for Dict in RSUs_util]), digits =3),
        RSU_count = sum(getfield.(RSUs,:count))
                        )
    return statistics_tuple
end
