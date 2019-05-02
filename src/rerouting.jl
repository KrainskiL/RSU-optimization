##################################
## Rerouting module ##
##################################

"""
`k_shortest_path_rerouting!` change agents current route according to traffic conditions

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `inAgent` : rerouting agent
* `speeds` : current speeds matrix
* `k` : number of fastest routes returned
* `T` : control variable for probability distribution
"""
function k_shortest_path_rerouting!(OSMmap::MapData,
                                    inAgent::Agent,
                                    speeds::AbstractMatrix,
                                    k::Int64,
                                    T::Float64)
    k_routes = LightGraphs.yen_k_shortest_paths(OSMmap.g, OSMmap.v[inAgent.route[2]],
                                                OSMmap.v[inAgent.end_node], OSMmap.w./speeds, k)
    if k == 1
        new_path = k_routes.paths[1]
    else
        #Normalize k-paths travelling time
        norm_time = k_routes.dists/maximum(k_routes.dists)
        #Calculate probability of being picked for every route
        exp_ntime = exp.(-norm_time/T)
        probs = exp_ntime/sum(exp_ntime)
        #Assign new route
        new_path = sample(k_routes.paths, StatsBase.weights(probs))
    end
    nodes_new_path = map(i-> OSMmap.n[i], new_path)
    inAgent.route = [inAgent.route[1]; nodes_new_path]
end

"""
`send_weights_update` function mark agents receiving weights update

**Input parameters**
* `Agents` : set of agents created with generate_agents function
* `OSMmap` : MapData type object with road network data
* `RSUs` : vector with RSUs objects
* `range` : range of RSUs transfer
"""
function send_weights_update(Agents::Vector{Agent},
                            OSMmap::MapData,
                            RSUs::Vector{RSU},
                            range::Float64)
    tmpRSUs = deepcopy(RSUs) #Creating working copy of RSUs
    #Vector marking if agent receive update in given iteration
    update_received = falses(length(Agents))
    #Vector with ENU coordinates of agents not receiving an update
    no_update = Vector{ENU}()
    smart_active = 0
    for (i,a) in enumerate(Agents)
        #Send updates only to smart and active agents
        if a.active && a.smart
            smart_active += 1
            #Get agent coordinates
            a_coor = get_agent_coordinates(OSMmap, a)
            #If agent is in range of RSU with availabe transfer update is received
            for rsu in tmpRSUs
                if rsu.total_thput > 0 && OpenStreetMapX.distance(a_coor, rsu.ENU) <= range
                    update_received[i] = true
                    #Decrease available throughput
                    rsu.total_thput -= 1
                    break
                end
            end
            if !update_received[i] push!(no_update, a_coor) end
        end
    end
    #Service availability in current update
    updt_service_avblty = sum(update_received)/smart_active
    #Percentage RSUs utilization in current update
    updt_RSUs_utilization = Dict([RSUs[r].node => (RSUs[r].total_thput - tmpRSUs[r].total_thput)/RSUs[r].total_thput for r = 1:length(RSUs)])
    return update_received, no_update, updt_RSUs_utilization, updt_service_avblty
end
