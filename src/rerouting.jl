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
* `V2V` : V2V hybrid model switch
* `V2V_range` : range of V2V communication
* `V2V_throughput` : throughput of V2V master to slaves communication
"""
function send_weights_update(Agents::Vector{Agent},
                            OSMmap::MapData,
                            RSUs::Vector{RSU},
                            range::Float64,
                            V2V::Bool,
                            V2V_range::Float64,
                            V2V_throughput::Int64)
    tmpRSUs = deepcopy(RSUs) #Creating working copy of RSUs
    #Vector marking if agent receive update in given iteration
    update_received = falses(length(Agents))
    #Vector with ENU coordinates of agents not receiving an update
    no_update = Vector{ENU}()
    #Get smart and active agent coordinates
    agents_coor = Dict{Int64,ENU}()
    for (i,a) in enumerate(Agents)
        if a.active && a.smart
            agents_coor[i] = get_agent_coordinates(OSMmap, a)
        end
    end
    smart_active = length(agents_coor)
    #If agent is in range of RSU with availabe transfer, update is received
    if !isempty(agents_coor)
        if V2V
            V2V_hybrid_transfer!(tmpRSUs, agents_coor, range, update_received, V2V_range, V2V_throughput)
        else
            V2I_transfer!(tmpRSUs, agents_coor, range, update_received)
        end
        no_update = collect(values(agents_coor))
    end
    #Service availability in current update
    updt_service_avblty = sum(update_received)/smart_active
    if smart_active == 0 updt_service_avblty = 1.0 end
    #Percentage RSUs utilization in current update
    updt_RSUs_utilization = Dict([RSUs[r].node => (RSUs[r].total_thput - tmpRSUs[r].total_thput)/RSUs[r].total_thput for r = 1:length(RSUs)])
    return update_received, no_update, updt_RSUs_utilization, updt_service_avblty
end


function V2I_transfer!(RSUs::Vector{RSU},
                       agents_position::Dict{Int64,ENU},
                       range::Float64,
                       update_received::BitArray{1})
    for rsu in RSUs
        for (k, v) in agents_position
            rsu.total_thput == 0 && break
            if OpenStreetMapX.distance(v, rsu.ENU) <= range
                update_received[k] = true
                #Decrease available throughput
                rsu.total_thput -= 1
                delete!(agents_position, k)
            end
        end
    end
end

function V2V_hybrid_transfer!(RSUs::Vector{RSU},
                       agents_position::Dict{Int64,ENU},
                       range::Float64,
                       update_received::BitArray{1},
                       V2V_range::Float64,
                       V2V_throughput::Int64)
    for rsu in RSUs
        #Find all agents in range
        in_range = Dict{Int64,Float64}()
        for (k, v) in agents_position
            dist = OpenStreetMapX.distance(v, rsu.ENU)
            if dist <= range
                in_range[k] = dist
            end
        end
        #Serve all agents in range if throughput is available
        while !isempty(in_range)
            rsu.total_thput == 0 && break
            #Serve agents furthest from RSU first
            ID = findmax(in_range)[2]
            update_received[ID] = true  #Master receive update
            rsu.total_thput -= 1 #Decrease throughput
            #Calculate other agents distances to master
            dist_to_master = [(k,OpenStreetMapX.distance(agents_position[ID], v)) for (k,v) in agents_position if k!=ID]
            #Filter agents in V2V range
            filter!(t-> t[2] <= V2V_range, dist_to_master)
            if !isempty(dist_to_master)
                #Sort list by distance
                sort!(dist_to_master, by = t-> t[2])
                to = min(length(dist_to_master), V2V_throughput)
                #Serve up to V2V_throughput agents
                for e in dist_to_master[1:to]
                    update_received[e[1]] = true
                    delete!(agents_position, e)
                    if haskey(in_range, e) delete!(in_range,e) end
                end
            end
            #Delete master
            delete!(agents_position, ID)
            delete!(in_range, ID)
        end
    end
end
