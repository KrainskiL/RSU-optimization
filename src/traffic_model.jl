###################################
## Discrete events traffic model ##
###################################

function simulation(N::Int, StartArea::Vector{Rect}, EndArea::Vector{Rect}, map::MapData)
    Agents, inititaltime = generate_agents(N, StartArea, EndArea, map)
    AgentsCopy = deepcopy(Agents)
    active = ones(Int,1,N)
    traffictime = zeros(N)
    #Initital velocities on edges
    densities = countmap([a.edge for a in Agents])
    ##RSU Optimization module
    stats_densities = deepcopy(densities)
    ##
    max_densities = get_max_densities(map, 5.0)
    max_speeds = OpenStreetMapX.get_velocities(map)
    speeds = deepcopy(max_speeds)
    update_weights!(speeds, densities, max_densities, max_speeds)
    #Starting simulation
    simtime = Dict{Int, Float64}()
    counter = 0
    while sum(active) != 0
        counter += 1
        #Calculate next event time
        for i = 1:N
            if active[i] == 1
                A = Agents[i]
                simtime[i] = (map.w[A.edge[1], A.edge[2]] - A.pos)/
                speeds[A.edge[1], A.edge[2]]
            end
        end
        next_event, ID = findmin(simtime)
        vAgent = Agents[ID]
        #take agent from previous edge
        p_edge = vAgent.edge
        densities[p_edge] -= 1
        #change agent's route and current edge or remove if destination reached
        if length(vAgent.route[2:end]) == 1
            #disable agent
            traffictime[ID] = vAgent.travel_time + next_event
            vAgent.pos = 0.0
            active[ID] = 0
            simtime[ID] = Inf
            println("Active agents $(sum(active))")
            update_weights!(speeds, Dict(p_edge => densities[p_edge]),
                                        max_densities, max_speeds)
        else
            vAgent.route = vAgent.route[2:end]
            c_edge = [map.v[vAgent.route[1]], map.v[vAgent.route[2]]]
            vAgent.edge = c_edge
            vAgent.pos = 0.0
            vAgent.travel_time += next_event
            #add density on new edge
            haskey(densities, c_edge) ? densities[c_edge] += 1 : densities[c_edge] = 1
            ##RSU Optimization module
            if !haskey(stats_densities, c_edge) || densities[c_edge] > stats_densities[c_edge]
                stats_densities[c_edge] = densities[c_edge]
            end
            ##
            update_weights!(speeds, Dict(c_edge => densities[c_edge],
                                        p_edge => densities[p_edge]),
                                        max_densities, max_speeds)
        end
        #update other agents position
        for i = 1:N
            if active[i] == 1 && i != ID
                a = Agents[i]
                a.pos = a.pos + next_event*speeds[a.edge[1],a.edge[2]]
                a.travel_time += next_event
            end
        end
    end
    #Percentage difference in initial and real travel time
    timediff = [(traffictime[i]-inititaltime[i])/inititaltime[i]*100 for i in 1:N]
    return counter, maximum(traffictime), timediff, stats_densities, AgentsCopy
end


function get_max_densities(map::MapData, density_factor::Float64)
    roads_lanes = Dict{Int64,Int64}()
    for r in map.roadways
        OpenStreetMapX.haslanes(r) ? lanes = OpenStreetMapX.getlanes(r) : lanes = 1
        roads_lanes[r.id] = lanes
    end
    segments = OpenStreetMapX.find_segments(map.nodes, map.roadways, map.intersections)
    segments = [[map.v[s.node0], map.v[s.node1], roads_lanes[s.parent]] for s in segments]
    sparse_lanes = SparseArrays.sparse([x[1] for x in segments],
                                        [x[2] for x in segments],
                                        [x[3] for x in segments],
                                        length(map.v),length(map.v))
    return map.w .* sparse_lanes / density_factor
end

function update_weights!(speed_matrix::SparseMatrixCSC{Float64,Int64}, rho::Dict,
                        rho_max::SparseMatrixCSC{Float64,Int64}, V_max::SparseMatrixCSC{Float64,Int64}, V_min = 1.0)
    for (k,v) in rho
        speed_matrix[k[1],k[2]]  = (V_max[k[1],k[2]] - V_min)* max((1 - v/rho_max[k[1],k[2]]), 0.0) + V_min
    end
end


function simulation_ITS(map_data::MapData, Agents::Vector{Agent}, stats::Dict{Array{Int64,1},Int64},
    range::Float64, throughput::Int64, α::Float64, ϵ::Float64, update_period::Int64)
    #Find optimal RSUs location
    RSU_Dict = optimize_RSU_location(map_data, stats, range, throughput, α, ϵ)
    active = ones(Int,1,N)
    traffictime = zeros(N)
    next_update = update_period
    #Initital velocities on edges
    densities = countmap([a.edge for a in Agents])
    ##RSU Optimization module
    stats_densities = deepcopy(densities)
    ##
    max_densities = get_max_densities(map_data, 5.0)
    max_speeds = OpenStreetMapX.get_velocities(map_data)
    speeds = deepcopy(max_speeds)
    update_weights!(speeds, densities, max_densities, max_speeds)
    #Starting simulation
    simtime = Dict{Int, Float64}()
    counter = 0
    while sum(active) != 0
        counter += 1
        #Calculate next event time
        for i = 1:N
            if active[i] == 1
                A = Agents[i]
                simtime[i] = (map.w[A.edge[1], A.edge[2]] - A.pos)/
                speeds[A.edge[1], A.edge[2]]
            end
        end
        next_event, ID = findmin(simtime)
        #Check if weight updates occur before next_event time
        if next_update < next_event
            #Initialize tracking array for given update
            update_received = zeros(Int,N,2)
            RSUs_thput = Dict([k=>RSU_Dict[k]*throughput for k in keys(RSU_Dict)])
            #Update position of all agents
            for i = 1:N
                if active[i] == 1
                    a = Agents[i]
                    a.pos = a.pos + next_update*speeds[a.edge[1],a.edge[2]]
                    a.travel_time += next_update
                    #Mark all agents receiveing an update
                    #Find RSUs in which range agent is in
                    RSU_ENU = Dict([k=>map_data.nodes[k] for k in keys(RSU_Dict)])
                    Agent_coor = get_agent_coor(map_data, testAgent)
                    RSU_in_range = [k for (k,v) in RSU_ENU if OpenStreetMapX.distance(Agent_coor, v) < range]
                    if !isempty(RSU_in_range)
                        #Check if any throughput is available
                        in_range_thput = Dict(map(x->x=>RSUs_thput[x],RSU_in_range))
                        if all(values(in_range_thput) .== 0)
                            update_received[i,1] = -1
                        else
                            RSU_ID = findmax(in_range_thput)[2]
                            RSUs_thput[RSU_ID] -= 1
                            update_received[i,:] = [RSU_ID,1]
                            #Re-route module
                            #If k = 1 run deterministic algorithm
                            if k == 1
                                a.route[2:end] = OpenStreetMapX.fastest_route(map_data, a.route[2], a.end_node)[1]
                            else
                                
                            end
                            """
                            Jesli mark=1 run reroute from route[2] to end_node
                            1.Get k fastest routes and their distances
                            2.Calculate probs based on Boltzmann distr
                            3.Pick new route
                            4.Check if route was changed - if yes add 1 to rerouting count
                            5.continue
                            """
                        end
                    end
                end
            end
            #Calculate service availability
            service_avblty = sum(update_received[:,2])/sum(active)

        end
        vAgent = Agents[ID]
        #take agent from previous edge
        p_edge = vAgent.edge
        densities[p_edge] -= 1
        #change agent's route and current edge or remove if destination reached
        if length(vAgent.route[2:end]) == 1
            #disable agent
            traffictime[ID] = vAgent.travel_time + next_event
            vAgent.pos = 0.0
            active[ID] = 0
            simtime[ID] = Inf
            println("Active agents $(sum(active))")
            update_weights!(speeds, Dict(p_edge => densities[p_edge]),
                                        max_densities, max_speeds)
        else
            vAgent.route = vAgent.route[2:end]
            c_edge = [map.v[vAgent.route[1]], map.v[vAgent.route[2]]]
            vAgent.edge = c_edge
            vAgent.pos = 0.0
            vAgent.travel_time += next_event
            #add density on new edge
            haskey(densities, c_edge) ? densities[c_edge] += 1 : densities[c_edge] = 1
            ##RSU Optimization module
            if !haskey(stats_densities, c_edge) || densities[c_edge] > stats_densities[c_edge]
                stats_densities[c_edge] = densities[c_edge]
            end
            ##
            update_weights!(speeds, Dict(c_edge => densities[c_edge],
                                        p_edge => densities[p_edge]),
                                        max_densities, max_speeds)
        end
        #update other agents position
        for i = 1:N
            if active[i] == 1 && i != ID
                a = Agents[i]
                a.pos = a.pos + next_event*speeds[a.edge[1],a.edge[2]]
                a.travel_time += next_event
            end
        end
    end
    #Percentage difference in initial and real travel time
    timediff = [(traffictime[i]-inititaltime[i])/inititaltime[i]*100 for i in 1:N]
    return counter, maximum(traffictime), timediff, stats_densities, AgentsCopy
end
