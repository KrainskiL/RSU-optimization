###################################
## Discrete events traffic model ##
###################################

function simulation(N::Int, map::MapData)
    Agents, inititaltime = generate_agents(N)
    traffictime = Dict{Int,Float64}()
    #Initital velocities on edges
    densities = countmap([[a.edge["start_v"],a.edge["end_v"]] for a in Agents])
    max_densities = get_max_densities(map, 5.0)
    max_speeds = OpenStreetMapX.get_velocities(map)
    speeds = deepcopy(max_speeds)
    update_weights!(speeds, densities, max_densities, max_speeds)
    #Starting simulation
    simtime = 0
    counter = 0
    while length(Agents) != 0
        counter += 1
        times_to_event = Dict([ (a.ID,
                            (map.w[a.edge["start_v"], a.edge["end_v"]] - a.pos)/
                            speeds[a.edge["start_v"], a.edge["end_v"]]) for a in Agents])

        next_event, vID = findmin(times_to_event)
        #Debug
        vAgent = Agents[getfield.(Agents,:ID).== Int(vID)][1]
        #take agent from previous edge
        pedgekey = [vAgent.edge["start_v"], vAgent.edge["end_v"]]
        densities[pedgekey] -= 1
        #change agent's route and current edge or remove if destination reached
        if length(vAgent.route[2:end]) == 1
            #remove agent
            traffictime[vAgent.ID]=vAgent.travel_time
            Agents = Agents[.!(getfield.(Agents,:ID).== vID)]
            update_weights!(speeds, Dict(pedgekey => densities[pedgekey]),
                                        max_densities, max_speeds)
        else
            vAgent.route = vAgent.route[2:end]
            currEdge = Dict("start_v"=> map.v[vAgent.route[1]],
                            "end_v"=> map.v[vAgent.route[2]])
            vAgent.edge = currEdge
            vAgent.pos = 0.0
            vAgent.travel_time += next_event
            #add density on new edge
            cedgekey = [vAgent.edge["start_v"], vAgent.edge["end_v"]]
            haskey(densities,cedgekey) ? densities[cedgekey] += 1 : densities[cedgekey] = 1
            update_weights!(speeds, Dict(cedgekey => densities[cedgekey],
                                        pedgekey => densities[pedgekey]),
                                        max_densities, max_speeds)
        end

        #update agents position
        for a in Agents[.!(getfield.(Agents,:ID).== vID)]
            a.pos = a.pos + next_event*speeds[a.edge["start_v"], a.edge["end_v"]]
            a.travel_time += next_event
        end
        simtime += next_event
    end
    #Percentage difference in initial and real travel time
    timediff = [(traffictime[i]-inititaltime[i])/inititaltime[i]*100 for i in 1:N]
    return counter, simtime, timediff
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
                                        [x[3] for x in segments])
    return map.w .* sparse_lanes / density_factor
end

function update_weights!(speed_matrix::SparseMatrixCSC{Float64,Int64}, rho::Dict,
                        rho_max::SparseMatrixCSC{Float64,Int64}, V_max::SparseMatrixCSC{Float64,Int64}, V_min = 1.0)
    for (k,v) in rho
        speed_matrix[k[1],k[2]]  = (V_max[k[1],k[2]] - V_min)* max((1 - v/rho_max[k[1],k[2]]), 0.0) + V_min
    end
end
