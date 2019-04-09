###################################
## Discrete events traffic model ##
###################################

function simulation(Agents::Vector{Agent}, map::MapData)
    #Initital velocities on edges
    densities = countmap([[map.v[x.route[1]],map.v[x.route[2]]] for x in Agents])
    max_densities = get_max_densities(map, 5.0)
    max_speeds = OpenStreetMapX.get_velocities(map)
    speeds = deepcopy(max_speeds)
    update_weights!(speeds, densities, max_densities, max_speeds)
    #Starting simulation
    simtime = 0

    times_to_event = map(Agents) do x
        edge_start, edge_end = [map.v[agent.pos[1][1]], map.v[agent.pos[1][2]]]
        time_to_event = (map.w[edge_start, edge_end] - agent.pos[2])/speeds[edge_start, edge_end]
        return Dict(time_to_event => x.ID)
    end
    next_event = minimum(times_to_event)
    #Debug
    println(next_event)

    return speeds
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



times_to_event = map(AgentsArr) do x
    edge_start, edge_end = [map_data.v[x.pos[1][1]], map_data.v[x.pos[1][2]]]
    time_to_event = (map_data.w[edge_start, edge_end] - x.pos[2])/speeds[edge_start, edge_end]
    return (x.ID,time_to_event)
end
times_to_event = Dict(times_to_event)
focusID = findmin(times_to_event)[2]
densities = countmap([[map_data.v[x.route[1]],map_data.v[x.route[2]]] for x in AgentsArr])
haskey(densities,[,1966])
