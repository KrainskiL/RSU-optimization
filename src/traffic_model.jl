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
        for agent in Agents
            edge_start, edge_end = collect(map.v[agent.pos[1]])
                next_event = min([map.w[x.pos[1],x.pos[2]]/speeds[x.pos[1],x.pos[2]] for x in Agents])
        end
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
