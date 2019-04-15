###################################
## Discrete events traffic model ##
###################################

function simulation(N::Int, StartArea::Vector{Rect}, EndArea::Vector{Rect}, map::MapData)
    Agents, inititaltime = generate_agents(N, StartArea, EndArea, map)
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

    #Znajdz node z najwiekszym zagregowanych ruchem
    #Sprawdz ile jest ruchu w zasiegu z tego punktu wstaw ceil(N/limit) RSU
    #Usun obsluzony ruch z nodow
    #POWTORZ AZ sum(values(traffic_dict)) == 0

    ##RSU Optimization module
    function optimize_RSU_location(map::MapData, stats::Dict{Array{Int64,1},Int64},
        range::Float64, throughput::UInt64, α::Float64, ϵ::Float64)
        RSUs = Dict{Int64,Int64}()
    #Gather traffic in nodes
    nodes_traffic = Dict{Int,Int}()
    for (key,val) in stats
        map(x-> haskey(nodes_traffic,x) ? nodes_traffic[x]+=val : nodes_traffic[x]=val, key)
    end
    #Place new RSUs in node with highest traffic
    traffic, node = findmax(nodes_traffic)
    RSUs[node] = ceil(traffic/range)
    end
    return counter, maximum(traffictime), timediff, stats_densities
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
