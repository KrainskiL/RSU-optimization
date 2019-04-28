#####################################
## Optimization auxilary functions ##
#####################################

"""
`optimize_RSU_location` function returns dictionary with number of RSUs in given nodes.

**Input parameters**
* `map` : MapData type from OpenStreetMapX package
* `stats` : dictionary with highest traffic density on edges during base simulation run
* `range` : range of RSUs
* `throughput` : number of agents RSU can serve at once
* `α` : required service availabilty/data coverage α ∈ <0,1>
* `ϵ` : service availabilty tolerance - all weights updates must be within α ± ϵ
"""

function optimize_RSU_location(OSMmap::MapData,
                                stats::Dict{Array{Int64,1},Int64},
                                range::Float64,
                                throughput::Int64)
    RSUs = Dict{Int64,Int64}()
    #Create working dictionary
    temp = deepcopy(stats)
    while sum(values(temp)) > 0
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
        rng_edges = [p for p in edges_product if haskey(temp, p) && temp[p] != 0]
        if isempty(rng_edges) rng_edges = [k for k in keys(temp) if maxvertex in k] end
        #Sum traffic on edges in range
        N = sum([temp[e] for e in rng_edges])
        #Place new RSUs in node
        RSUs[maxnode] = ceil(N/(0.5*length(rng_edges)*throughput))
        #Delete all traffic in covered edges
        for e in rng_edges temp[e] = 0 end
    end
    return RSUs
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
