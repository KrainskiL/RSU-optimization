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

function optimize_RSU_location(map_data::MapData, stats::Dict{Array{Int64,1},Int64},
    range::Float64, throughput::Int64, α::Float64, ϵ::Float64)
    RSUs = Dict{Int64,Int64}()
    #Create working dictionary
    temp = deepcopy(stats)
    statsum = sum(values(stats))
    while sum(values(temp)) > (1-α + ϵ)*statsum
        #Divide traffic density by roads length - traffic per road unit
        unit_density = Dict{Array{Int64,1},Float64}()
        for k in keys(temp)
            unit_density[k] = temp[k]/map_data.w[k[1],k[2]]
        end
        #Gather unit traffic in nodes
        nodes_traffic = Dict{Int,Float64}()
        for (key,val) in temp
            map(x-> haskey(nodes_traffic,x) ? nodes_traffic[x]+=val : nodes_traffic[x]=val, key)
        end
        #Node with highest aggregated traffic
        maxnode = findmax(nodes_traffic)[2]
        maxnode_mapv = [k for (k,v) in map_data.v if v == maxnode][1]
        #Find nodes within RSU range from chosen node
        rng_nodes = OpenStreetMapX.nodes_within_range(map_data.nodes,map_data.nodes[maxnode_mapv], range)
        rng_nodes = [map_data.v[k] for k in rng_nodes if haskey(map_data.v,k)]
        #Transform nodes within range into edges
        rng_edges = [collect(p) for p in Iterators.product(rng_nodes,rng_nodes) if haskey(temp, collect(p)) && temp[collect(p)] != 0]
        if isempty(rng_edges) rng_edges = [k for k in keys(temp) if maxnode in k] end
        #Sum traffic on edges in range
        N = sum([temp[e] for e in rng_edges])
        #Place new RSUs in node
        RSUs[maxnode_mapv] = ceil(N/(0.5*length(rng_edges)*throughput))
        #Delete all traffic in covered edges
        for e in rng_edges temp[e] = 0 end
    end
    return RSUs
end

function get_agent_coor(map_data::MapData, inAgent::Agent)
    if inAgent.pos == 0.0 return map_data.nodes[inAgent.route[1]] end
    pA = map_data.nodes[inAgent.route[1]]
    pB = map_data.nodes[inAgent.route[2]]
    rel_pos = inAgent.pos/map_data.w[inAgent.edge[1],inAgent.edge[2]]
    Agent_coor = ENU(pA.east+(pB.east-pA.east)*rel_pos,
                    pA.north+(pB.north-pA.north)*rel_pos,
                    pA.up+(pB.up-pA.up)*rel_pos)
    return Agent_coor
end
