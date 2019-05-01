##################################
## Setting up agents properties ##
##################################
Rect = Tuple{Tuple{Float64,Float64},Tuple{Float64,Float64}}
"""
`pick_random_node` function is used to set starting and ending node of agents.
Nodes are randomly chosen from set of rectangles corresponding to areas on map.

**Input parameters**
* `OSMmap` : mapData type object with road network data
* `rects` : vector of tuples with two Latitude-Longitude point interpreted as a set of rectangle areas

"""
function pick_random_node(OSMmap::OpenStreetMapX.MapData, rects::Vector{Rect})
    nodes_in_rects = Vector{Int}()
    for rect in rects
        frect = collect(Iterators.flatten(rect))
        p1 = ENU(LLA(frect[1], frect[2]), OSMmap.bounds)
        p2 = ENU(LLA(frect[3], frect[4]), OSMmap.bounds)
        exE = extrema([p1.east, p2.east])
        exN = extrema([p1.north, p2.north])
        for key in keys(OSMmap.v)
            if (exE[1] <= OSMmap.nodes[key].east <= exE[2] &&
                exN[1] <= OSMmap.nodes[key].north <= exN[2])
                push!(nodes_in_rects, key)
            end
        end
    end
    chosen_node = rand(unique!(nodes_in_rects))
    return chosen_node
end

"""
`generate_agents` function creating vector of agents and returning travel time
for initial routes travelled with maximal speed

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `N` : number of agents to be generated
* `StartArea` : vector of points corresponding to area from which agents randomly pick starting point
* `EndArea` : vector of points corresponding to area from which agents randomly pick ending point
* `α` : percentage of smart agentss
"""
function generate_agents(OSMmap::OpenStreetMapX.MapData, N::Int, StartArea::Vector{Rect}, EndArea::Vector{Rect}, α::Float64)
    #Initialize empty working variables
    AgentsArr = Vector{Agent}()
    times = Dict{Int,Float64}()
    dists = Dict{Int,Float64}()
    #Indicate smart agents
    N_int= Int(ceil(N*α))
    smart_ind = [trues(N_int); falses(N-N_int)]
    #Generating N agents
    for i in 1:N
        dist = Inf
        start_node = end_node = counter = time =  0
        init_route = Array{Int64,1}()
        while dist == Inf
            start_node = pick_random_node(OSMmap, StartArea)
            end_node = pick_random_node(OSMmap, EndArea)
            init_route, dist, time = fastest_route(OSMmap, start_node, end_node)
            counter +=1
            if counter == 100
                error("Route from starting to ending point can't be calculated.")
            end
        end
        times[i] = time
        dists[i] = dist
        #First edge in vertices notation
        firstEdge = [OSMmap.v[init_route[1]], OSMmap.v[init_route[2]]]
        NewAgent = Agent(smart_ind[i], start_node, end_node, init_route, 0.0, firstEdge, 0.0, true)
        push!(AgentsArr, NewAgent)
    end
    return AgentsArr, times, dists
end
