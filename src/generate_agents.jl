##################################
## Setting up agents properties ##
##################################

"""
`pick_random_node` function is used to set starting and ending node of agents.
Nodes are randomly chosen from set of rectangles corresponding to areas on map.

**Input parameters**
* `map` : MapData type from OpenStreetMapX package
* `rects` : vector of tuples with two Latitude-Longitude point interpreted as a set of rectangle areas;

"""
Rect = Tuple{Tuple{Float64,Float64},Tuple{Float64,Float64}}
function pick_random_node(map::MapData, rects::Vector{Rect})
    nodes_in_rects = Vector{Int}()
    for rect in rects
        frect = collect(Iterators.flatten(rect))
        p1 = ENU(LLA(frect[1], frect[2]), map.bounds)
        p2 = ENU(LLA(frect[3], frect[4]), map.bounds)
        exE = extrema([p1.east, p2.east])
        exN = extrema([p1.north, p2.north])
        for key in keys(map.v)
            if (exE[1] <= map.nodes[key].east <= exE[2] &&
                exN[1] <= map.nodes[key].north <= exN[2])
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
* `N` : number of agents to be generated

"""
function generate_agents(N::Int, StartArea::Vector{Rect}, EndArea::Vector{Rect}, map::MapData)
    AgentsArr = Vector{Agent}()
    times = Dict{Int,Float64}()
    #Generating N agents
    for i in 1:N
        dist = Inf
        start_node = end_node = counter = time =  0
        init_route = Array{Int64,1}()
        while dist == Inf
            start_node = pick_random_node(map, StartArea)
            end_node = pick_random_node(map, EndArea)
            init_route, dist, time = fastest_route(map, start_node, end_node)
            counter +=1
            if counter == 100
                error("Route from starting to ending point can't be calculated.")
            end
        end
        times[i] = time
        firstEdge = [map.v[init_route[1]], map.v[init_route[2]]]
        NewAgent = Agent(start_node, end_node, init_route, 0.0, firstEdge, 0.0)
        push!(AgentsArr, NewAgent)
    end
    return AgentsArr, times
end
