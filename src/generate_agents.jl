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
#Check bottom left and top right points compliance
    [if (rect[1][1] == rect[2][1] || rect[1][2] == rect[2][2])
    throw(DomainError(rect, "coordinates not defining a rectangle")) end for rect in rects]
    nodes_in_rects = Vector{Int}()

    for rect in rects
        frect = collect(Iterators.flatten(rect))
        p1 = ENU(LLA(frect[1], frect[2]), map_data.bounds)
        p2 = ENU(LLA(frect[3], frect[4]), map_data.bounds)
        exE = extrema([p1.east, p2.east])
        exN = extrema([p1.north, p2.north])
        for key in keys(map_data.v)
            if (exE[1] <= map_data.nodes[key].east <= exE[2] &&
                exN[1] <= map_data.nodes[key].north <= exN[2])
                push!(nodes_in_rects, key)
            end
        end
    end
    chosen_node = rand(unique!(nodes_in_rects))
    return chosen_node
end

function pick_random_node(map::MapData, rects::Rect)
    pick_random_node(map, [rects])
end
