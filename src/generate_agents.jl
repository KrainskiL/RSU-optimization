##################################
## Setting up agents properties ##
##################################

"""
`pick_random_node` function is used to set starting and ending node of agents.
Nodes are randomly chosen from set of rectangles corresponding to areas on map.

**Input parameters**
* `map` : MapData type from OpenStreetMapX package
* `rects` : vector of tuples with two Latitude-Longitude point interpreted as set of rectangles on map;
first point - rectangle bottom left corner
second point - top right corner
"""
SetRecs = Vector{Tuple{Tuple{Float64,Float64},Tuple{Float64,Float64}}}
function pick_random_node(map::MapData, rects::SetRecs)
#Check bottom left and top right points compliance
    [if !(rect[1][1] < rect[2][1] || rect[1][2] < rect[2][2])
    throw(DomainError(rect, "wrong coordinates definition")) end for rect in rects]
    nodes_in_rects = Vector{Int}()
    for rect in rects
        for node in map_data.nodes
            if (node.ENU.east >= ENU(LLA(rec[1][1]))

            end
        end
    end
    return chosen_node
end

collect([(5.6346,6.2341),(4.123,5.74343)])

pick_random_node(map_data, [((5.6346,6.2341),(4.123,5.74343))])
typeof(ans)
return 1 <=2

map_data.nodes[140002483]
