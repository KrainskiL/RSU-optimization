################################
## Types used in simulation ##
################################
"""
`Agent` type stores information about agents

**Fields**
* `ID` : unique identificator
* `start_node` : starting point of agent's route
* `end_node` : ending point of agent's route
* `route` : array of nodes determining agent's route (may be changed by re-routing)
* `travel_time` : time spend in simulation
* `edge` : dictionary with current edge agent represented by vertices and nodes
* `pos` : position on current edge
"""

mutable struct Agent
    ID::Int64
    start_node::Int64
    end_node::Int64
    route::Union{Array{Int64,1}, Nothing}
    travel_time::Float64
    edge::Dict{String,Int}
    pos::Float64
end

"""
`RSU` type stores information about Road Side Units - V2I infrastructure

**Fields**
* [C] `ID` : unique identificator
* [C] `Location` : node in which RSU is located
"""

struct RSU
    ID::Int64
    Location::Int64
end

"""
`Rect` type is used to pass rectangle-shape areas to generate_agents function
"""

Rect = Tuple{Tuple{Float64,Float64},Tuple{Float64,Float64}}
