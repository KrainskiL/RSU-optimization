################################
## Types used in simulation ##
################################
"""
`Agent` type stores information about agents

**Fields**
* `smart` : logical value indicating if agent can communicate with RSUs
* `start_node` : starting point of agent's route
* `end_node` : ending point of agent's route
* `route` : array of nodes determining agent's route (may be changed by re-routing)
* `travel_time` : time spend in simulation
* `edge` : current edge agent is on
* `pos` : position on current edge
* `active` : indicates if agent is active in simulation
"""
mutable struct Agent
    smart::Bool
    start_node::Int64
    end_node::Int64
    route::Union{Array{Int64,1}, Nothing}
    travel_time::Float64
    edge::Vector{Int64}
    pos::Float64
    active::Bool
end

"""
`RSU` type stores information about set of Road Side units in one node

**Fields**
* `node` : ID of node where RSU is located
* `ENU` : ENU coordinates of the node
* `count` : number of units in node
* `total_thput` : total throughput of all units in the node
"""
mutable struct RSU
    node::Int64
    ENU::OpenStreetMapX.ENU
    count::Int64
    total_thput::Int64
end
