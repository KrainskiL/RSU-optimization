################################
## Types used in simulation ##
################################
"""
Fields indicators:
[C]onstant
[V]ariable
"""
"""
'Agent' type stores information about agents

**Fields**
[C] 'ID' : unique identificator
[C] 'start_node' : starting point of agent's route
[C] 'end_node' : ending point of agent's route
[V] 'route' : array of nodes determining agent's route (may be changed by re-routing)
"""

mutable struct Agent
    ID::Int64
    start_node::Int64
    end_node::Int64
    route::Union{Vector{Int64}, Nothing}
end

"""
'RSU' type stores information about Road Side Units - V2I infrastructure

**Fields**
[C] 'ID' : unique identificator
[C] 'Location' : node in which RSU is located
"""

struct RSU
    ID::Int64
    Location::Int64
end
