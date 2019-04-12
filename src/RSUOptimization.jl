module RSUOptimization

#packages
using OpenStreetMapX
using StatsBase
using SparseArrays

#types
export Agent, Rect

#functions
export generate_agents, pick_random_node
export get_max_densities, update_weights!
export simulation

#files
include("generate_agents.jl")
include("types.jl")
include("traffic_model.jl")

end
