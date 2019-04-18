module RSUOptimization

#packages
using OpenStreetMapX
using StatsBase
using SparseArrays
using Base.Iterators

#types
export Agent

#functions
export generate_agents, pick_random_node
export get_max_densities, update_weights!
export base_simulation, simulation_ITS
export optimize_RSU_location, get_agent_coor

#files
include("generate_agents.jl")
include("types.jl")
include("traffic_model.jl")
include("optimization.jl")


end
