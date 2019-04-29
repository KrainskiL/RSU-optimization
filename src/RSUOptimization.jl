module RSUOptimization

#packages
using OpenStreetMapX
using StatsBase
using SparseArrays
using Base.Iterators
using LightGraphs

#types
export Agent

#functions
#generate_agents.jl
export generate_agents, pick_random_node
#optimization.jl
export optimize_RSU_location, get_agent_coordinates, ITS_quality_assess
#rerouting.jl
export k_shortest_path_rerouting!, send_weights_update
#simulations.jl
export base_simulation, simulation_ITS
#traffic_model.jl
export get_max_densities, traffic_constants, init_traffic_variables, next_edge
export update_weights!, update_event_agent!, update_smart_densities!, update_agents_position!

#files
include("types.jl")
include("generate_agents.jl")
include("optimization.jl")
include("rerouting.jl")
include("simulations.jl")
include("traffic_model.jl")


end
