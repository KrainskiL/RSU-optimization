module RSUOptimization

#packages
using OpenStreetMapX
using StatsBase
using SparseArrays
using Base.Iterators
using LightGraphs
using DataFrames

#types
export Rect, Agent, RSU

#functions
#generate_agents.jl
export generate_agents, pick_random_node
#optimization.jl
export calculate_RSU_location, adjust_RSU_availability!, adjust_RSU_utilization!, get_agent_coordinates, gather_statistics
#rerouting.jl
export k_shortest_path_rerouting!, send_weights_update
#simulations.jl
export base_simulation, simulation_ITS, iterative_simulation_ITS
#traffic_model.jl
export get_max_densities, traffic_constants, init_traffic_variables, next_edge
export update_weights!, update_event_agent!, update_agents_position!

#files
include("types.jl")
include("generate_agents.jl")
include("optimization.jl")
include("rerouting.jl")
include("simulations.jl")
include("traffic_model.jl")

end
