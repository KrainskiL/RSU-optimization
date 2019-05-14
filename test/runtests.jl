using Test
using OpenStreetMapX
using RSUOptimization
using Random
using SparseArrays

test_map = OpenStreetMapX.get_map_data("C:/RSUOptimization.jl/example/reno_east3.osm", use_cache = false)
Rect1 = ((39.50,-119.70),(39.55,-119.74))
Rect2 = ((39.50,-119.80),(39.55,-119.76))
AgentsSet, AgentsTime, AgentsDists = generate_agents(test_map,10,[Rect1],[Rect2], 0.5)

rangeRSU = 300.0
throughput = 10

#generate_agents.jl
@testset "agents" begin

Random.seed!(0);
@test rand(Int) == -4635026124992869592

@test in(pick_random_node(test_map, [Rect1]), keys(test_map.nodes))

@test in(pick_random_node(test_map, [Rect1, Rect2]), keys(test_map.nodes))

@test all([in(x, keys(test_map.nodes)) for x in getfield.(AgentsSet,:start_node)])
@test AgentsTime[5] == OpenStreetMapX.fastest_route(test_map, AgentsSet[5].start_node, AgentsSet[5].end_node)[3]
@test sum(getfield.(AgentsSet,:smart)) == 5

end

#rerouting.jl
@testset "rerouting" begin

constantNode = AgentsSet[1].route[1]
k_shortest_path_rerouting!(test_map, AgentsSet[1], OpenStreetMapX.get_velocities(test_map), 3, 1.0)
@test AgentsSet[1].route[1] == constantNode

RSUs = calculate_RSU_location(test_map, AgentsSet, rangeRSU, throughput)

updateResults = send_weights_update(AgentsSet, test_map, RSUs, rangeRSU)
@test typeof(updateResults) == Tuple{BitArray{1},Array{ENU,1},Dict{Int64,Float64},Float64}
end

#simulations.jl
@testset "simulations" begin

newAgents = generate_agents(test_map,10,[Rect1],[Rect2], 0.5)[1]
output, tracking = base_simulation(test_map, newAgents, debug_level = 0)
@test length(output) == 3
@test typeof(tracking) == Dict{Int64,Array{Tuple,1}}
@test typeof(output) == NamedTuple{(:Steps, :Simtime, :TravelTimes),Tuple{Int64,Float64,Array{Float64,1}}}

RSUs = calculate_RSU_location(test_map, newAgents, rangeRSU, throughput)
ITSOutput, ITStracking = simulation_ITS(test_map, newAgents, rangeRSU, RSUs, 50, 1.0, 3, 5.0, 0)

@test length(ITSOutput) == 6
@test typeof(ITStracking) == Dict{Int64,Array{Tuple,1}}
@test typeof(ITSOutput) == NamedTuple{(:Steps, :Simtime, :TravelTimes, :ServiceAvailability, :RSUsUtilization, :FailedUpdates),Tuple{Int64,Float64,Array{Float64,1},Array{Float64,1},Array{Dict{Int64,Float64},1},Array{Array{ENU,1},1}}}

iterITSOutput, NewRSUs = iterative_simulation_ITS(test_map, newAgents, rangeRSU, throughput, 50, threshold = 0.60, debug_level = 0)

@test length(iterITSOutput) == 6
@test typeof(iterITSOutput) == typeof(ITSOutput)

@test typeof(NewRSUs) == Vector{RSU}

end

#traffic_model.jl
@testset "traffic_model" begin

max_dens = get_max_densities(test_map, 5.0)
max_speeds = OpenStreetMapX.get_velocities(test_map)
speeds = deepcopy(max_speeds)
#get_max_densities tests
@test round(sum(max_dens);digits=3) == 206315.887

update_weights!(speeds, Dict((2,1)=>15,(3,4)=>60), max_dens, max_speeds)
#update_weights! tests
@test speeds[2,1] < max_speeds[2,1] && speeds[3,4] < max_speeds[3,4] && speeds[487,1] == max_speeds[487,1]
@test round(speeds[2,1];digits=3) == 5.125 && round(max_speeds[2,1];digits=3) == 11.111

#traffic_constants tests
max_d, max_s = traffic_constants(test_map, 5.0)
@test max_d == max_dens && max_s == max_speeds

#init_traffic_variables tests
i_densities, i_speeds = init_traffic_variables(test_map, AgentsSet)
@test sum(values(i_densities)) == 10
@test i_speeds == max_speeds

#next_edge tests
event = next_edge(AgentsSet, max_speeds, test_map.w)
@test typeof(event[2]) == Int64
@test typeof(event[1]) == Float64

#update_event_agent! tests
Agent1 = deepcopy(AgentsSet[1])
update_event_agent!(AgentsSet[1],event[1], i_densities, test_map.v)
@test AgentsSet[1].edge[1] == Agent1.edge[2]
@test length(AgentsSet[1].route) == length(Agent1.route) - 1

#update_agents_position! tests
Agent2 = deepcopy(AgentsSet[2])
update_agents_position!(AgentsSet, event[1], max_speeds)
@test AgentsSet[2].pos != Agent2.pos
end

#optimization.jl
@testset "optimization" begin


RSUs = calculate_RSU_location(test_map, AgentsSet, rangeRSU, throughput)
@test rand(RSUs).node in keys(test_map.v)
@test typeof(RSUs[1]) == RSU

ITSOutput, ITStracking = simulation_ITS(test_map, AgentsSet, rangeRSU, RSUs, 50, 1.0, 3, 5.0, 0)
oldRSUs = deepcopy(RSUs)
adjust_RSU_availability!(test_map, RSUs, ITSOutput.FailedUpdates, rangeRSU, throughput)
@test oldRSUs != RSUs

failed_utilization = adjust_RSU_utilization!(RSUs, ITSOutput.RSUsUtilization, throughput)
@test typeof(failed_utilization) == Bool

@test typeof(get_agent_coordinates(test_map, AgentsSet[1])) == ENU

output, tracking = base_simulation(test_map, AgentsSet, debug_level = 0)
stats = gather_statistics(getfield.(AgentsSet,:smart),
                    output.TravelTimes,
                    ITSOutput.TravelTimes,
                    ITSOutput.ServiceAvailability,
                    ITSOutput.RSUsUtilization,
                    RSUs)

@test length(stats) == 6
@test typeof(stats) == NamedTuple{(:overall_time, :smart_time, :other_time, :service_availability, :RSUs_utilization, :RSU_count),Tuple{Float64,Float64,Float64,Float64,Float64,Int64}}
end
