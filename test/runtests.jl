using Test
using OpenStreetMapX
using RSUOptimization
using Random
using SparseArrays

test_map = OpenStreetMapX.get_map_data("reno_east3.osm", use_cache = false)
Rect1 = ((39.50,-119.70),(39.55,-119.74))
Rect2 = ((39.50,-119.80),(39.55,-119.76))
AgentsSet, AgentsTime, AgentsDists = generate_agents(test_map,10,[Rect1],[Rect2], 0.5)

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
@test 1 == 1
end

#simulations.jl
@testset "simulations" begin

newAgents = generate_agents(test_map,10,[Rect1],[Rect2], 0.5)[1]
output = base_simulation(test_map, newAgents, 5.0)
@test length(output) == 4
@test typeof(output) == Tuple{Int64,Float64,Array{Float64,1},Dict{Array{Int64,1},Float64}}

end

#traffic_model.jl
@testset "traffic_model" begin
"""
V get_max_densities
V traffic_constants
V init_traffic_variables
V next_edge
V update_weights!
V update_event_agent!
V update_smart_densities!
V update_agents_position!
"""

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
i_densities, i_speeds, avg_s_densities = init_traffic_variables(test_map, AgentsSet, true)
@test sum(values(i_densities)) == 10
@test i_speeds == max_speeds
@test sum(values(avg_s_densities)) == 5

#next_edge tests
event = next_edge(AgentsSet, max_speeds, test_map.w)
@test typeof(event[2]) == Int64
@test typeof(event[1]) == Float64

#update_event_agent! tests
Agent1 = deepcopy(AgentsSet[1])
update_event_agent!(AgentsSet[1],event[1], i_densities, test_map.v)
@test AgentsSet[1].edge[1] == Agent1.edge[2]
@test length(AgentsSet[1].route) == length(Agent1.route) - 1

#update_smart_densities! tests
avg_before = deepcopy(avg_s_densities)
update_smart_densities!(AgentsSet, avg_s_densities, 10.0, 300.0, 10)
@test avg_before != avg_s_densities

#update_agents_position! tests
Agent2 = deepcopy(AgentsSet[2])
update_agents_position!(AgentsSet, event[1], max_speeds)
@test AgentsSet[2].pos != Agent2.pos
end

#optimization.jl
@testset "optimization" begin

avg_s_densities = init_traffic_variables(test_map, AgentsSet, true)[3]

RSUs = optimize_RSU_location(test_map, 100.0, 100, "cover_all_nodes", AgentsSet)
@test rand(keys(RSUs)) in keys(test_map.v)
@test typeof(sum(values(RSUs))) == Int64

@test typeof(get_agent_coordinates(test_map, AgentsSet[1])) == ENU
end
