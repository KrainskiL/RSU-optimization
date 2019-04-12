using Test
using OpenStreetMapX
using RSUOptimization
using Random
using SparseArrays

println(pwd())
test_map = OpenStreetMapX.get_map_data("reno_east3.osm", use_cache = false)
Rect1 = ((39.50,-119.70),(39.55,-119.74))
Rect2 = ((39.50,-119.80),(39.55,-119.76))

@testset "agents" begin

Random.seed!(0);
@test rand(Int) == -4635026124992869592

@test in(pick_random_node(test_map, [Rect1]), keys(test_map.nodes))

@test in(pick_random_node(test_map, [Rect1, Rect2]), keys(test_map.nodes))

AgentsSet, AgentsTime = generate_agents(10, [Rect1], [Rect2], test_map)
@test all([in(x, keys(test_map.nodes)) for x in getfield.(AgentsSet,:start_node)])
@test AgentsTime[5] == OpenStreetMapX.fastest_route(test_map, AgentsSet[5].start_node, AgentsSet[5].end_node)[3]

end

@testset "weights" begin
max_dens = get_max_densities(test_map, 5.0)
max_speeds = OpenStreetMapX.get_velocities(test_map)
speeds = deepcopy(max_speeds)

@test round(sum(max_dens);digits=3) == 206315.887

update_weights!(speeds, Dict((2,1)=>15,(3,4)=>60), max_dens, max_speeds)

@test speeds[2,1] < max_speeds[2,1] && speeds[3,4] < max_speeds[3,4] && speeds[487,1] == max_speeds[487,1]
@test round(speeds[2,1];digits=3) == 5.125 && round(max_speeds[2,1];digits=3) == 11.111
end

@testset "simulation" begin
iterations, totaltime, timediff = simulation(10, [Rect1], [Rect2], test_map)

@test typeof(iterations) == Int64
@test typeof(totaltime) == Float64
@test typeof(timediff) == Vector{Float64}

end
