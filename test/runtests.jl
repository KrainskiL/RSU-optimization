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

Random.seed!(0);
@test pick_random_node(test_map, [Rect1]) == 580387849

Random.seed!(0);
@test pick_random_node(test_map, [Rect1, Rect2]) == 580387570

Random.seed!(0);
AgentsSet, AgentsTime = generate_agents(10, [Rect1], [Rect2], test_map)
@test AgentsSet[5].start_node == 140311966
@test AgentsTime[5] == 255.72200861225798

end

@testset "weights" begin
max_dens = get_max_densities(test_map, 5.0)
max_speeds = OpenStreetMapX.get_velocities(test_map)
speeds = deepcopy(max_speeds)

@test sum(max_dens) == 206315.88713471388

update_weights!(speeds, Dict((2,1)=>15,(3,4)=>60), max_dens, max_speeds)

@test speeds[2,1] < max_speeds[2,1] && speeds[3,4] < max_speeds[3,4] && speeds[487,1] == max_speeds[487,1]
@test speeds[2,1] == 5.12499971730625 && max_speeds[2,1] == 11.11111111111111
end

@testset "simulation" begin
Random.seed!(0);
iterations, totaltime, timediff = simulation(10, [Rect1], [Rect2], test_map)

@test iterations == 466
@test totaltime == 504.76176645787575
@test sum(timediff) == 20.520435962368882

end
