using Test
using OpenStreetMapX
using Random

println(pwd())
map = OpenStreetMapX.get_map_data("reno_east3.osm", use_cache = false)

@testset "agents" begin

Random.seed!(0);
@test rand(Int) == -4635026124992869592

Rect1 = ((39.50,-119.70),(39.55,-119.74))
Random.seed!(0);
@test pick_random_node(map, Rect1) == 2750414408

Rect2 = ((39.50,-119.80),(39.55,-119.76))
Random.seed!(0);
@test pick_random_node(map, [Rect1, Rect2]) == 3052967037

Random.seed!(0);
AgentsSet, AgentsTime = generate_agents(10, [Rect1], [Rect2], map)
@test AgentsSet[5].start_node == 140551865
@test AgentsTime[5] == 343.3036686747631

end

@testset "weights" begin
max_dens = get_max_densities(map, 5.0)
max_speeds = OpenStreetMapX.get_velocities(map)
speeds = deepcopy(max_speeds)

@test sum(max_dens) == 206315.88713471388

update_weights!(speeds, Dict((2,1)=>15,(76,3)=>60), max_dens, max_speeds)

@test speeds[2,1] < max_speeds[2,1] && speeds[76,3] < max_speeds[76,3] && speeds[487,1] == max_speeds[487,1]
@test speeds[2,1] == 5.12499971730625 && max_speeds[2,1] == 11.11111111111111
end

@testset "simulation" begin
Random.seed!(0);
iterations, totaltime, timediff = simulation(10, [Rect1], [Rect2], map)

@test iterations == 550
@test totaltime == 446.9596470795234
@test sum(timediff) == 23.526540408410753

end
