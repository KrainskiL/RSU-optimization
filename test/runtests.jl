using Test
using OpenStreetMapX
using Random

@testset "maps" begin

using Random
Random.seed!(0);
@test rand(Int) == -4635026124992869592

#working
pick_random_node(map_data, [((4.12,5.74),(5.63,6.23))])
#error
pick_random_node(map_data, [((5.63,6.23),(4.12,5.74))])
end
