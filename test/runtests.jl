using Test
using OpenStreetMapX
import LightGraphs

@testset "maps" begin

m = OpenStreetMapX.get_map_data("example/reno_east3.osm",use_cache=false);

@test length(m.nodes) == 9032

using Random
Random.seed!(0);
@test rand(Int) == -4635026124992869592

end
