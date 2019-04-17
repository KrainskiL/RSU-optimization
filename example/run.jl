using OpenStreetMapX
using RSUOptimization
using StatsBase
using LightGraphs
mapfile = "reno_east3.osm"
datapath = "C:/RSUOptimization.jl/example";
map_data = get_map_data(datapath, mapfile,use_cache=false; road_levels= Set(1:4));

Start = ((39.50,-119.70),(39.55,-119.74))
End = ((39.50,-119.80),(39.55,-119.76))

@time output = simulation(1000, [Start], [End], map_data)
RSU_Dict = optimize_RSU_location(map_data,output[4],600.0,1000,1.0,0.05)
output2 = simulation_ITS
a=zeros(Int,10,2)

LightGraphs.yen_k_shortest_paths(map_data.g, map_data.v[testAgent.start_node], map_data.v[testAgent.end_node],map_data.w,2)
fastest_route(map_data,testAgent.start_node,testAgent.end_node)
map_data.v[3625688614]
all(values(in_range_thput2) .== 1)
OpenStreetMapX.distance.(get_agent_coor(map_data, testAgent), [map_data.nodes[k] for k in keys(RSU_Dict)])
map_data.n[519]
"""
Ns = [10, 100, 500, 1000, 2000]
ResultsVec = Vector()
for element in Ns
    output = simulation(element, map_data)
    push!(ResultsVec, output)
    @info "simulation with $element agents done"
end


print(mean_and_std.([ResultsVec[i][3] for i in 1:length(ResultsVec)]))
for i in 1:length(ResultsVec)
    print(quantile(ResultsVec[i][3] ,[0.0, 0.25, 0.5, 0.75, 1.0]))
end
"""
