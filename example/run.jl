using OpenStreetMapX
using RSUOptimization
using StatsBase
using LightGraphs
using SparseArrays

#Creating MapData object
mapfile = "reno_east3.osm"
datapath = "C:/RSUOptimization.jl/example";
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels= Set(1:4));

#Defining starting and ending area
Start = ((39.50,-119.70),(39.55,-119.74))
End = ((39.50,-119.80),(39.55,-119.76))

#Proportion of smart agents
α = 0.8
N = 1000
range = 300.0
#Generating agents
Agents, init_times, init_dists = generate_agents(map_data, N, [Start], [End], α)

#Running base simulation - no V2I system
@time BaseOutput = base_simulation(map_data, Agents, 5.0)
avg_density = BaseOutput[4]

@time ITSOutput = simulation_ITS(map_data, Agents, 5.0, avg_density, range, 100, 100, 1.0, 1)

RSU_Dict = ITSOutput[4]
RSU_ENU = [map_data.nodes[k] for k in keys(RSU_Dict)]
means, fail_reasons, out_of_range = ITS_quality_assess(getfield.(Agents, :smart),
                                        BaseOutput[3],
                                        ITSOutput[3],
                                        ITSOutput[5],
                                        ITSOutput[6],
                                        RSU_ENU,
                                        range,
                                        ITSOutput[7])
println(means)
println(out_of_range)
minimum(ITSOutput[5])
mixed_out_of_range = hcat([sum(e) for e in fail_reasons],
                            [sum(e)/length(e) for e in fail_reasons])
hcat([1,2,3],[4,5,6])
nodes_within_grid = unique(Iterators.flatten([OpenStreetMapX.nodes_within_range(map_data.nodes,n,range) for n in RSU_ENU]))
sum([all([n in nodes_within_grid for n in a.route]) for a in Agents if a.smart])
println([n in nodes_within_grid for n in Agents[5].route])

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
