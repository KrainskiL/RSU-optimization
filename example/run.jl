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
α = 1.0
N = 1000

#Generating agents
Agents, init_times, init_dists = generate_agents(map_data, N, [Start], [End], α)

#Running base simulation - no V2I system
@time BaseOutput = base_simulation(map_data, Agents, 5.0)
avg_density = BaseOutput[4]
@time ITSOutput = simulation_ITS(map_data, Agents, 5.0, avg_density, 300.0, 100, 100, 1.0, 3)

using Plots
histogram((BaseOutput[3].-ITSOutput[3])./BaseOutput[3])
mean((BaseOutput[3].-ITSOutput[3])./BaseOutput[3])

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
