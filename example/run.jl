using OpenStreetMapX
using RSUOptimization
using StatsBase
using LightGraphs
using SparseArrays
using Serialization
#Creating MapData object
mapfile = "reno_east3.osm"
datapath = "C:/RSUOptimization.jl/example";
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels= Set(1:4));

#Defining starting and ending area
Start = ((39.50,-119.70),(39.55,-119.74))
End = ((39.50,-119.80),(39.55,-119.76))

#Proportion of smart agents
α = 0.9
N = 1000
density_factor = 5.0
range = 400.0
throughput = 100
updt_period = 200

#Generating agents
Agents, init_times, init_dists = generate_agents(map_data, N, [Start], [End], α)

#Running base simulation - no V2I system
@time BaseOutput = base_simulation(map_data, Agents, density_factor)
avg_density = BaseOutput[4]
open("test2.bin", "w") do f
    serialize(f, avg_density)
end
avg1 = deserialize(open("test2.bin"))
RSU_ENU, Agents, failedENU = deserialize(open("packet.bin"))
#@time ITSOutput = simulation_ITS(map_data, Agents, density_factor, avg_density, range, throughput, updt_period, 1.0, 1)
@time ITSOutput, RSU_Dict = iterative_simulation_ITS(map_data, Agents, density_factor, avg_density, range, throughput, updt_period, 1.0, 2, 0.95, true)

ITSOutput
RSU_Dict

RSU_ENU = [map_data.nodes[k] for k in keys(RSU_Dict)]
packet = (RSU_ENU, Agents ,ITSOutput)
open("packet.bin", "w") do f
    serialize(f, packet)
end

means, fail_reasons, out_of_range = ITS_quality_assess(getfield.(Agents, :smart),
                                        BaseOutput[3],
                                        ITSOutput[3],
                                        ITSOutput[5],
                                        ITSOutput[6],
                                        RSU_ENU,
                                        range,
                                        ITSOutput[7]);
println(means)
println(out_of_range)
minimum(ITSOutput[5])
mixed_out_of_range = hcat([sum(e) for e in fail_reasons],
                            [sum(e)/length(e) for e in fail_reasons])

using IJulia
notebook()
IJulia.installkernel("Julia nodeps", "--depwarn=no")

using PyCall
flm = pyimport("folium")
matplotlib_cm = pyimport("matplotlib.cm")
matplotlib_colors = pyimport("matplotlib.colors")

cmap = matplotlib_cm.get_cmap("prism")

m = flm.Map()
locs = [LLA(n, map_data.bounds) for n in RSU_ENU]
for k=1:length(RSU_ENU)

    info = "RSU number: $k"
    flm.Circle(
      location=[locs[k].lat,locs[k].lon],
      popup=info,
      tooltip=info,
      radius=300,
      color="blue",
      weight=0.5,
      fill=true,
      fill_color="blue"
   ).add_to(m)
        flm.Circle(
      location=[locs[k].lat,locs[k].lon],
      popup=info,
      tooltip=info,
      radius=1,
      color="crimson",
      weight=3,
      fill=false,
      fill_color="crimson"
   ).add_to(m)
end

routes = [a.route for a in Agents if a.smart]
#Paths
for z=1:length(routes)
    locs = [LLA(map_data.nodes[n],map_data.bounds) for n in routes[z]]
    info = "Agent $z route\n<BR>"*
        "Length: $(length(routes[z])) nodes\n<br>" *
        "From: $(routes[z][1]) $(round.((locs[1].lat, locs[1].lon),digits=4))\n<br>" *
        "To: $(routes[z][end]) $(round.((locs[end].lat, locs[end].lon),digits=4))"
    flm.PolyLine(
        [(loc.lat, loc.lon) for loc in locs ],
        popup=info,
        tooltip=info,
        weight = 2,
        color="green"
    ).add_to(m)
end

#failed transmissions
failedENU1 = unique(collect(Iterators.flatten(failedENU)))
flocs = [LLA(n, map_data.bounds) for n in failedENU1]
for i = 1:length(failedENU1)
    info = "Failed number: $i"
    flm.Circle(
      location=[flocs[i].lat,flocs[i].lon],
      popup=info,
      tooltip=info,
      radius=8,
      color="black",
      weight=3,
      fill=true,
        fill_color="black"
   ).add_to(m)
end

MAP_BOUNDS = [(map_data.bounds.min_y,map_data.bounds.min_x),(map_data.bounds.max_y,map_data.bounds.max_x)]
flm.Rectangle(MAP_BOUNDS, color="black",weight=6).add_to(m)
m.fit_bounds(MAP_BOUNDS)
m.save("index.html")
m

using IJulia
notebook()

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
