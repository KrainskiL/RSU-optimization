#Creating MapData object
mapfile = "reno_east3.osm"
datapath = "C:/RSUOptimization.jl/example";
RoadSet = 5
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels = Set(1:RoadSet));

#Defining starting and ending area
Start = [Rect((39.50,-119.70),(39.55,-119.74))]
End = [Rect((39.50,-119.80),(39.55,-119.76))]


#Creating MapData object for Warsaw
mapfile = "WarsawFiltered.osm"
datapath = "C:/RSUOptimization.jl/example";
RoadSet = 5
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels = Set(1:RoadSet));
Start = [Rect((52.2188,21.0068),(52.2300,21.03))]
End = [Rect((52.2482,21.0068),(52.235,21.03))]
#Crossing bridge
# Start = [Rect((52.2188,21.0068),(52.2482,21.02))]
# End = [Rect((52.2188,21.06),(52.2482,21.0888))]

α = 0.9
N = 1000
density_factor = 5.0
range = 500.0
throughput = 100
updt_period = 200
T = 0.1
k = 3
V2V = true
V2V_range = 200.0
V2V_throughput = 5
#Generating agents
Agents = generate_agents(map_data, N, Start, End, α)[1]
#Running base simulation - no V2I system
@time BaseOutput = base_simulation(map_data, Agents)
#ITS model with iterative RSU optimization
@time ITSOutput, RSUs = iterative_simulation_ITS(map_data,
                        Agents,
                        range,
                        throughput,
                        updt_period,
                        debug_level=2,
                        V2V=V2V,
                        V2V_range=V2V_range,
                        V2V_throughput=V2V_throughput)

RSUs = calculate_RSU_location(map_data, Agents, range, throughput, V2V_throughput)
sum(getfield.(RSUs,:count))
@time ITSOutput = simulation_ITS(map_data,
                                                Agents,
                                                range,
                                                RSUs,
                                                updt_period,
                                                T,
                                                k,
                                                density_factor,
                                                2,
                                                V2V,
                                                V2V_range,
                                                V2V_throughput)

include("C:/RSUOptimizationVis.jl/src/RSUOptimizationVis.jl")
RSUOptimizationVis.visualize_bounds(map_data,Start,End,"TEST.html")
RSUOptimizationVis.visualize_RSUs_and_failures(map_data, Start, End, Agents, [ITSOutput.FailedUpdates[1]],RSUs,range,"debug.html")

using StatsBase
mean((BaseOutput.TravelTimes - ITSOutput.TravelTimes)./BaseOutput.TravelTimes)
(sum(BaseOutput.TravelTimes)-sum(ITSOutput.TravelTimes))/sum(BaseOutput.TravelTimes)
