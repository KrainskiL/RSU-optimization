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

α = 0.6
N = 1000
density_factor = 5.0
RSU_range = 400.0
throughput = 50
updt_period = 200
T = 0.1
k = 3
mode = "V2V"
V2V_range = 1000.0
V2V_throughput = 9

#Generating agents
Agents = generate_agents(map_data, N, Start, End, α)[1]

#ITS model with iterative RSU optimization
ITSOutput, RSUs, OptimCrit, runtime = iterative_simulation_ITS(mode,
                                          map_data,
                                          Agents,
                                          RSU_range,
                                          throughput,
                                          updt_period,
                                          T = T,
                                          debug_level = 1,
                                          V2V_range = V2V_range,
                                          V2V_throughput = V2V_throughput)
RSUs = calculate_RSU_location("v2v", map_data, Agents, RSU_range, throughput, V2V_throughput)
sum(getfield.(RSUs,:count))
[r.count for r in RSUs if r.count>1]

@time ITSOutput = simulation_run("V2V", map_data,
                                                Agents,
                                                RSU_range,
                                                RSUs,
                                                updt_period,
                                                T,
                                                k,
                                                density_factor,
                                                V2V_range,
                                                V2V_throughput,
                                                debug_level=2)

include("C:/RSUOptimizationVis.jl/src/RSUOptimizationVis.jl")
RSUOptimizationVis.visualize_bounds(map_data,Start,End,"TEST.html")
RSUOptimizationVis.visualize_RSUs_and_failures(map_data, Start, End, Agents, [ITSOutput.FailedUpdates[1]],RSUs,range,"debug.html")

using StatsBase
mean((BaseOutput.TravelTimes - ITSOutput.TravelTimes)./BaseOutput.TravelTimes)
(sum(BaseOutput.TravelTimes)-sum(ITSOutput.TravelTimes))/sum(BaseOutput.TravelTimes)


#Creating MapData object
mapfile = "SanFranciscoFiltered.osm"
datapath = "C:/RSUOptimization.jl/example";
RoadSet = 5
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels = Set(1:RoadSet));
#Defining starting and ending area
Start = [Rect((37.795,-122.42),(37.811,-122.3942))]
End = [Rect((37.7805,-122.485),(37.79,-122.45))]


#Creating MapData object for Warsaw
mapfile = "WarsawFiltered.osm"
datapath = "C:/RSUOptimization.jl/example";
RoadSet = 5
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels = Set(1:RoadSet));
Start = [Rect((52.2188,21.0068),(52.2300,21.03))]
End = [Rect((52.2482,21.0068),(52.235,21.03))]
