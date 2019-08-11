using OpenStreetMapX
using RSUOptimization
using RSUOptimizationVis
using CSV
using DataFrames

#Creating MapData object
mapfile = "reno_east3.osm";
datapath = "C:/RSUOptimization.jl/example";
RoadSet = 5;
map_data = OpenStreetMapX.get_map_data(datapath, mapfile, use_cache = false; road_levels = Set(1:RoadSet));

#Defining starting and ending area
Start = [Rect((39.50,-119.70),(39.55,-119.74))]
End = [Rect((39.50,-119.80),(39.55,-119.76))]

#Input parameters
α = 0.5
N = 800
density_factor = 5.0
RSU_range = 500.0
throughput = 50
updt_period = 200
T = 0.1
k = 3
mode = "V2I"
V2V_range = 200.0
V2V_throughput = 5

"""
Parameters analysis
"""
ResultFrame = DataFrame(Map = String[],
              RoadSet = Int64[],
              Start = String[],
              End = String[],
              alfa = Float64[],
              N = Int64[],
              density_factor = Float64[],
              range = Float64[],
              throughput = Int64[],
              update_period = Int64[],
              T = Float64[],
              k = Int64[],
              mode = String[],
              V2V_range = Float64[],
              V2V_throughput = Int64[],
              TotalTimeReduction = Float64[],
              SmartTimeReduction = Float64[],
              NotSmartTimeReduction = Float64[],
              MinAvailability = Float64[],
              MeanRSUUtilization = Float64[],
              RSUs = Int[],
              OptimizationFailed = Bool[],
              Runtime = Float64[])

ParameterRange = 0.1:0.1:1.0
for element in ParameterRange
      for i in 1:5
      println("$element : $i")
      #Generating agents
      Agents = generate_agents(map_data, N, Start, End, element)[1]
      #Running base simulation - no V2I system
      BaseOutput = simulation_run("base", map_data, Agents)
      #ITS model with iterative RSU optimization
      ITSOutput, RSUs, OptimFail, runtime = iterative_simulation_ITS(mode,
                                                map_data,
                                                Agents,
                                                RSU_range,
                                                throughput,
                                                updt_period,
                                                T = T,
                                                k = k,
                                                debug_level = 1,
                                                V2V_range = V2V_range,
                                                V2V_throughput = V2V_throughput)
      step_statistics = gather_statistics(getfield.(Agents,:smart),
                                          BaseOutput.TravelTimes,
                                          ITSOutput.TravelTimes,
                                          ITSOutput.ServiceAvailability,
                                          ITSOutput.RSUsUtilization,
                                          RSUs)
      println(step_statistics)
      push!(ResultFrame, [mapfile, RoadSet,
                          string((Start[1].p1,Start[1].p2)), string((End[1].p1,End[1].p2)),
                          element, N, density_factor,
                          RSU_range, throughput, updt_period, T, k,
                          mode, V2V_range, V2V_throughput,
                          step_statistics.overall_time,
                          step_statistics.smart_time,
                          step_statistics.other_time,
                          step_statistics.service_availability,
                          step_statistics.RSUs_utilization,
                          step_statistics.RSU_count,
                          OptimFail,
                          runtime])
      end
end
CSV.write("RenoV2I.csv", ResultFrame)
