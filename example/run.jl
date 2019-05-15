using OpenStreetMapX
using RSUOptimization
using Tables
using CSV
using DataFrames
#Creating MapData object
mapfile = "reno_east3.osm"
datapath = "C:/RSUOptimization.jl/example";
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels = Set(1:5));

#Creating MapData object for Warsaw
mapfile = "WarsawFiltered.osm"
datapath = "C:/RSUOptimization.jl/example";
RoadSet = 5
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels = Set(1:RoadSet));
map_data.bounds
#Defining starting and ending area
Start = ((39.50,-119.70),(39.55,-119.74))
End = ((39.50,-119.80),(39.55,-119.76))

StartWaw = ((52.2188,21.0068),(52.2300,21.03))
EndWaw = ((52.2482,21.0068),(52.235,21.03))
#Proportion of smart agents
α = 0.9
N = 1000
density_factor = 5.0
range = 1000.0
throughput = 200
updt_period = 200
T = 0.1
k = 3
#Generating agents
Agents, init_times, init_dists = generate_agents(map_data, N, [StartWaw], [EndWaw], α)
#Running base simulation - no V2I system
@time BaseOutput, tracking = base_simulation(map_data, Agents)
#ITS model with iterative RSU optimization
@time ITSOutput, RSUs = iterative_simulation_ITS(map_data, Agents, range, throughput, updt_period, debug_level=2)

RSUs = calculate_RSU_location(map_data, Agents, range, throughput)
@time ITSOutput, trackingITS = simulation_ITS(map_data,Agents,range,RSUs,150,T,k,density_factor,2)

typeof(trackingITS)
for i in 1:1000
  Agents[i].smart = true
end
println(tracking[295])
println(trackingITS[295])

map_data.w[389,390]
mean((BaseOutput.TravelTimes - ITSOutput.TravelTimes)./BaseOutput.TravelTimes)
(sum(BaseOutput.TravelTimes)-sum(ITSOutput.TravelTimes))/sum(BaseOutput.TravelTimes)
using StatsBase
typeof(StartWaw)
"""
Smart cars percentage analysis
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
              TotalTimeReduction = Float64[],
              SmartTimeReduction = Float64[],
              NotSmartTimeReduction = Float64[],
              MinAvailability = Float64[],
              MeanRSUUtilization = Float64[],
              RSUs = Int[])

αs = 0.1:0.2:1.0
for element in αs
    for i=1:5
      println("$element : $i")
      #Generating agents
      Agents, init_times, init_dists = generate_agents(map_data, N, [StartWaw], [EndWaw], element)
      #Running base simulation - no V2I system
      BaseOutput, tracking = base_simulation(map_data, Agents, debug_level = 0)
      #ITS model with iterative RSU optimization
      #ITSOutput, RSUs = iterative_simulation_ITS(map_data, Agents, range, throughput, updt_period, T = element, debug_level = 1)
      RSUs = calculate_RSU_location(map_data, Agents, range, throughput)
      ITSOutput, trackingITS = simulation_ITS(map_data,Agents,range,RSUs,updt_period,T,k,density_factor,1)
      step_statistics = gather_statistics(getfield.(Agents,:smart),
                                          BaseOutput.TravelTimes,
                                          ITSOutput.TravelTimes,
                                          ITSOutput.ServiceAvailability,
                                          ITSOutput.RSUsUtilization,
                                          RSUs)
      println(step_statistics)
      push!(ResultFrame, [mapfile, RoadSet,
                          string(StartWaw), string(EndWaw),
                          element, N, density_factor,
                          range, throughput, updt_period, T, k,
                          step_statistics.overall_time,
                          step_statistics.smart_time,
                          step_statistics.other_time,
                          step_statistics.service_availability,
                          step_statistics.RSUs_utilization,
                          step_statistics.RSU_count])
    end
end
CSV.write("resultsWarsawAlfa.csv",ResultFrame)
typeof(ITSOutput.FailedUpdates)
length(StartWaw)
using IJulia
notebook()
IJulia.installkernel("Julia nodeps", "--depwarn=no")
