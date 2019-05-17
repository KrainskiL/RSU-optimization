using OpenStreetMapX
using RSUOptimization
using CSV
using DataFrames
using Distributed

#Creating MapData object
mapfile = "reno_east3.osm"
datapath = "C:/RSUOptimization.jl/example";
RoadSet = 5
map_data = OpenStreetMapX.get_map_data(datapath, mapfile,use_cache=false; road_levels = Set(1:RoadSet));

#Defining starting and ending area
Start = [Rect((39.50,-119.70),(39.55,-119.74))]
End = [Rect((39.50,-119.80),(39.55,-119.76))]

#Input parameters
α = 0.9
N = 1000
density_factor = 5.0
range = 500.0
throughput = 100
updt_period = 200
T = 0.1
k = 4

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
              TotalTimeReduction = Float64[],
              SmartTimeReduction = Float64[],
              NotSmartTimeReduction = Float64[],
              MinAvailability = Float64[],
              MeanRSUUtilization = Float64[],
              RSUs = Int[])

αs = 0.7:0.1:1.0
for element in αs
      for i in 1:5
      println("$element : $i")
      #Generating agents
      Agents = generate_agents(map_data, N, Start, End, element)[1]
      #Running base simulation - no V2I system
      BaseOutput = base_simulation(map_data, Agents, debug_level = 0)
      #ITS model with iterative RSU optimization
      ITSOutput, RSUs = iterative_simulation_ITS(map_data, Agents, range, throughput, updt_period, T = T, debug_level = 0)
      # RSUs = calculate_RSU_location(map_data, Agents, range, throughput)
      # ITSOutput = simulation_ITS(map_data,Agents,range,RSUs,updt_period,T,k,density_factor,1)
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
                          range, throughput, updt_period, T, k,
                          step_statistics.overall_time,
                          step_statistics.smart_time,
                          step_statistics.other_time,
                          step_statistics.service_availability,
                          step_statistics.RSUs_utilization,
                          step_statistics.RSU_count])
      end
end
CSV.write("reno_east3_Full_alfa_part2.csv", ResultFrame)

unique(collect(Iterators.flatten(keys.(Vector{Vector{RSU}}()))))
