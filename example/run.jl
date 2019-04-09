using OpenStreetMapX
using StatsBase
using SparseArrays

mapfile = "reno_east3.osm"
datapath = "C:/RSUOptimization.jl/example";
map_data = get_map_data(datapath, mapfile,use_cache=false);

N = 3000
AgentsArr = Vector{Agent}()

#Generating N agents
for i in 1:N
    dist = Inf
    start_node = 0
    end_node = 0
    counter = 0
    init_route = Array{Int64,1}()
    time = 0
    while dist == Inf
        start_node = pick_random_node(map_data, ((41.0,-119.70),(39.0,-119.74)))
        end_node = pick_random_node(map_data, ((41.0,-119.70),(39.0,-119.74)))
        while start_node == end_node
            start_node = pick_random_node(map_data, ((41.0,-119.70),(39.0,-119.74)))
            end_node = pick_random_node(map_data, ((41.0,-119.70),(39.0,-119.74)))
        end
        init_route, dist, time = fastest_route(map_data, start_node, end_node)
        counter +=1
        if counter == 100
            error("Route from starting to ending point can't be calculated.")
        end
    end
    firstEdge = Dict("start_v"=> map_data.v[init_route[1]],
                    "end_v"=> map_data.v[init_route[2]],
                    "start_n"=> init_route[1],
                    "end_n"=> init_route[2])
    NewAgent = Agent(i,  start_node, end_node, init_route, 0.0, firstEdge, 0.0)
    push!(AgentsArr, NewAgent)
end

@time FinalAgents, iterations, time = simulation(AgentsArr, map_data)
init_route, dist, time = fastest_route(map_data, -2334529966553521260, 3066052536)

(51.437-47.477)/47.477


sweep = collect(Iterators.product( 1:2*2,
              range(0.0, stop = 1.0, step = 0.25),
              range(0.0, stop = 1.0, step = 0.25)))
