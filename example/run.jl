using OpenStreetMapX

mapfile = "reno_east3.osm"
datapath = "C:/RSUOptimization.jl/example";
map_data = get_map_data(datapath, mapfile,use_cache=false);

N = 100
AgentsArr = Vector{Agent}()

#Generating N agents
for i in 1:N
    global dist = Inf
    counter = 0
    while dist == Inf
        global start_node = pick_random_node(map_data, ((41.0,-119.70),(39.0,-119.74)))
        global end_node = pick_random_node(map_data, ((41.0,-119.70),(39.0,-119.74)))
        global init_route, dist, time = fastest_route(map_data, start_node, end_node)
        counter +=1
        if counter == 100
            error("Route from starting to ending point can't be calculated.")
        end
    end
    NewAgent = Agent(i,  start_node, end_node, init_route, 0.0, ((init_route[1],init_route[2]),0.0))
    push!(AgentsArr, NewAgent)
end
