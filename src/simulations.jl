###################################
## Simulation models ##
###################################
"""
`base_simulation` run simulation without information exchange.

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `inAgents` : set of agents created with generate_agents function
* `density_factor` : road length reserved for one vehicle
"""

function base_simulation(OSMmap::OpenStreetMapX.MapData, inAgents::Vector{Agent}, density_factor::Float64 = 5.0)
    #Creating working copy of agents
    Agents = deepcopy(inAgents)
    #Traffic characteristic constants
    max_densities, max_speeds = traffic_constants(OSMmap, density_factor)
    #Traffic characteristic variables
    densities, speeds, avg_smart_dens = init_traffic_variables(OSMmap, Agents, true)
    #Initial speeds update
    update_weights!(speeds, densities, max_densities, max_speeds)
    #Initialize simulation variables
    simtime = 0.0
    steps = 0
    counter = 2
    #Loop until all agents are deactivated
    while sum(getfield.(Agents,:active)) != 0
        steps += 1
        #Calculate next event time
        event_time, ID = next_edge(Agents, speeds, OSMmap.w)
        simtime += event_time
        vAgent = Agents[ID]
        #Update all agents positions
        update_agents_position!(Agents, event_time, speeds)
        #Process agent connected with event
        density_change = update_event_agent!(vAgent, simtime, densities, OSMmap.v)
        if length(density_change) == 1 && sum(getfield.(Agents,:active)) % 10 ==0 println("Active agents $(sum(getfield.(Agents,:active)))") end
        #Update speeds
        update_weights!(speeds, density_change, max_densities, max_speeds)
        #Update average density for smart cars
        update_smart_densities!(Agents, avg_smart_dens, 50.0, simtime, counter)
    end
    times = getfield.(Agents,:travel_time)
    return steps, simtime, times, avg_smart_dens
end

"""
`simulation_ITS` run simulation with V2I communication.

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `inAgents` : set of agents created with generate_agents function
* `density_factor` : road length reserved for one vehicle
* `stats` : average density of smart cars used for RSUs optimization
* `range` : infrastructure transfer range
* `throughput` : infrastructure transfer limit
* `update_period` : period of weights updates
* `T` : distribution parameter in k-shortest path rerouting
* `k` : number of fastest routes generated in rerouting function
"""
function simulation_ITS(OSMmap::MapData,
                        inAgents::Vector{Agent},
                        density_factor::Float64,
                        stats::Dict{Array{Int64,1},Float64},
                        range::Float64,
                        RSU_ENU::Dict{ENU,Int64},
                        update_period::Int64,
                        T::Float64,
                        k::Int64,
                        debug::Bool = true)
    #Creating working copy of agents
    Agents = deepcopy(inAgents)
    #Traffic characteristic constants
    max_densities, max_speeds = traffic_constants(OSMmap, density_factor)
    #Initialize statistic vectors
    RSUs_utilization = Vector{Dict{ENU, Int64}}()
    no_updates = Vector{Vector{ENU}}()
    service_avblty = Vector{Float64}()
    #Traffic characteristic variables
    densities, speeds = init_traffic_variables(OSMmap, Agents, false)
    #Initial speeds update
    update_weights!(speeds, densities, max_densities, max_speeds)
    #Initialize simulation variables
    simtime = 0.0
    steps = 0
    modulo = 10^Int(floor(log10(length(Agents))))
    #Loop until all agents are deactivated
    while sum(getfield.(Agents,:active)) != 0
        steps += 1
        #Calculate next edge change time
        event_time, ID = next_edge(Agents, speeds, OSMmap.w)
        #Calculate next weights update time
        next_update = (simtime ÷ update_period + 1)*update_period - simtime
        #Check if weight updates occur before next_event time
        if next_update < event_time
            update_nr = Int(simtime ÷ update_period + 1)
            #Update position of all agents
            update_agents_position!(Agents, next_update, speeds)
            #Send update to agents in range if throughput limit not reached
            updates, no_update, updt_utilization, updt_avblty = send_weights_update(Agents, OSMmap, RSU_ENU, range)
            smart_active = sum(getfield.(Agents, :active).*getfield.(Agents, :smart))
            perc_served = round(sum(updates)/smart_active*100, digits = 2)
            debug && println("Update $update_nr: $(sum(updates))/$(smart_active) = $perc_served%")
            #Update statistic variables
            service_avblty = [service_avblty; updt_avblty]
            RSUs_utilization = [RSUs_utilization; updt_utilization]
            no_updates = [no_updates; [no_update]]
            #Reroute updated agents
            for i in 1:length(Agents)
                updates[i] && k_shortest_path_rerouting!(OSMmap, Agents[i], speeds, k, T)
            end
            #Increase simulation time
            simtime += next_update
            #Skip to next event
            continue
        end
        simtime += event_time
        vAgent = Agents[ID]
        #Update position of all agents
        update_agents_position!(Agents, event_time, speeds)
        #Process agent connected with event
        density_change = update_event_agent!(vAgent, simtime, densities, OSMmap.v)
        if length(density_change) == 1 && sum(getfield.(Agents,:active)) % modulo ==0 println("Active agents $(sum(getfield.(Agents,:active)))") end
        #Update speeds
        update_weights!(speeds, density_change, max_densities, max_speeds)
        #Update average density for smart cars
        #update_smart_densities!(Agents, avg_smart_dens, steps)
    end
    times = getfield.(Agents,:travel_time)
    return steps, simtime, times, service_avblty, RSUs_utilization, no_updates
end


function iterative_simulation_ITS(OSMmap::MapData,
                        inAgents::Vector{Agent},
                        density_factor::Float64,
                        stats::Dict{Array{Int64,1},Float64},
                        range::Float64,
                        throughput::Int64,
                        update_period::Int64,
                        T::Float64,
                        k::Int64,
                        threshold::Float64,
                        debug::Bool = true)

    min_availability = 0.0
    iteration = 1
    #Find initial RSUs locations
    RSU_Dict = optimize_RSU_location(OSMmap, range, throughput, "cover_all_nodes", inAgents)
    RSU_ENU = Dict([OSMmap.nodes[k] => RSU_Dict[k]*throughput for k in keys(RSU_Dict)])
    failed_updates = Vector{Vector{ENU}}()
    ITSOutput = Tuple{}()
    while min_availability < threshold
        debug && println("Iteration nr $iteration started")
        if iteration > 1
            #Repeat optimization process
            RSU_Dict = reoptimize_RSU_location!(OSMmap, RSU_Dict, failed_updates, range)
            RSU_ENU = Dict([OSMmap.nodes[k] => RSU_Dict[k]*throughput for k in keys(RSU_Dict)])
        end
        #Run ITS simulation
        ITSOutput = simulation_ITS(OSMmap, inAgents, density_factor, stats, range, RSU_ENU, update_period, T, k, debug)
        service_avblty = ITSOutput[4]
        failed_updates = ITSOutput[6]
        min_availability = round(minimum(service_avblty),digits=3)
        if debug
            println("Service availability:")
            println(round.(service_avblty,digits=3))
            println("Minimum service availability: $min_availability")
            min_availability < threshold && println("Threshold not met, reiterating")
            println("#################################")
        end
        iteration += 1
    end
    return ITSOutput, RSU_Dict
end
