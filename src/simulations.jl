###################################
## Simulation models ##
###################################
"""
`base_simulation` run simulation without information exchange.

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `inAgents` : set of agents created with generate_agents function
* `density_factor` : road length reserved for one vehicle
* `debug` : debug messages switch
"""
function base_simulation(OSMmap::OpenStreetMapX.MapData,
                        inAgents::Vector{Agent},
                        density_factor::Float64 = 5.0,
                        debug::Bool = true)
    Agents = deepcopy(inAgents) #Creating working copy of agents
    max_densities, max_speeds = traffic_constants(OSMmap, density_factor) #Traffic characteristic constants
    densities, speeds = init_traffic_variables(OSMmap, Agents) #Traffic characteristic variables
    update_weights!(speeds, densities, max_densities, max_speeds) #Initial speeds update
    #Initialize simulation variables
    simtime = 0.0
    steps = 0
    active = sum(getfield.(Agents,:active))
    if debug modulo = 10^(Int(round(Int, log10(length(Agents)))) - 1) end
    #Loop until all agents are deactivated
    while active != 0
        steps += 1
        event_time, ID = next_edge(Agents, speeds, OSMmap.w) #Calculate next event time
        simtime += event_time
        vAgent = Agents[ID]
        update_agents_position!(Agents, event_time, speeds) #Update all agents positions
        #Process agent connected with event
        density_change = update_event_agent!(vAgent, simtime, densities, OSMmap.v)
        active = sum(getfield.(Agents,:active))
        if debug && length(density_change) == 1 && active % modulo == 0
            println("Active agents: $active")
        end
        update_weights!(speeds, density_change, max_densities, max_speeds) #Update speeds
    end
    times = getfield.(Agents,:travel_time)
    return steps, simtime, times
end

"""
`simulation_ITS` run simulation with V2I communication.

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `inAgents` : set of agents created with generate_agents function
* `range` : infrastructure transfer range
* `RSUs` : vector with RSUs used in simulation
* `update_period` : period of weights updates
* `T` : distribution parameter in k-shortest path rerouting
* `k` : number of fastest routes generated in rerouting function
* `density_factor` : road length reserved for one vehicle
* `debug` : debug messages switch
"""
function simulation_ITS(OSMmap::MapData,
                        inAgents::Vector{Agent},
                        range::Float64,
                        RSUs::Vector{RSU},
                        update_period::Int64,
                        T::Float64 = 1.0,
                        k::Int64 = 3,
                        density_factor::Float64 = 5.0,
                        debug::Bool = true)
    Agents = deepcopy(inAgents) #Creating working copy of agents
    max_densities, max_speeds = traffic_constants(OSMmap, density_factor) #Traffic characteristic constants
    densities, speeds = init_traffic_variables(OSMmap, Agents) #Traffic characteristic variables
    update_weights!(speeds, densities, max_densities, max_speeds) #Initial speeds update
    #Initialize simulation variables
    simtime = 0.0
    steps = 0
    active = sum(getfield.(Agents,:active))
    if debug modulo = 10^(Int(round(Int, log10(length(Agents)))) - 1) end

    #Initialize statistic vectors
    RSUs_utilization = Vector{Dict{ENU, Int64}}()
    no_updates = Vector{Vector{ENU}}()
    service_avblty = Vector{Float64}()
    #Loop until all agents are deactivated
    while active != 0
        steps += 1
        event_time, ID = next_edge(Agents, speeds, OSMmap.w) #Calculate next edge change time
        #Calculate time to next weights update
        next_update = (simtime ÷ update_period + 1) * update_period - simtime
        #Check if weight updates occur before event_time
        if next_update < event_time
            if debug update_nr = Int(simtime ÷ update_period + 1) end
            update_agents_position!(Agents, next_update, speeds) #Update position of all agents
            #Send update to agents in range if throughput limit not reached
            updates, no_update, updt_util, updt_avblty = send_weights_update(Agents, OSMmap, RSUs, range)
            if debug
                smart_active = sum([1 for a in Agents if a.active && a.smart])
                sum_updt = sum(updates)
                perc_served = round(updt_avblty*100, digits = 2)
                print("Update $update_nr | Service availability: $(sum_updt)/$(smart_active) = $perc_served%")
            end
            #Update statistic variables
            service_avblty = [service_avblty; updt_avblty]
            RSUs_utilization = [RSUs_utilization; updt_util]
            no_updates = [no_updates; [no_update]]
            #Reroute updated agents
            for a in Agents[updates]
                k_shortest_path_rerouting!(OSMmap, a, speeds, k, T)
            end
            simtime += next_update #Increase simulation time
            debug && println(" Finished")
            continue #Skip to next event
        end
        simtime += event_time
        vAgent = Agents[ID]
        #Update position of all agents
        update_agents_position!(Agents, event_time, speeds)
        #Process agent connected with event
        density_change = update_event_agent!(vAgent, simtime, densities, OSMmap.v)
        active = sum(getfield.(Agents,:active))
        if debug && length(density_change) == 1 && active % modulo == 0
            println("Active agents: $active")
        end
        update_weights!(speeds, density_change, max_densities, max_speeds) #Update speeds
    end
    times = getfield.(Agents,:travel_time)
    output_tuple = (Steps = steps,
                    Simtime = simtime,
                    ServiceAvailability = service_avblty,
                    RSUsUtilization = RSUs_utilization,
                    FailedUpdates = no_updates)
    return output_tuple
end

"""
`iterative_simulation_ITS` run simulation with iterative RSUs optimization.

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `inAgents` : set of agents created with generate_agents function
* `range` : infrastructure transfer range
* `throughput` : number of agents RSU can serve at once
* `update_period` : period of weights updates
* `threshold` : constraint for minimum service availability
* `T` : distribution parameter in k-shortest path rerouting
* `k` : number of fastest routes generated in rerouting function
* `density_factor` : road length reserved for one vehicle
* `debug` : debug messages switch
"""
function iterative_simulation_ITS(OSMmap::MapData,
                        inAgents::Vector{Agent},
                        range::Float64,
                        throughput::Int64,
                        update_period::Int64,
                        threshold::Float64 = 0.95,
                        T::Float64 = 1.0,
                        k::Int64 = 3,
                        density_factor::Float64 = 5.0,
                        debug::Bool = true)
    #Initialize working variables
    min_availability = 0.0
    iteration = 1
    #Find initial RSUs locations
    RSUs = optimize_RSU_location(OSMmap, inAgents, range, throughput)
    ITSOutput = Tuple{}()
    while min_availability < threshold
        if debug
            println("#################################")
            println("Iteration nr $iteration started")
            println("#################################")
        end
        #Repeat optimization process if threshold not met
        if iteration > 1 RSUs = reoptimize_RSU_location!(OSMmap, RSUs, ITSOutput.FailedUpdates, range) end
        #Run ITS simulation
        ITSOutput = simulation_ITS(OSMmap, inAgents, range, RSUs, update_period, T, k, density_factor, debug)
        service_avblty = round.(ITSOutput.ServiceAvailability, digits=3)
        min_availability = minimum(service_avblty)
        if debug
            println("Service availability in updates:")
            println(service_avblty)
            println("Minimum service availability: $min_availability")
            min_availability < threshold && println("Threshold not met, repeating optimization process")
        end
        iteration += 1
    end
    return ITSOutput, RSUs
end
