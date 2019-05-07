###################################
## Simulation models ##
###################################
"""
`base_simulation` run simulation without information exchange.

**Input parameters**
* `OSMmap` : MapData type object with road network data
* `inAgents` : set of agents created with generate_agents function
* `density_factor` : road length reserved for one vehicle
* `debug_level_level` : debug_level messages switch
"""
function base_simulation(OSMmap::OpenStreetMapX.MapData,
                        inAgents::Vector{Agent};
                        density_factor::Float64 = 5.0,
                        debug_level::Int64 = 3)
    Agents = deepcopy(inAgents) #Creating working copy of agents
    max_densities, max_speeds = traffic_constants(OSMmap, density_factor) #Traffic characteristic constants
    densities, speeds = init_traffic_variables(OSMmap, Agents) #Traffic characteristic variables
    update_weights!(speeds, densities, max_densities, max_speeds) #Initial speeds update
    #route_tracking = Vector{Vector{Tuple}}(((0,0),0.0,0.0), 1000)
    #route_tracking[3][end]-5
    #Initialize simulation variables
    simtime = 0.0
    steps = 0
    active = sum(getfield.(Agents,:active))
    if debug_level > 2 modulo = 10^(Int(round(Int, log10(length(Agents)))) - 1) end
    #Loop until all agents are deactivated
    while active != 0
        steps += 1
        event_time, ID = next_edge(Agents, speeds, OSMmap.w) #Calculate next event time
        simtime += event_time
        vAgent = Agents[ID]
        #push!(route_tracking[ID], (vAgent.edge,simtime,))
        update_agents_position!(Agents, event_time, speeds) #Update all agents positions
        #Process agent connected with event
        density_change = update_event_agent!(vAgent, simtime, densities, OSMmap.v)
        active = sum(getfield.(Agents,:active))
        if debug_level > 2 && length(density_change) == 1 && active % modulo == 0
            println("Active agents: $active")
        end
        update_weights!(speeds, density_change, max_densities, max_speeds) #Update speeds
    end
    times = getfield.(Agents,:travel_time)
    output_tuple =(
        Steps = steps,
        Simtime = simtime,
        TravelTimes = times
                    )
    return output_tuple
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
* `debug_level` : debug messages switch
    * 0 and 1 - none
    * 2 - updates info
    * 3 - active agents info
"""
function simulation_ITS(OSMmap::MapData,
                        inAgents::Vector{Agent},
                        range::Float64,
                        RSUs::Vector{RSU},
                        update_period::Int64,
                        T::Float64 = 1.0,
                        k::Int64 = 3,
                        density_factor::Float64 = 5.0,
                        debug_level::Int64 = 3)
    Agents = deepcopy(inAgents) #Creating working copy of agents
    max_densities, max_speeds = traffic_constants(OSMmap, density_factor) #Traffic characteristic constants
    densities, speeds = init_traffic_variables(OSMmap, Agents) #Traffic characteristic variables
    update_weights!(speeds, densities, max_densities, max_speeds) #Initial speeds update
    #Initialize simulation variables
    simtime = 0.0
    steps = 0
    active = sum(getfield.(Agents,:active))
    if debug_level > 2 modulo = 10^(Int(round(Int, log10(length(Agents)))) - 1) end

    #Initialize statistic vectors
    RSUs_utilization = Vector{Dict{Int64, Int64}}()
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
            if debug_level > 1 update_nr = Int(simtime ÷ update_period + 1) end
            update_agents_position!(Agents, next_update, speeds) #Update position of all agents
            #Send update to agents in range if throughput limit not reached
            updates, no_update, updt_util, updt_avblty = send_weights_update(Agents, OSMmap, RSUs, range)
            if debug_level > 1
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
            debug_level > 1 && println(" Finished")
            continue #Skip to next event
        end
        simtime += event_time
        vAgent = Agents[ID]
        #Update position of all agents
        update_agents_position!(Agents, event_time, speeds)
        #Process agent connected with event
        density_change = update_event_agent!(vAgent, simtime, densities, OSMmap.v)
        active = sum(getfield.(Agents,:active))
        if debug_level > 2 && length(density_change) == 1 && active % modulo == 0
            println("Active agents: $active")
        end
        update_weights!(speeds, density_change, max_densities, max_speeds) #Update speeds
    end
    times = getfield.(Agents,:travel_time)
    RSUs_utilization = Vector{Dict{Int64, Float64}}(RSUs_utilization)
    output_tuple = (Steps = steps,
                    Simtime = simtime,
                    TravelTimes = times,
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
* `debug_level` : debug messages switch
    * 0 - none
    * 1 - iteration summaries
    * 2 - updates info
    * 3 - active agents info
"""
function iterative_simulation_ITS(OSMmap::MapData,
                        inAgents::Vector{Agent},
                        range::Float64,
                        throughput::Int64,
                        update_period::Int64;
                        threshold::Float64 = 0.95,
                        T::Float64 = 1.0,
                        k::Int64 = 3,
                        density_factor::Float64 = 5.0,
                        debug_level::Int64 = 1)
    #Initialize working variables
    iteration = 1
    IterationResults = DataFrame(MinAvailability = Float64[], RSUs = Int[], Avail_per_RSU = Float64[])
    #Find initial RSUs locations
    RSUs = calculate_RSU_location(OSMmap, inAgents, range, throughput)
    ITSOutput = Tuple{}()
    availability_failed = utilization_failed = true
    while availability_failed || utilization_failed
        if debug_level > 0
            println("#################################")
            println("Iteration nr $iteration started")
            println("#################################")
        end
        #Run ITS simulation
        ITSOutput = simulation_ITS(OSMmap, inAgents, range, RSUs, update_period, T, k, density_factor, debug_level)
        service_avblty = round.(ITSOutput.ServiceAvailability, digits=3)
        min_availability = minimum(service_avblty)
        RSU_Count = sum(getfield.(RSUs, :count))
        push!(IterationResults, [min_availability, RSU_Count, min_availability*100/RSU_Count])
        #Check optimization criteria
        utilization_failed = adjust_RSU_utilization!(RSUs, ITSOutput.RSUsUtilization, throughput)
        availability_failed = min_availability < threshold
        availability_failed && adjust_RSU_availability!(OSMmap, RSUs, ITSOutput.FailedUpdates, range, throughput)
        if debug_level > 0
            println("Service availability in updates:")
            println(round.(ITSOutput.ServiceAvailability, digits=3))
            println("Minimum service availability: $min_availability")
            availability_failed && println("Availability too low, RSUs recalculated")
            utilization_failed && println("Utilization too low, number of RSUs reduced")
        end
        iteration += 1
    end
    if debug_level > 0 println(IterationResults) end
    return ITSOutput, RSUs
end
