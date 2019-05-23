###################################
## Simulation models ##
###################################
"""
`simulation_run` run traffic simulation with specified model.

**Input parameters**
* `mode` : simulation model switch
    * `base` : simulation with no vehicular communication
    * `V2I` : simulation with V2I communication
    * `V2V` : simulation with V2I and V2V communication (hybrid)
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
* `V2V` : V2V hybrid model switch
* `V2V_range` : range of V2V communication
* `V2V_throughput` : throughput of V2V master to slaves communication
"""
function simulation_run(mode::String,
                        OSMmap::MapData,
                        inAgents::Vector{Agent},
                        range::Float64 = 1000.0,
                        RSUs::Vector{RSU} = Vector{RSU}(),
                        update_period::Int64 = 100,
                        T::Float64 = 1.0,
                        k::Int64 = 3,
                        density_factor::Float64 = 5.0,
                        V2V_range::Float64 = 0.0,
                        V2V_throughput::Int64 = 1;
                        debug_level::Int64 = 0)
    mode = lowercase(mode)
    if !in(mode, ["base","v2v","v2i"]) error("Wrong mode specified.") end
    Agents = deepcopy(inAgents) #Creating working copy of agents
    max_densities, max_speeds = traffic_constants(OSMmap, density_factor) #Traffic characteristic constants
    densities, speeds = init_traffic_variables(OSMmap, Agents) #Traffic characteristic variables
    update_weights!(speeds, densities, max_densities, max_speeds) #Initial speeds update
    #Initialize simulation variables
    simtime = 0.0
    steps = 0
    active = sum(getfield.(Agents,:active))
    if debug_level > 2 modulo = 10^(Int(round(Int, log10(length(Agents)))) - 1) end
    if mode != "base"
        #Initialize statistic vectors
        RSUs_utilization = Vector{Dict{Int64, Int64}}()
        no_updates = Vector{Vector{ENU}}()
        service_avblty = Vector{Float64}()
    end
    #Calculate initial time to nearest junction for all agents
    times_to_event = next_edge(Agents, speeds, OSMmap.w)
    #Loop until all agents are deactivated
    while active != 0
        steps += 1
        #Calculate next event time
        event_time, ID = findmin(times_to_event)
        if mode != "base"
            #Calculate time to next weights update
            next_update = (simtime ÷ update_period + 1) * update_period - simtime
            #Check if weight updates occur before event_time
            if next_update < event_time
                if debug_level > 1 update_nr = Int(simtime ÷ update_period + 1) end
                times_to_event .-= next_update
                #Send update to agents in range if throughput limit not reached
                updates, no_update, updt_util, updt_avblty = send_weights_update(Agents, OSMmap, speeds,
                times_to_event, RSUs, range, mode, V2V_range, V2V_throughput)
                if debug_level > 1
                    smart_active = sum([1 for a in Agents if a.active && a.smart])
                    sum_updt = sum(updates)
                    perc_served = round(updt_avblty*100, digits = 2)
                    print("Update $update_nr | Service availability: $(sum_updt)/$(smart_active) = $perc_served% ")
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
                debug_level > 1 && println("Finished")
                continue #Skip to next event
            end
        end
        simtime += event_time #Increase total simulation time
        vAgent = Agents[ID] #Pick agent for which event is occuring
        times_to_event .-= event_time #Move agents forward to event time
        #Process agent for which event is occuring
        changed_edges = update_event_agent!(vAgent, simtime, densities, OSMmap.v)
        if !vAgent.active times_to_event[ID] = Inf end
        #Update speeds
        speed_factors = update_weights!(speeds, changed_edges, densities, max_densities, max_speeds)
        #Correct events time due to speed changes
        event_time_correction!(Agents, changed_edges, speed_factors, times_to_event)
        if vAgent.active times_to_event[ID] = next_edge(vAgent, speeds, OSMmap.w) end
        active = sum(getfield.(Agents,:active))
        if debug_level > 2 && length(changed_edges) == 1 && active % modulo == 0
            println("Active agents: $active")
        end
    end
    times = getfield.(Agents,:travel_time)
    if mode == "base"
        output_tuple =(
            Steps = steps,
            Simtime = simtime,
            TravelTimes = times)
    else
        RSUs_utilization = Vector{Dict{Int64, Float64}}(RSUs_utilization)
        output_tuple = (Steps = steps,
                        Simtime = simtime,
                        TravelTimes = times,
                        ServiceAvailability = service_avblty,
                        RSUsUtilization = RSUs_utilization,
                        FailedUpdates = no_updates)
    end
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
* `V2V` : V2V hybrid model switch
* `V2V_range` : range of V2V communication
* `V2V_throughput` : throughput of V2V master to slaves communication
"""
function iterative_simulation_ITS(mode::String,
                        OSMmap::MapData,
                        inAgents::Vector{Agent},
                        range::Float64,
                        throughput::Int64,
                        update_period::Int64;
                        threshold::Float64 = 0.95,
                        T::Float64 = 0.1,
                        k::Int64 = 3,
                        density_factor::Float64 = 5.0,
                        debug_level::Int64 = 1,
                        V2V_range::Float64 = 0.0,
                        V2V_throughput::Int64 = 1)
    #Initialize working variables
    iteration = 1
    IterationResults = DataFrame(MinAvailability = Float64[], RSUs = Int[], Avail_per_RSU = Float64[])
    #Find initial RSUs locations
    RSUs = calculate_RSU_location(OSMmap, inAgents, range, throughput, V2V_throughput)
    ITSOutput = Tuple{}()
    availability_failed = utilization_failed = true
    while availability_failed || utilization_failed
        if debug_level > 0
            println("#################################")
            println("Iteration nr $iteration started")
            println("#################################")
        end
        #Run ITS simulation
        ITSOutput = simulation_run(mode, OSMmap, inAgents, range, RSUs,
            update_period, T, k, density_factor, V2V_range, V2V_throughput, debug_level = debug_level)
        service_avblty = round.(ITSOutput.ServiceAvailability, digits=3)
        min_availability = minimum(service_avblty)
        RSU_Count = sum(getfield.(RSUs, :count))
        push!(IterationResults, [min_availability, RSU_Count, min_availability*100/RSU_Count])
        #Check optimization criteria
        utilization_failed = adjust_RSU_utilization!(RSUs, ITSOutput.RSUsUtilization, throughput)
        availability_failed = min_availability < threshold
        availability_failed && adjust_RSU_availability!(OSMmap, RSUs, ITSOutput.FailedUpdates, range, throughput, V2V_throughput)
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
