Reference
=========

```@meta
CurrentModule = RSUOptimization
DocTestSetup = quote
    using RSUOptimization
end
```

Agent generation
----------------------
```@docs
Agent
pick_random_node
generate_agents
```

RSU location optimization
----------------------
```@docs
get_agent_coordinates
optimize_RSU_location
```

Rerouting
----------------------
```@docs
k_shortest_path_rerouting!
```

Vehicular communication
----------------------
```@docs
send_weights_update
```

Traffic model
----------------------
```@docs
get_max_densities
update_weights!
traffic_constants
init_traffic_variables
next_edge
update_event_agent!
update_smart_densities!
update_agents_position!
```
