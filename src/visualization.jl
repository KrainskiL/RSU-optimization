#######################################
## Functions creating visualizations ##
#######################################

using PyCall
function visualize_RSUs_and_failures(OSMmap::MapData,
                                    Start::Vector{Rect},
                                    End::Vector{Rect},
                                    Agents::Vector{Agent},
                                    failedENU::Array{Array{ENU,1},1},
                                    RSUs::Array{RSU,1},
                                    outfile::String)
    RSU_ENU = getfield.(RSUs,:ENU)
    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")

    m = flm.Map()
    #Add RSUs location and their range
    locs = [OpenStreetMapX.LLA(n, OSMmap.bounds) for n in RSU_ENU]
    for k = 1:length(RSU_ENU)
        info = "RSU number: $k"
        flm.Circle(
          location=[locs[k].lat,locs[k].lon],
          popup=info,
          tooltip=info,
          radius=range,
          color="blue",
          weight=0.5,
          fill=true,
          fill_color="blue"
       ).add_to(m)
            flm.Circle(
          location=[locs[k].lat,locs[k].lon],
          popup=info,
          tooltip=info,
          radius=1,
          color="crimson",
          weight=3,
          fill=false,
          fill_color="crimson"
       ).add_to(m)
    end

    #Add subset of agents' routes
    routes = [a.route for a in Agents if a.smart]
    #Subset routes if more than 500 smart agents - HTML performance
    if length(routes) > 500 routes = routes[1:500] end
    for z = 1:length(routes)
        locs = [OpenStreetMapX.LLA(OSMmap.nodes[n], OSMmap.bounds) for n in routes[z]]
        info = "Agent $z route\n<BR>"*
            "Length: $(length(routes[z])) nodes\n<br>" *
            "From: $(routes[z][1]) $(round.((locs[1].lat, locs[1].lon),digits=4))\n<br>" *
            "To: $(routes[z][end]) $(round.((locs[end].lat, locs[end].lon),digits=4))"
        flm.PolyLine(
            [(loc.lat, loc.lon) for loc in locs ],
            popup=info,
            tooltip=info,
            weight = 2,
            color="green"
        ).add_to(m)
    end

    #Add points of failed transmission
    failedENU = unique(collect(Iterators.flatten(failedENU)))
    flocs = [OpenStreetMapX.LLA(n, OSMmap.bounds) for n in failedENU]
    for i = 1:length(failedENU)
        info = "Failed ID: $i"
        flm.Circle(
          location=[flocs[i].lat, flocs[i].lon],
          popup=info,
          tooltip=info,
          radius=8,
          color="black",
          weight=3,
          fill=true,
            fill_color="black"
       ).add_to(m)
    end

    #Add bounds and
    MAP_BOUNDS = [(map_data.bounds.min_y,map_data.bounds.min_x),(map_data.bounds.max_y,map_data.bounds.max_x)]
    START = [StartWaw[1],StartWaw[2]]
    END = [EndWaw[1], EndWaw[2]]
    flm.Rectangle(MAP_BOUNDS, color="black",weight=6).add_to(m)
    for r in Start
      flm.Rectangle(r, color="blue",weight=2).add_to(m)
    end
    for r in End
      flm.Rectangle(r, color="red",weight=2).add_to(m)
    end
    m.fit_bounds(MAP_BOUNDS)
    m.save(outfile)
end
