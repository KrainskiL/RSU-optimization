using Documenter

try
    using RSUOptimization
catch
    if !("../src/" in LOAD_PATH)
       push!(LOAD_PATH,"../src/")
       @info "Added \"../src/\"to the path: $LOAD_PATH "
       using RSUOptimization
    end
end

makedocs(
    sitename = "RSUOptimization",
    format = format = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true"
    ),
    modules = [RSUOptimization],
    pages = ["index.md", "reference.md"],
    doctest = true
)



deploydocs(
    repo ="github.com/KrainskiL/RSUOptimization.jl.git",
    target="build"
)
