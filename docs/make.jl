using FlyingRobots
using Documenter

DocMeta.setdocmeta!(FlyingRobots, :DocTestSetup, :(using FlyingRobots); recursive=true)

makedocs(;
    modules=[FlyingRobots],
    authors="Sarath Suresh",
    repo="https://github.com/Sarath Suresh/FlyingRobots.jl/blob/{commit}{path}#{line}",
    sitename="FlyingRobots.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://Sarath Suresh.github.io/FlyingRobots.jl",
        edit_link="main",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/Sarath Suresh/FlyingRobots.jl",
    devbranch="main",
)
