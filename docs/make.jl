using FlyingRobots
using Documenter

DocMeta.setdocmeta!(FlyingRobots, :DocTestSetup, :(using FlyingRobots); recursive=true)

makedocs(;
    modules=[FlyingRobots],
    authors="sarath-menon",
    repo="https://github.com/sarath-menon/FlyingRobots.jl/blob/{commit}{path}#{line}",
    sitename="FlyingRobots.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://sarath-menon.github.io/FlyingRobots.jl",
        edit_link="main",
        assets=String[]
    ),
    pages=[
        "Home" => "index.md",
    ]
)

deploydocs(;
    repo="github.com/sarath-menon/FlyingRobots.jl",
    devbranch="main"
)
