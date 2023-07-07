using FlyingRobots
using Documenter

import Downloads
Downloads.download(
    "https://raw.githubusercontent.com/JuliaDynamics/doctheme/master/build_docs_with_style.jl",
    joinpath(@__DIR__, "styling/build_docs_with_style.jl")
)
include("styling/build_docs_with_style.jl")

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
        "Introduction" => "index.md",
        "Overarching tutorial" => "tutorial.md",
        "Contents" => "contents.md",
        "Animations, GUIs, Visuals" => "visualization.md",
    ]
)

deploydocs(;
    repo="github.com/sarath-menon/FlyingRobots.jl",
    devbranch="main"
)
