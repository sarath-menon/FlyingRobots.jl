# [Interactive GUIs, animations, visualizations](@id visualization)

Using the functionality of package extensions in Julia v1.9+, DynamicalSystems.jl provides various visualization tools as soon as the [Makie](https://makie.juliaplots.org/stable/) package comes into scope (i.e., when `using Makie` or any of its backends like `GLMakie`).

The main functionality is [`interactive_trajectory`](@ref) that allows building custom GUI apps for visualizing the time evolution of dynamical systems. The remaining GUI applications in this page are dedicated to more specialized scenarios.

## Interactive- or animated trajectory evolution

```@raw html
<iframe width="560" height="315" src="https://www.youtube.com/embed/VT1XY1-fNlY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
```
