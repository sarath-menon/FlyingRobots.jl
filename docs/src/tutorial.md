# [Overarching tutorial for DynamicalSystems.jl](@id tutorial)

This page serves as a short, but to-the-point, introduction to the **DynamicalSystems.jl** library. It outlines the core components, and how they establish an interface that is used by the rest of the library. It also provides a couple of usage examples to connect the various packages of the library together.

Going through this tutorial should take you about 20 minutes.

## Installation

To install **DynamicalSystems.jl**, simply do:
```julia
using Pkg; Pkg.add("DynamicalSystems")
```

As discussed in the [contents](@ref contents) page, this installs several packages for the Julia language, that are all exported under a common name. To use them, simply do:
```julia
using DynamicalSystems
```
in your Julia session.

## Core components