abstract type SimAccMode end

struct RealtimeSim <: SimAccMode end
struct AcceleratedSim <: SimAccMode end