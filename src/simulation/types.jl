

abstract type SimAccMode end

struct RealtimeSim <: SimAccMode end
struct AcceleratedSim <: SimAccMode end


abstract type SimState end

struct SimIdle <: SimState end
struct SimRunning <: SimState end
struct SimPaused <: SimState end

abstract type Selva end