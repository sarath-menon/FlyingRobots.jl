# collision checker alg result
abstract type CollisionChecker end
abstract type Feasible <: CollisionChecker end
abstract type Infeasible <: CollisionChecker end
abstract type Indeterminable <: CollisionChecker end

# obstacle types
abstract type Obstacle end

# aircraft
abstract type Aircraft end
abstract type Quadcopter <: Aircraft end
abstract type FixedWingPlane <: Aircraft end

# motion primitive types
abstract type MotionPrimitive end

struct V3 <: FieldVector{3,Float64}
    x::Float64
    y::Float64
    z::Float64

    V3() = new(0, 0, 0)
    V3(x, y, z) = new(x, y, z)
end

struct FullyDefined1 <: MotionPrimitive
    p_0::V3
    v_0::V3
    a_0::V3

    p_f::V3
    v_f::V3
    a_f::V3
end

FullyDefined = FullyDefined1

struct PositionFree1 <: MotionPrimitive
    p_0::V3
    v_0::V3
    a_0::V3

    v_f::V3
    a_f::V3
end

PositionFree = PositionFree1


struct SingleAxisTrajectory5
    p_0::Float64
    v_0::Float64
    a_0::Float64

    p_f::Float64
    v_f::Float64
    a_f::Float64
    # motion_primitive::MotionPrimitive

    T::Float64

    α::Float64
    β::Float64
    γ::Float64

    # constructor for trajectory for fully defined end state
    function SingleAxisTrajectory5(mp::FullyDefined, T; id)
        p_0 = mp.p_0[id]
        v_0 = mp.v_0[id]
        a_0 = mp.a_0[id]
        p_f = mp.p_f[id]
        v_f = mp.v_f[id]
        a_f = mp.a_f[id]

        Δp = p_f - p_0 - v_0 * T - 0.5 * a_0 * T^2
        Δv = v_f - v_0 - v_0 * T - a_0 * T
        Δa = a_f - a_0

        M = [720 -360*T 60*T^2;
            -360*T 168*T^2 -24*T^3;
            60*T^2 -24*T^3 3*T^4]

        (α, β, γ) = (1 / T^5) * M * [Δp; Δv; Δa]

        return new(p_0, v_0, a_0, p_f, v_f, a_f, T, α, β, γ)
    end

    # constructor for single axis trajectory for end state with free position
    function SingleAxisTrajectory5(mp::PositionFree, T; id)
        p_0 = mp.p_0[id]
        v_0 = mp.v_0[id]
        a_0 = mp.a_0[id]
        v_f = mp.v_f[id]
        a_f = mp.a_f[id]

        Δv = v_f - v_0 - v_0 * T - a_0 * T
        Δa = a_f - a_0

        M = [0 0;
            -12 6*T;
            6*T -2*T^2]

        (α, β, γ) = (1 / T^3) * M * [Δv; Δa]

        return new(p_0, v_0, a_0, 0, v_f, a_f, T, α, β, γ)
    end
end

SingleAxisTrajectory = SingleAxisTrajectory5

struct XYZTrajectory8
    x::SingleAxisTrajectory
    y::SingleAxisTrajectory
    z::SingleAxisTrajectory

    motion_primitive::MotionPrimitive

    T::Float64

    function XYZTrajectory8(x, y, z, motion_primitive)
        if (x.T == y.T == z.T) == false
            print("Trajectory duration for x,y,z, axes is not same !")
        end

        T = x.T

        return new(x, y, z, motion_primitive, T)
    end
end

XYZTrajectory = XYZTrajectory8

struct SphericalObstacle2 <: Obstacle
    pos::V3
    vel::V3
    acc::V3

    radius::Float64
end

SphericalObstacle = SphericalObstacle2

struct PrismObstacle3 <: Obstacle
    pos::V3
    orientation::QuatRotation
    dims::V3
end

PrismObstacle = PrismObstacle3

