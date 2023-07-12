
module TypeUtilitiesTest

using FlyingRobots
using Test

struct SampleStruct
    x::Float64
    y::Float64
end

struct SampleStructNotFloat64
    x::Float64
    y::Float32
end



function run_tests()

    obj = SampleStruct(1.0, 2.0)
    # convert struct to Float64 vector
    vec_ = struct_to_float64_vector(obj)

    @testset "Type Utities: Low level tests" begin
        @test checkif_struct_has_only_Float64(SampleStruct) == true
        @test get_struct_length(SampleStruct) == 2

        # check if struct elements and vector values match 
        @test [obj.x, obj.y] == vec_
    end

    # state = Quad2DState(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    # control_cmd = Quad2DControlCmd(7.0, 8.0)
    # diffeq_state = convert_Fr_types_to_diffeq_state(state, control_cmd)


    # @testset "Type Utities: High level tests" begin
    #     @test diffeq_state == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    # end

end

run_tests()

end