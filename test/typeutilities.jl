
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


    @testset "Type Utities: Low level tests" begin
        # Test 1
        @test checkif_struct_has_only_Float64(SampleStruct) == true

        # Test 2
        @test get_struct_length(SampleStruct) == 2

        # Test 3: check if struct elements and vector values match 
        # convert struct to Float64 vector
        vec_ = struct_to_float64_vector(obj)
        @test [obj.x, obj.y] == vec_

        # Test4 : test function check whether two structs are equal 
        state_1 = fr_create(Quad2DState; y=0.0, z=0.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)
        state_2 = fr_create(Quad2DState; y=0.0, z=0.0, θ=0.0, ẏ=0.0, ż=0.0, θ̇=0.0)

        @test state_1 == state_2

    end



    # state = Quad2DState(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    # control_cmd = Quad2DActuatorCmd(7.0, 8.0)
    # diffeq_state = convert_Fr_types_to_diffeq_state(state, control_cmd)


    # @testset "Type Utities: High level tests" begin
    #     @test diffeq_state == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    # end

end

run_tests()

end