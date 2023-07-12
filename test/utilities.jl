
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


obj = SampleStruct(1.0, 2.0)

# convrt struct to Float64 vector
vec_ = struct_to_float64_vector(obj)



function run_tests()

    @testset "Type Utlities tests" begin
        @test checkif_struct_has_only_Float64(SampleStruct) == true
        @test get_struct_length(SampleStruct) == 2

        # check if struct elements and vector values match 
        @test [obj.x, obj.y] == vec_

    end

end

run_tests()

end