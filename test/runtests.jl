using MacroEnergySolvers
using Test
using Aqua
using JuMP
using HiGHS

const MES = MacroEnergySolvers

@testset "MacroEnergySolvers.jl" begin
    @testset "MacroEnergySolvers.jl" begin
    end
    include("infeasible_solve_detection.jl")
    @testset "Code quality (Aqua.jl)" begin
        Aqua.test_all(MacroEnergySolvers)
    end
end
