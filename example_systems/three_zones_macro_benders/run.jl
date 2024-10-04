using Pkg
Pkg.activate(dirname(dirname(@__DIR__)))

case_path = @__DIR__

println("###### ###### ######")
println("Running case at $(case_path)")

using MacroEnergySystemsDecomposition
using Macro

using LinearAlgebra

# solve_model_monolithic(case_path,:MACRO)
result = solve_model_with_benders(case_path,:MACRO)

println()