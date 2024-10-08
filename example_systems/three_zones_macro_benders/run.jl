using Pkg
Pkg.activate(dirname(dirname(@__DIR__)))

using Debugger
case_path = @__DIR__

println("###### ###### ######")
println("Running case at $(case_path)")

using MacroEnergySystemsDecomposition
using Macro

using LinearAlgebra

# solve_model_monolithic(case_path,:MACRO)

if haskey(ENV, "DEBUGGER")
	result = Debugger.@run solve_model_with_benders(case_path,:MACRO)
else
	result = solve_model_with_benders(case_path,:MACRO)
end

# Export result to filename, keyed by date
# Used in 01_Examine_Multicut.ipynb
using Dates
now_unixtime = string(round(Int, datetime2unix(now())))
jldsave("benders_model_out__" * now_unixtime * ".jld2"; benders_res = result)
