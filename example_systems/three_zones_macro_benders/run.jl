using Pkg
Pkg.activate(dirname(dirname(@__DIR__)))

case_path = @__DIR__

println("###### ###### ######")
println("Running case at $(case_path)")

using MacroEnergySystemsDecomposition

start_distributed_processes!(2,:MACRO)

# system = Macro.load_system(case_path);
# system_decomp = Macro.generate_decomposed_system(system);
# planning_model,linking_variables = Macro.generate_planning_problem(system);
# operation_model,linking_variables_sub = Macro.generate_operation_subproblem(system_decomp[5]);

# # _,planning_sol,LB_hist,UB_hist,cpu_time  = Macro.benders(benders_models);


println()