using JuMP # used for mathematical programming
using Distributed
using DistributedArrays
using ClusterManagers
using Gurobi
using Pkg
Pkg.activate(dirname(dirname(@__DIR__)))

# set gurobi
const GRB_ENV = Ref{Gurobi.Env}()
GRB_ENV[] = Gurobi.Env()

case_path = @__DIR__

println("###### ###### ######")
println("Running case at $(case_path)")

using MacroEnergySystemsDecomposition
using Macro

#solve model with benders
model_type=:MACRO

system = Macro.load_system(case_path);

system_decomp = Macro.generate_decomposed_system(system);

number_of_subperiods = length(system_decomp);



# initialise planning problem
planning_problem,linking_variables = Macro.generate_planning_problem(system)


planning_optimizer = optimizer_with_attributes(()->Gurobi.Optimizer(GRB_ENV[]),"Method"=>2,"BarConvTol"=>1e-3,"Crossover"=>0,"MIPGap"=>1e-3)

set_optimizer(planning_problem,planning_optimizer)

set_silent(planning_problem)


start_distributed_processes!(number_of_subperiods,:MACRO)

# subproblems_dict,linking_variables_sub = initialize_dist_subproblems(system_decomp,model_type)

subproblem_generation_time = time()
    
subproblems_all = distribute([Dict() for i in 1:length(system_decomp)]);

@sync for p in workers()
    @async @spawnat p begin
        W_local = localindices(subproblems_all)[1];
        system_local = [system_decomp[k] for k in W_local];
        initialize_local_subproblems!(system_local,localpart(subproblems_all),model_type);
    end
end

p_id = workers();
np_id = length(p_id);

linking_variables_sub = [Dict() for k in 1:np_id];

@sync for k in 1:np_id
            @async linking_variables_sub[k]= @fetchfrom p_id[k] get_local_linking_variables(localpart(subproblems_all))
end

linking_variables_sub = merge(linking_variables_sub...);

## Record pre-solver time
subproblem_generation_time = time() - subproblem_generation_time
println("Distributed operational subproblems generation took $subproblem_generation_time seconds")

    # return subproblems_all,linking_variables_sub

planning_problem, planning_sol, LB_hist, UB_hist, runtime = benders(planning_problem,linking_variables,subproblems_dict,linking_variables_sub)



for w in 1:52
    avoid=false
        for j in k-1
            if abs(subop_sol_hist[j][w].lambda.-subop_sol_hist[k][w].lambda) <= 1e-8 && abs(subop_sol_hist[j][w].opcost.-subop_sol_hist[k][w].optcost) <= 1e-8
                avoid=true
                break
            end
        end
        if avoid==false
            @constraint(m,[w in W],subop_sol[w].theta_coeff*m[:vTHETA][w] >= subop_sol[w].op_cost + sum(subop_sol[w].lambda[i]*(variable_by_name(m,linking_variables_sub[w][i]) - planning_sol.values[linking_variables_sub[w][i]]) for i in 1:length(linking_variables_sub[w])));
        end   
end
                

