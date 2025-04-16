"""
	benders(planning_problem::Model, 
		linking_variables::Vector{String}, 
		subproblems::Union{Vector{Dict{Any, Any}}, DistributedArrays.DArray}, 
		linking_variables_sub::Dict, 
		setup::Dict
	)

Implements a regularized Benders decomposition algorithm for solving large-scale energy systems planning problems.

## Algorithm from:
F. Pecci and J. D. Jenkins (2025). “Regularized Benders Decomposition for High Performance Capacity Expansion Models”. doi: [10.1109/TPWRS.2025.3526413](https://ieeexplore.ieee.org/document/10829583)

It's a regularized version of the Benders decomposition algorithm in:

A. Jacobson, F. Pecci, N. Sepulveda, Q. Xu, and J. Jenkins (2024). “A computationally efficient Benders decomposition for energy systems planning problems with detailed operations and time-coupling constraints.” doi: [https://doi.org/10.1287/ijoo.2023.0005](https://doi.org/10.1287/ijoo.2023.0005)

# Arguments
- `planning_problem::Model`: The master problem JuMP model representing the investment decisions
- `linking_variables::Vector{String}`: Names of the variables linking the master and subproblems
- `subproblems::Union{Vector{Dict{Any, Any}},DistributedArrays.DArray}`: Collection of operational subproblems
- `linking_variables_sub::Dict`: Mapping between subproblems and their associated linking variables
- `setup::Dict`: Algorithm parameters including:
	- `MaxIter`: Maximum number of iterations
	- `ConvTol`: Convergence tolerance
	- `MaxCpuTime`: Maximum CPU time allowed
	- `StabParam`: Stabilization parameter γ
	- `StabDynamic`: Boolean for dynamic stabilization adjustment
	- `IntegerInvestment`: Boolean for integer investment variables

# Returns
@NamedTuple containing:
- `planning_problem`: Updated master problem model
- `planning_sol`: Best solution found
- `LB_hist`: History of lower bounds
- `UB_hist`: History of upper bounds
- `cpu_time`: CPU time history
- `sol_hist`: Solution history for linking variables
"""
function benders(planning_problem::Model,linking_variables::Vector{String},subproblems::Union{Vector{Dict{Any, Any}},DistributedArrays.DArray},linking_variables_sub::Dict,setup::Dict)
	
    #### Algorithm from:
    ### F. Pecci and J. D. Jenkins (2025). “Regularized Benders Decomposition for High Performance Capacity Expansion Models”. doi: https://doi.org/10.1109/TPWRS.2025.3526413

	### It's a regularized version of the Benders decomposition algorithm in:
	### A. Jacobson, F. Pecci, N. Sepulveda, Q. Xu, and J. Jenkins (2024). “A computationally efficient Benders decomposition for energy systems planning problems with detailed operations and time-coupling constraints.” doi: https://doi.org/10.1287/ijoo.2023.0005

	# Initialize the MacroEnergySolvers logger
	old_logger = set_logger(Logging.Info)
	@info("Running Benders decomposition algorithm from `MacroEnergySolvers.jl`")
	
	add_slacks_to_subproblems!(subproblems);

	## Start solver time
	solver_start_time = time()
    
    #### Algorithm parameters:
	MaxIter = setup[:MaxIter];
    ConvTol = setup[:ConvTol];
	MaxCpuTime = setup[:MaxCpuTime];
	γ = setup[:StabParam];

	stab_dynamic = setup[:StabDynamic];

	if γ ≈ 0.0
		stab_method = "off";
	else
		stab_method = "int_level_set";
	end

    integer_investment = setup[:IntegerInvestment];

	integer_routine_flag = false

	if integer_investment == 1 && stab_method != "off"
		all_linking_variables = all_variables(planning_problem);
		integer_variables = all_linking_variables[is_integer.(all_linking_variables)];
		binary_variables = all_linking_variables[is_binary.(all_linking_variables)];
		unset_integer.(integer_variables)
		unset_binary.(binary_variables)
		integer_routine_flag = true;
	end

    #### Initialize UB and LB
	planning_sol = solve_planning_problem(planning_problem,linking_variables);

    UB = Inf;
    LB = planning_sol.LB;

    LB_hist = Float64[];
    UB_hist = Float64[];
    cpu_time = Float64[];

	planning_sol_best = deepcopy(planning_sol);

	sol_hist = [planning_sol.values[s] for s in linking_variables];

    #### Run Benders iterations
    for k = 0:MaxIter
		
		start_subop_sol = time();

		sol_hist = hcat(sol_hist, [planning_sol.values[s] for s in linking_variables])
		
        subop_sol = solve_subproblems(subproblems,planning_sol);
        
		cpu_subop_sol = time()-start_subop_sol;
		@info("Solving the subproblems required $cpu_subop_sol seconds")

		UBnew = compute_upper_bound(planning_problem,planning_sol,subop_sol);
		if UBnew < UB
			planning_sol_best = deepcopy(planning_sol);
			UB = UBnew;
		end

		print("Updating the planning problem....")
		time_start_update = time()

		update_planning_problem_multi_cuts!(planning_problem,subop_sol,planning_sol,linking_variables_sub)
		
		time_planning_update = time()-time_start_update
		println("done (it took $time_planning_update s).")

		start_planning_sol = time()

		unst_planning_sol = solve_planning_problem(planning_problem,linking_variables);

		cpu_planning_sol = time()-start_planning_sol;
		@info("Solving the planning problem required $cpu_planning_sol seconds")

		LB = max(LB,unst_planning_sol.LB);
		@info("The optimal value of the planning problem is $(unst_planning_sol.LB)")
		
		append!(LB_hist,LB)
        append!(UB_hist,UB)
        append!(cpu_time,time()-solver_start_time)

		running_gap = (UB-LB)/abs(LB)

		if any(subop_sol[w].theta_coeff==0 for w in keys(subop_sol))
			@info(string("***k = ", k,"      LB = ", round_from_tol(LB, ConvTol, 2),"     UB = ", round_from_tol(UB, ConvTol, 2),"       Gap = ", round_from_tol(running_gap, ConvTol, 2),"       CPU Time = ", round(cpu_time[end], digits=5)))
		else
			@info(string("k = ", k,"      LB = ", round_from_tol(LB, ConvTol, 2),"     UB = ", round_from_tol(UB, ConvTol, 2),"       Gap = ", round_from_tol(running_gap, ConvTol, 2),"       CPU Time = ", round(cpu_time[end], digits=5)))
		end

        if running_gap <= ConvTol
			if integer_routine_flag
				@info("*** Switching on integer constraints *** ")
				UB = Inf;
				set_integer.(integer_variables)
				set_binary.(binary_variables)
				planning_sol = solve_planning_problem(planning_problem,linking_variables);
				LB = planning_sol.LB;
				planning_sol_best = deepcopy(planning_sol);
				integer_routine_flag = false;
			else
				break
			end
		elseif (cpu_time[end] >= MaxCpuTime)|| (k == MaxIter)
			break
		elseif UB==Inf
			planning_sol = deepcopy(unst_planning_sol);
		else
			if stab_method == "int_level_set"
				if stab_dynamic == true && k >= 1
					γ = update_stab_param(γ,UB_hist[end],LB_hist[end],UB_hist[end-1],LB_hist[end-1]);
				end

				start_stab_method = time()
				if  integer_investment==1 && integer_routine_flag==false
					unset_integer.(integer_variables)
					unset_binary.(binary_variables)
					for v in integer_variables
						fix(v,unst_planning_sol.values[name(v)];force=true)
					end
					for v in binary_variables
						fix(v,unst_planning_sol.values[name(v)];force=true)
					end
                    @info("Solving the interior level set problem with γ = $γ")
					planning_sol = solve_int_level_set_problem(planning_problem,linking_variables,unst_planning_sol,LB,UB,γ);
					unfix.(integer_variables)
					unfix.(binary_variables)
					set_integer.(integer_variables)
					set_binary.(binary_variables)
					set_lower_bound.(integer_variables,0.0)
					set_lower_bound.(binary_variables,0.0)
				else
                    @info("Solving the interior level set problem with γ = $γ")
					planning_sol = solve_int_level_set_problem(planning_problem,linking_variables,unst_planning_sol,LB,UB,γ);
				end
				cpu_stab_method = time()-start_stab_method;
				@info("Solving the interior level set problem required $cpu_stab_method seconds")
			else
				planning_sol = deepcopy(unst_planning_sol);
			end

		end

    end

	# Restore the old logger
	set_logger(old_logger)
	
	return (planning_problem=planning_problem,planning_sol = planning_sol_best,LB_hist = LB_hist,UB_hist = UB_hist,cpu_time = cpu_time,sol_hist = sol_hist)
end

function update_planning_problem_multi_cuts!(m::Model,subop_sol::Dict,planning_sol::NamedTuple,linking_variables_sub::Dict)
    
	W = keys(subop_sol);
	
    @constraint(m,[w in W],subop_sol[w].theta_coeff*m[:vTHETA][w] >= subop_sol[w].op_cost + sum(subop_sol[w].lambda[i]*(variable_by_name(m,linking_variables_sub[w][i]) - planning_sol.values[linking_variables_sub[w][i]]) for i in 1:length(linking_variables_sub[w])));

end

function update_stab_param(γ,UB,LB,UB_old,LB_old)
	
	r=(UB_old-UB)/(UB_old-(LB+γ*(UB_old-LB)))
					
	ap=(UB_old-UB)
	pp=(UB_old-(LB+γ*(UB_old-LB)))
	@info(r, ap, pp)
	if ap>=0 && pp>=0
		if r<=0.2
			γ=0.9-0.5*(0.9-γ)
			@info("Increase γ: ", γ)
		elseif 0.2<r<0.8
			γ=γ
			@info("Keep γ: ", γ)
		else
			γ=0.5*γ
			@info("Decrease γ: ", γ)
		end
	end
	
	return γ
end

function compute_upper_bound(m::Model,planning_sol::NamedTuple,subop_sol::Dict)
	any(subop_sol[w].theta_coeff==0 for w in keys(subop_sol)) && return Inf;

	point = Dict(m[:vTHETA][w] => subop_sol[w].op_cost for w in keys(subop_sol));

	UB = planning_sol.fixed_cost + value(x -> point[x], m[:eApproximateVariableCost])

	return UB
end