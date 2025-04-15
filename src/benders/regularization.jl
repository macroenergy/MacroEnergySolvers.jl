"""
    solve_int_level_set_problem(m::Model, 
		linking_variables::Vector{String}, 
		planning_sol::NamedTuple, 
		LB, 
		UB, 
		γ
	)

Solves the interior level set stabilization problem for the regularized Benders decomposition algorithm.

This stabilization technique helps improve convergence by restricting the master problem solution
to lie within a level set defined by the current lower and upper bounds, controlled by the
stabilization parameter γ.

# Arguments
- `m::Model`: The JuMP model representing the planning problem
- `linking_variables::Vector{String}`: Names of the variables linking the master and subproblems
- `planning_sol::NamedTuple`: Current solution of the planning problem
- `LB`: Current lower bound
- `UB`: Current upper bound
- `γ`: Stabilization parameter controlling the size of the level set (0 ≤ γ ≤ 1)

# Returns
A NamedTuple containing the solution of the stabilized problem with the same structure as the input `planning_sol`

"""
function solve_int_level_set_problem(m::Model,linking_variables::Vector{String},planning_sol::NamedTuple,LB,UB,γ)
	
	### Interior point regularization based on https://ieeexplore.ieee.org/document/10829583

	@constraint(m,cLevel_set,m[:eFixedCost] + m[:eApproximateVariableCost]<=LB+γ*(UB-LB))

	@objective(m,Min, 0*m[:eApproximateVariableCost])

    optimize!(m)

	if has_values(m)

		fixed_cost,variable_values = process_planning_sol(m,linking_variables)

		planning_sol = (;planning_sol..., fixed_cost = fixed_cost, values = variable_values)
		
	else
		
		@warn  "the interior level set problem solution failed"

	end

	delete(m,m[:cLevel_set])
	unregister(m,:cLevel_set)
	@objective(m,Min, m[:eFixedCost] + m[:eApproximateVariableCost])
	
	return planning_sol

end
