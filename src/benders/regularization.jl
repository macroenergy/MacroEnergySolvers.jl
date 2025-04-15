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
