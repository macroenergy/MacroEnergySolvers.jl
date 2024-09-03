function solve_int_level_set_problem(m::Model,linking_variables::Vector{String},planning_sol::NamedTuple,LB,UB,γ)
	
	@constraint(m,cLevel_set,m[:eFixedCost] + m[:eApproximateVariableCost]<=LB+γ*(UB-LB))

	@objective(m,Min, 0*m[:eApproximateVariableCost])

    optimize!(m)

	if has_values(m)
		if check_negative_capacity(m)
			@warn  "skipping regularization because of negative capacities"
		else
			planning_sol = (;planning_sol..., fixed_cost=value(m[:eFixedCost]), values=Dict([s=>value(variable_by_name(m,s)) for s in linking_variables]), theta = value.(m[:vTHETA]))
		end
	else
		
		@warn  "the interior level set problem solution failed"

	end

	delete(m,m[:cLevel_set])
	unregister(m,:cLevel_set)
	@objective(m,Min, m[:eFixedCost] + m[:eApproximateVariableCost])
	
	return planning_sol

end
