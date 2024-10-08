function initialize_planning_problem(system::Any,model_type::Symbol)

    if model_type==:MACRO
        planning_problem,linking_variables = Macro.generate_planning_problem(system);
    end

    planning_optimizer = optimizer_with_attributes(()->Gurobi.Optimizer(GRB_ENV[]),"Method"=>2,"BarConvTol"=>1e-3,"Crossover"=>0,"MIPGap"=>1e-3)

    set_optimizer(planning_problem,planning_optimizer)

    set_silent(planning_problem)

    return planning_problem,linking_variables

end

function solve_planning_problem(m::Model,linking_variables::Vector{String})
	
	
    optimize!(m)

    if has_values(m) #
		# While at the thread where `optimize!`, capture the benders cuts'
		# duals, so can tell if is binding
		# These `multiCut_added_` (benders cuts) constraints are all GreaterThan
		# constraints
		# Technically, for now, seems to be the *only* one of these but also do
		# future-proof filtering
		GTE_cons = all_constraints(m, AffExpr, MOI.GreaterThan{Float64})
		# Collect tuples of names and dual values to return
		GTE_cons_names_dual = collect(zip(
			name.(GTE_cons), dual.(GTE_cons)
		))
		# Filter to only multicut_added named ones
        GTE_cons_names_dual = filter(tup -> startswith(tup[1], "multiCut_added_"),
									GTE_cons_names_dual)

        planning_sol = (
            LB=objective_value(m),
            inv_cost=value(m[:eFixedCost]),
            values=Dict([s => value.(variable_by_name(m, s)) for s in linking_variables]),
			theta=value.(m[:vTHETA]),
			GTE_cons_names_dual = GTE_cons_names_dual
			)
    else
        compute_conflict!(m)
        list_of_conflicting_constraints = ConstraintRef[];
        for (F, S) in list_of_constraint_types(m)
            for con in all_constraints(m, F, S)
                if get_attribute(con, MOI.ConstraintConflictStatus()) == MOI.IN_CONFLICT
                    push!(list_of_conflicting_constraints, con)
                end
            end
        end
        display(list_of_conflicting_constraints)
        @error "The planning solution failed. This should not happen"
    end

    return planning_sol
end