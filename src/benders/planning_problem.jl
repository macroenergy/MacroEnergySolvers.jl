function initialize_planning_problem!(system::Any,model::Module)

    planning_optimizer = optimizer_with_attributes(()->Gurobi.Optimizer(GRB_ENV[]),"Method"=>2,"BarConvTol"=>1e-3,"Crossover"=>0,"MIPGap"=>1e-3)
    
    planning_problem,linking_variables = model.generate_planning_problem(system);
    
    set_optimizer(planning_problem,planning_optimizer)

    set_silent(planning_problem)

    return planning_problem,linking_variables

end

function solve_planning_problem(m::Model,linking_variables::Vector{String})
	
	
    optimize!(m)

    if has_values(m) #
        planning_sol =  (LB = objective_value(m), inv_cost =value(m[:eFixedCost]), values =Dict([s=>value.(variable_by_name(m,s)) for s in linking_variables]), theta = value.(m[:vTHETA])) 
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