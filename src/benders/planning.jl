function solve_planning_problem(m::Model,linking_variables::Vector{String})
	
	
    optimize!(m)

    if has_values(m)
        
        # values = Dict([s=>round_small_values(value(variable_by_name(m,s))) for s in linking_variables])
        # point = Dict(variable_by_name(m,s) => values[s] for s in linking_variables)
        # @show (values)
        # @show (point)
        # fixed_cost = value(x->point[x], m[:eFixedCost])
        # planning_sol =  (LB = objective_value(m), fixed_cost = fixed_cost, values = values) 

        if check_negative_capacity(m)
            @info "Resolving the planning problem with Crossover = 1 because of negative capacities"
            set_optimizer_attribute(m, "Crossover", 1)
            optimize!(m)
            if has_values(m)
                planning_sol =  (LB = objective_value(m), fixed_cost =value(m[:eFixedCost]), values =Dict([s=>value.(variable_by_name(m,s)) for s in linking_variables]), theta = value.(m[:vTHETA])) 
            end
            set_optimizer_attribute(m, "Crossover", 0)
        else
            planning_sol =  (LB = objective_value(m), fixed_cost =value(m[:eFixedCost]), values =Dict([s=>value.(variable_by_name(m,s)) for s in linking_variables]), theta = value.(m[:vTHETA])) 
        end
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


function check_negative_capacity(m::Model)
    return any(value(m[:eAvailableCapacity][y]) < -1e-8 for y in keys(m[:eAvailableCapacity]))
end

function round_small_values(z::Float64)
    if abs(z) < 1e-6
        return 0.0
    else
        return z
    end
end