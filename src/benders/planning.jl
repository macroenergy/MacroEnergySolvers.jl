function solve_planning_problem(m::Model,linking_variables::Vector{String})
	
	
    optimize!(m)

    if has_values(m)
        fixed_cost, linking_variables_values = process_planning_sol(m::Model,linking_variables::Vector{String})
        planning_sol =  (LB = objective_value(m), fixed_cost = fixed_cost, values = linking_variables_values)
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
        @error "The planning solution failed. This should not happen."
    end

    return planning_sol
end


function process_planning_sol(m::Model,linking_variables::Vector{String})

    capacity_variables = values(m[:eAvailableCapacity])

    if any(value(vcap) < -1e-8 for vcap in capacity_variables)
        @info "Found negative capacity values, setting them to zero."
        planning_variables_values = Dict();
        all_planning_variables = all_variables(m)
        for v in all_planning_variables
            if in(v,capacity_variables)
                planning_variables_values[v] = round_small_values(value(v))
            else
                planning_variables_values[v] = value(v)
            end
        end
        fixed_cost = value(x->planning_variables_values[x], m[:eFixedCost])
        linking_variables_values = Dict([s => planning_variables_values[variable_by_name(m,s)] for s in linking_variables])
    else
        fixed_cost = value(m[:eFixedCost])
        linking_variables_values = Dict([s=>value.(variable_by_name(m,s)) for s in linking_variables])
    end

    return fixed_cost, linking_variables_values
    
end

function round_small_values(z::Float64)
    if z < -1e-8
        return 0.0
    else
        return z
    end
end