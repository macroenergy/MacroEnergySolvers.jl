"""
    solve_planning_problem(m::Model, planning_variables::Vector{String})

Solves the planning (upper-level) problem in the Benders decomposition algorithm.

This function attempts to solve the planning problem and handles potential numerical issues,
particularly with negative capacities, by rounding small values to zero if needed.

# Arguments
- `m::Model`: The JuMP model representing the planning problem
- `planning_variables::Vector{String}`: Names of the variables of the planning problem

# Returns
A NamedTuple containing:
- `fixed_cost`: Fixed cost component of the solution 
- `values`: Dictionary mapping linking variable names to their optimal values

# Notes
If negative capacities are detected, the solver will be reconfigured with `Crossover = 1` 
and the problem will be re-solved. If the solution fails, the function will compute
and display conflicting constraints (if the solver supports it) before throwing an error.
"""
function solve_planning_problem(m::Model,planning_variables::Vector{String})
	
	
    optimize!(m)

    if has_values(m)
        planning_sol = process_planning_sol(m,planning_variables)
        LB = objective_value(m)
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

    return planning_sol, LB
end


function process_planning_sol(m::Model,planning_variables::Vector{String})

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
        planning_variables_values = Dict([s => planning_variables_values[variable_by_name(m,s)] for s in planning_variables])
    else
        fixed_cost = value(m[:eFixedCost])
        planning_variables_values = Dict([s=>value.(variable_by_name(m,s)) for s in planning_variables])
    end

    planning_sol =  (fixed_cost = fixed_cost, values = planning_variables_values)

    return planning_sol
    
end

function round_small_values(z::Float64)
    if z < -1e-8
        return 0.0
    else
        return z
    end
end