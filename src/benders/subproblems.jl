
function add_slacks_to_subproblems!(m_subproblems::Vector{Dict{Any, Any}})
    
    add_slacks_to_local_subproblems!(m_subproblems); 

    return nothing
end

function add_slacks_to_subproblems!(m_subproblems::DArray{Dict{Any, Any}, 1, Vector{Dict{Any, Any}}})

    @sync for p in workers()
        @async @spawnat p begin
            add_slacks_to_local_subproblems!(localpart(m_subproblems));
        end
    end

    return nothing
end

function add_slacks_to_local_subproblems!(subproblem_local::Vector{Dict{Any,Any}})

    for sp in subproblem_local
        add_slacks_to_subproblem!(sp[:model]);
    end
    return nothing
end


function add_slacks_to_subproblem!(subproblem::Model)
    ### Slack variables are added to the subproblems and fixed to zero. 
    ### We will then allow slack variables to be non-zero to generate feasibility cuts when a subproblem is infeasible.

    eq_cons =  all_constraints(subproblem,AffExpr,MOI.EqualTo{Float64})
    less_ineq_cons = all_constraints(subproblem,AffExpr,MOI.LessThan{Float64})
    greater_ineq_cons = all_constraints(subproblem,AffExpr,MOI.GreaterThan{Float64})


    @variable(subproblem, slack_max)
    @constraint(subproblem, slack_max >= 0)

    if !isempty(less_ineq_cons)
        for c in less_ineq_cons
            set_normalized_coefficient(c, slack_max, -1)
        end
    end

    if !isempty(greater_ineq_cons)
        for c in greater_ineq_cons
            set_normalized_coefficient(c, slack_max, 1)
        end
    end

    if !isempty(eq_cons)
        n = length(eq_cons)
        @variable(subproblem, slack_eq[1:n])
        for i in 1:n
            set_normalized_coefficient(eq_cons[i], slack_eq[i], -1)
        end
        @constraint(subproblem, [i in 1:n], slack_eq[i] <= slack_max)
        @constraint(subproblem, [i in 1:n], -slack_eq[i] <= slack_max)
    end

        fix.(slack_max,0.0); 
    

    return nothing
end

function fix_linking_variables!(m::Model,planning_sol::NamedTuple,linking_variables_sub::Vector{String})
    ### Fix linking variables in the subproblem to the values computed by the planning problem. 
	for y in linking_variables_sub
		vy = variable_by_name(m,y);
		fix(vy,planning_sol.values[y];force=true)
		if is_integer(vy)
			unset_integer(vy)
		elseif is_binary(vy)
			unset_binary(vy)
		end
	end
end

function solve_subproblem(m::Model,planning_sol::NamedTuple,linking_variables_sub::Vector{String},expect_feasible_subproblems::Bool)
	
    ### Solve the operational subproblem. If it is infeasible, compute feasibility cuts.

	fix_linking_variables!(m,planning_sol,linking_variables_sub)

	optimize!(m)
	
	if has_values(m)
		op_cost = objective_value(m);
		lambda = [dual(FixRef(variable_by_name(m,y))) for y in linking_variables_sub];
		theta_coeff = 1;	
    elseif expect_feasible_subproblems==true
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
        @error "The subproblem is infeasible, but ExpectFeasibleSubproblems = true. Set it to false to generate feasibility cuts."
    else
        @info "Subproblem is infeasible, generating feasibility cut..."
        #### Feasibility cuts generation based on https://link.springer.com/chapter/10.1007/978-3-030-45771-6_7 
        
        unfix.(m[:slack_max]);
        objfun = objective_function(m);
        @objective(m, Min, m[:slack_max])
        
        optimize!(m)
        if !has_values(m)
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
            @error "Feasibility subproblem is infeasible, this should not happen. Check the model."
        end
        op_cost = objective_value(m);
        lambda = [dual(FixRef(variable_by_name(m,y))) for y in linking_variables_sub];
		theta_coeff = 0;	

        fix.(m[:slack_max],0.0);

        @objective(m, Min, objfun)

	end
    
	return (op_cost=op_cost,lambda = lambda,theta_coeff=theta_coeff)

end


function solve_local_subproblems(subproblem_local::Vector{Dict{Any,Any}},planning_sol::NamedTuple, expect_feasible_subproblems::Bool)

    local_sol=Dict();
    for sp in subproblem_local
        m = sp[:model];
        linking_variables_sub = sp[:linking_variables_sub]
        w = sp[:subproblem_index];
		local_sol[w] = solve_subproblem(m,planning_sol,linking_variables_sub,expect_feasible_subproblems);
    end
    return local_sol
end

"""
    solve_subproblems(
        m_subproblems::DArray{Dict{Any, Any}, 1, Vector{Dict{Any, Any}}}, 
        planning_sol::NamedTuple
    )

Solves subproblems in parallel using distributed computing capabilities.

This function coordinates the parallel solution of operational subproblems across multiple workers,
using Julia's distributed computing framework. Each worker processes its local portion of the
distributed array of subproblems.

# Arguments
- `m_subproblems::DArray`: Distributed array containing the subproblems, where each element is a
   dictionary representing a subproblem
- `planning_sol::NamedTuple`: Current solution of the planning problem containing variable values
   needed for the subproblem solutions

# Returns
A merged dictionary containing results from all subproblems, where each entry contains:
- Optimal objective value
- Dual variables
- Other solution information from each subproblem

# Implementation Details
Uses `@sync` and `@async` for coordinated parallel execution, with results fetched from each worker
and merged into a single dictionary containing all subproblem solutions.
"""
function solve_subproblems(m_subproblems::DArray{Dict{Any, Any}, 1, Vector{Dict{Any, Any}}},planning_sol::NamedTuple,expect_feasible_subproblems::Bool)

    p_id = workers();
    np_id = length(p_id);

    sub_results = [Dict() for k in 1:np_id];

    @sync for k in 1:np_id
              @async sub_results[k]= @fetchfrom p_id[k] solve_local_subproblems(localpart(m_subproblems),planning_sol,expect_feasible_subproblems); ### This is equivalent to fetch(@spawnat p .....)
    end

	sub_results = merge(sub_results...);

    return sub_results
end


function solve_subproblems(m_subproblems::Vector{Dict{Any, Any}},planning_sol::NamedTuple,expect_feasible_subproblems::Bool)
    
    sub_results = solve_local_subproblems(m_subproblems,planning_sol,expect_feasible_subproblems); 

    return sub_results
end
