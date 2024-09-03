
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
        add_slacks_to_subproblem!(sp[:model],sp[:slack_penalty_value]);
    end
    return nothing
end


function add_slacks_to_subproblem!(subproblem::Model,slack_penalty_value::Union{Float64,Nothing}=nothing)
    
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

    if isnothing(slack_penalty_value)
        fix.(slack_max,0.0); #Note: no need to have force=true because the slacks do not have variable bounds
    else
        set_objective_coefficient(subproblem, slack_max, slack_penalty_value);
    end


    return nothing
end

function fix_linking_variables!(m::Model,planning_sol::NamedTuple,linking_variables_sub::Vector{String})

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

function solve_subproblem(m::Model,planning_sol::NamedTuple,linking_variables_sub::Vector{String})

	
	fix_linking_variables!(m,planning_sol,linking_variables_sub)

	optimize!(m)
	
	if has_values(m)
		op_cost = objective_value(m);
		lambda = [dual(FixRef(variable_by_name(m,y))) for y in linking_variables_sub];
		theta_coeff = 1;	
	else
        @info "Subproblem is infeasible, generating feasibility cut..."
        # if !has_values(m)
        #     compute_conflict!(m)
        #     list_of_conflicting_constraints = ConstraintRef[];
        #     for (F, S) in list_of_constraint_types(m)
        #         for con in all_constraints(m, F, S)
        #             if get_attribute(con, MOI.ConstraintConflictStatus()) == MOI.IN_CONFLICT
        #                 push!(list_of_conflicting_constraints, con)
        #             end
        #         end
        #     end
        #     display(list_of_conflicting_constraints)
        # end
        
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
        end
        op_cost = objective_value(m);
        lambda = [dual(FixRef(variable_by_name(m,y))) for y in linking_variables_sub];
		theta_coeff = 0;	

        fix.(m[:slack_max],0.0);

        @objective(m, Min, objfun)

		# op_cost = 0;
		# lambda = zeros(length(linking_variables_sub));
		# theta_coeff = 0;

	end
    
	return (op_cost=op_cost,lambda = lambda,theta_coeff=theta_coeff)

end


function solve_local_subproblems(subproblem_local::Vector{Dict{Any,Any}},planning_sol::NamedTuple)

    local_sol=Dict();
    for sp in subproblem_local
        m = sp[:model];
        linking_variables_sub = sp[:linking_variables_sub]
        w = sp[:subproblem_index];
		local_sol[w] = solve_subproblem(m,planning_sol,linking_variables_sub);
    end
    return local_sol
end


function solve_subproblems(m_subproblems::DArray{Dict{Any, Any}, 1, Vector{Dict{Any, Any}}},planning_sol::NamedTuple)

    p_id = workers();
    np_id = length(p_id);

    sub_results = [Dict() for k in 1:np_id];

    @sync for k in 1:np_id
              @async sub_results[k]= @fetchfrom p_id[k] solve_local_subproblems(localpart(m_subproblems),planning_sol); ### This is equivalent to fetch(@spawnat p .....)
    end

	sub_results = merge(sub_results...);

    return sub_results
end


function solve_subproblems(m_subproblems::Vector{Dict{Any, Any}},planning_sol::NamedTuple)
    
    sub_results = solve_local_subproblems(m_subproblems,planning_sol); 

    return sub_results
end
