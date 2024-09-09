function init_subproblem(system::System, planning_variables::Vector{String})

    subproblem = generate_operation_subproblem(system)

    set_silent(subproblem)

    subproblem_optimizer = optimizer_with_attributes(()->Main.Gurobi.Optimizer(Main.GRB_ENV),"Method"=>2,"BarConvTol"=>1e-3,"Crossover"=>1)

    set_optimizer(subproblem,subproblem_optimizer)

    return subproblem
end

function init_local_subproblems!(system_local::Vector{System},subproblems_local::Vector{Dict{Any,Any}},planning_variables_local::Vector{Dict{Int64,Vector{String}}})

    nW = length(system_local)

    for i=1:nW
		subproblem = init_subproblem(system_local[i],planning_variables);
        subproblems_local[i][:model] = subproblem;
        subproblems_local[i][:planning_variables_sub] = planning_variables_local[i];
        subproblems_local[i][:subperiod_index] = subperiod_indices(system_local[i][1])[1]
    end
end

function init_dist_subproblems(system_decomp::Dict{Int64, System},planning_variables_sub::Dict{Int64,Vector{String}})

    ##### Initialize a distributed arrays of JuMP models
	## Start pre-solve timer
	subproblem_generation_time = time()
    
    subproblems_all = distribute([Dict() for i in 1:length(system_decomp)]);

    @sync for p in workers()
        @async @spawnat p begin
            W_local = localindices(subproblems_all)[1];
            system_local = [system_decomp[k] for k in W_local];
            planning_variables_local = [planning_variables_sub[k] for k in W_local];
            init_local_subproblems!(system_local,localpart(subproblems_all),planning_variables_local);
        end
    end

	# p_id = workers();
    # np_id = length(p_id);

    # planning_variables_sub = [Dict() for k in 1:np_id];

    # @sync for k in 1:np_id
    #           @async planning_variables_sub[k]= @fetchfrom p_id[k] get_local_planning_variables(localpart(subproblems_all))
    # end

	# planning_variables_sub = merge(planning_variables_sub...);

    ## Record pre-solver time
	subproblem_generation_time = time() - subproblem_generation_time
	println("Distributed operational subproblems generation took $subproblem_generation_time seconds")

    return subproblems_all

end

# function get_local_planning_variables(subproblems_local::Vector{Dict{Any,Any}})

#     local_variables=Dict();

#     for sp in subproblems_local
# 		w = sp[:subperiod_index];
#         local_variables[w] = sp[:planning_variables_sub]
#     end

#     return local_variables


# end

function solve_dist_subproblems(m_subproblems::DArray{Dict{Any, Any}, 1, Vector{Dict{Any, Any}}},planning_sol::NamedTuple)

    p_id = workers();
    np_id = length(p_id);

    sub_results = [Dict() for k in 1:np_id];

    @sync for k in 1:np_id
              @async sub_results[k]= @fetchfrom p_id[k] solve_local_subproblem(localpart(m_subproblems),planning_sol); ### This is equivalent to fetch(@spawnat p .....)
    end

	sub_results = merge(sub_results...);

    return sub_results
end

function solve_local_subproblem(subproblem_local::Vector{Dict{Any,Any}},planning_sol::NamedTuple)

    local_sol=Dict();
    for sp in subproblem_local
        m = sp[:model];
        planning_variables_sub = sp[:planning_variables_sub]
        w = sp[:subperiod_index];
		local_sol[w] = solve_subproblem(m,planning_sol,planning_variables_sub);
    end
    return local_sol
end

function solve_subproblem(m::Model,planning_sol::NamedTuple,planning_variables_sub::Vector{String})

	
	fix_planning_variables!(m,planning_sol,planning_variables_sub)

	optimize!(m)
	
	if has_values(m)
		op_cost = objective_value(m);
		lambda = [dual(FixRef(variable_by_name(m,y))) for y in planning_variables_sub];
		theta_coeff = 1;	
	else
		op_cost = 0;
		lambda = zeros(length(planning_variables_sub));
		theta_coeff = 0;
        # compute_conflict!(m)
		# 		list_of_conflicting_constraints = ConstraintRef[];
		# 		for (F, S) in list_of_constraint_types(m)
		# 			for con in all_constraints(m, F, S)
		# 				if get_attribute(con, MOI.ConstraintConflictStatus()) == MOI.IN_CONFLICT
		# 					push!(list_of_conflicting_constraints, con)
		# 				end
		# 			end
		# 		end
        #         display(list_of_conflicting_constraints)
		@warn "The subproblem solution failed. This should not happen, double check the input files"
	end
    
	return (op_cost=op_cost,lambda = lambda,theta_coeff=theta_coeff)

end

function fix_planning_variables!(m::Model,planning_sol::NamedTuple,planning_variables_sub::Vector{String})
	for y in planning_variables_sub
		vy = variable_by_name(m,y);
		fix(vy,planning_sol.values[y];force=true)
		if is_integer(vy)
			unset_integer(vy)
		elseif is_binary(vy)
			unset_binary(vy)
		end
	end
end

 