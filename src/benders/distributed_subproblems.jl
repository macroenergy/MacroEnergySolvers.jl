function initialize_subproblem(system::Any,model::Module)
    
    subproblem_optimizer = optimizer_with_attributes(()->Gurobi.Optimizer(GRB_ENV[]),"Method"=>2,"BarConvTol"=>1e-3,"Crossover"=>1)

    subproblem,linking_variables_sub = model.generate_operation_subproblem(system);

    subperiod_index = model.get_subperiod_index(system);

    set_optimizer(subproblem,subproblem_optimizer)

    set_silent(subproblem)

    return subproblem,linking_variables_sub,subperiod_index
end

function initialize_local_subproblems(system_local::Vector,subproblems_local::Vector{Dict{Any,Any}},model::Module)

    nW = length(system_local)

    for i=1:nW
		subproblem,linking_variables_sub,subperiod_index = initialize_subproblem(system_local[i],model);
        subproblems_local[i][:model] = subproblem;
        subproblems_local[i][:linking_variables_sub] = linking_variables_sub;
        subproblems_local[i][:subperiod_index] = subperiod_index;
    end
end

function initialize_dist_subproblems(system_decomp::Dict,model::Module)

    ##### Initialize a distributed arrays of JuMP models
	## Start pre-solve timer
	subproblem_generation_time = time()


    subproblems_all = distribute([Dict() for i in 1:length(system_decomp)]);

    @sync for p in workers()
        @async @spawnat p begin
            W_local = localindices(subproblems_all)[1];
            system_local = [system_decomp[k] for k in W_local];
            initialize_local_subproblems(system_local,localpart(subproblems_all),model);
        end
    end

	p_id = workers();
    np_id = length(p_id);

    linking_variables_sub = [Dict() for k in 1:np_id];

    @sync for k in 1:np_id
              @async linking_variables_sub[k]= @fetchfrom p_id[k] get_local_linking_variables(localpart(subproblems_all))
    end

	linking_variables_sub = merge(linking_variables_sub...);

    ## Record pre-solver time
	subproblem_generation_time = time() - subproblem_generation_time
	println("Distributed operational subproblems generation took $subproblem_generation_time seconds")

    return subproblems_all,linking_variables_sub

end

function get_local_linking_variables(subproblems_local::Vector{Dict{Any,Any}})

    local_variables=Dict();

    for sp in subproblems_local
		w = sp[:subperiod_index];
        local_variables[w] = sp[:linking_variables_sub]
    end

    return local_variables


end


function solve_subproblem(m::Model,planning_sol::NamedTuple,linking_variables_sub::Vector{String})

	
	fix_linking_variables!(m,planning_sol,linking_variables_sub)

	optimize!(m)
	
	if has_values(m)
		op_cost = objective_value(m);
		lambda = [dual(FixRef(variable_by_name(m,y))) for y in linking_variables_sub];
		theta_coeff = 1;	
	else
		op_cost = 0;
		lambda = zeros(length(linking_variables_sub));
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
		@warn "The subproblem solution failed. TerminationStatus: $(termination_status(m)), This should not happen, double check the input files"
	end
    
	return (op_cost=op_cost,lambda = lambda,theta_coeff=theta_coeff)

end


function solve_local_subproblem(subproblem_local::Vector{Dict{Any,Any}},planning_sol::NamedTuple)

    local_sol=Dict();
    for sp in subproblem_local
        m = sp[:model];
        linking_variables_sub = sp[:linking_variables_sub]
        w = sp[:subperiod_index];
		local_sol[w] = solve_subproblem(m,planning_sol,linking_variables_sub);
    end
    return local_sol
end


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