using Debugger

function benders(planning_problem::Model,linking_variables::Vector{String},subproblems::DistributedArrays.DArray,linking_variables_sub::Dict)
	
	#### Algorithm from:
	### Pecci, F. and Jenkins, J. D. “Regularized Benders Decomposition for High Performance Capacity Expansion Models”. arXiv:2403.02559 [math]. URL: http://arxiv.org/abs/2403.02559.

	### It's a regularized version of the Benders decomposition algorithm in:
	### A. Jacobson, F. Pecci, N. Sepulveda, Q. Xu, and J. Jenkins, “A computationally efficient Benders decomposition for energy systems planning problems with detailed operations and time-coupling constraints.” INFORMS Journal on Optimization 6(1):32-45. doi: https://doi.org/10.1287/ijoo.2023.0005

	## Start solver time
	solver_start_time = time()
	
	#### Algorithm parameters:
	MaxIter = 300;
	ConvTol = 1e-3;
	MaxCpuTime = 3600;
	γ = 0.5;
	stab_method ="int_level_set";

	multicut_removal_method = :none
	if !(multicut_removal_method in [:similar_dist, :repeated_nonbinding, :none])
		throw(DomainError(multicut_removal_method, "Invalid multicut_removal_method"))
	end

	integer_investment = false;
	# stab_method = "dynamic"; #dynamic or fixed 
	cut_selection_method = "norm";
	#############	

	integer_routine_flag = false

	if integer_investment == 1 && stab_method != "off"
		all_linking_variables = all_variables(planning_problem);
		integer_variables = all_linking_variables[is_integer.(all_linking_variables)];
		binary_variables = all_linking_variables[is_binary.(all_linking_variables)];
		unset_integer.(integer_variables)
		unset_binary.(binary_variables)
		integer_routine_flag = true;
	end

	#### Initialize UB and LB
	planning_sol = solve_planning_problem(planning_problem,linking_variables);

	UB = Inf;
	LB = planning_sol.LB;

	LB_hist = Float64[];
	UB_hist = Float64[];
	 # TODO add type, and add optionality for when not debugging
	 multicut_hists = [];
	cpu_time = Float64[];

	subop_sol_hist = Vector{Dict{Any, Any}}()

	planning_sol_best = deepcopy(planning_sol);

	# TODO add types, then update function format below
	constraints_dict_tocheck_binding = Dict()

	#### Run Benders iterations
	for k = 1:MaxIter
		
		start_subop_sol = time();

		subop_sol = solve_dist_subproblems(subproblems,planning_sol);
		
		cpu_subop_sol = time()-start_subop_sol;
		println("Solving the subproblems required $cpu_subop_sol seconds")

		UBnew = sum((subop_sol[w].theta_coeff==0 ? Inf : subop_sol[w].op_cost) for w in keys(subop_sol))+planning_sol.inv_cost;
		if UBnew < UB
			planning_sol_best = deepcopy(planning_sol);
			UB = UBnew;
		end

		push!(subop_sol_hist, deepcopy(subop_sol))	# store the subproblem solution

		# println(typeof(subop_sol))

		# println(subop_sol_hist)

		print("Updating the planning problem....")
		time_start_update = time()

		println(k)
		(multicut_hist, constraints_dict_tocheck_binding) = update_planning_problem_multi_cuts!(
			planning_problem, subop_sol, subop_sol_hist, planning_sol,
			linking_variables_sub, k, multicut_removal_method,
			constraints_dict_tocheck_binding )
		
		time_planning_update = time()-time_start_update
		println("done (it took $time_planning_update s).")

		start_planning_sol = time()
		unst_planning_sol = solve_planning_problem(planning_problem,linking_variables);
		cpu_planning_sol = time()-start_planning_sol;
		println("Solving the planning problem required $cpu_planning_sol seconds")

		LB = max(LB,unst_planning_sol.LB);
		

		# For multiCut_added constraints, if dual is = 0, then is binding, so
		# add to multiCut_added constraints_dict_tocheck_binding's count
		# If non-binding, reset count
		for (nm, con_dual) in unst_planning_sol.GTE_cons_names_dual
			# TODO abs?, within thresh?
			# Non binding, accumulate
			if abs(con_dual) == 0
				constraints_dict_tocheck_binding[nm][2] += 1
			# Reset count
			else
				constraints_dict_tocheck_binding[nm][2] = 0
			end
			# Keep track of dual for later plotting
			constraints_dict_tocheck_binding[nm][3] = con_dual
		end
		
		append!(LB_hist, LB)
		append!(UB_hist, UB)
		push!(multicut_hists, multicut_hist)
		append!(cpu_time, time() - solver_start_time)


		if any(subop_sol[w].theta_coeff==0 for w in keys(subop_sol))
			println("***k = ", k,"	  LB = ", LB,"	 UB = ", UB,"	   Gap = ", (UB-LB)/abs(LB),"	   CPU Time = ",cpu_time[end])
		else
			println("k = ", k,"	  LB = ", LB,"	 UB = ", UB,"	   Gap = ", (UB-LB)/abs(LB),"	   CPU Time = ",cpu_time[end])
		end

		if (UB-LB)/abs(LB) <= ConvTol
			if integer_routine_flag
				println("*** Switching on integer constraints *** ")
				UB = Inf;
				set_integer.(integer_variables)
				set_binary.(binary_variables)
				planning_sol = solve_planning_problem(planning_problem,linking_variables);
				LB = planning_sol.LB;
				planning_sol_best = deepcopy(planning_sol);
				integer_routine_flag = false;
			else
				break
			end
		elseif (cpu_time[end] >= MaxCpuTime)|| (k == MaxIter)
			break
		elseif UB==Inf
			planning_sol = deepcopy(unst_planning_sol);
		else
			if stab_method == "int_level_set"

				if k >= 2

					r=(UB_hist[k-1]-UB_hist[k])/(UB_hist[k-1]-(LB_hist[k]+γ*(UB_hist[k-1]-LB_hist[k])))
					ap=(UB_hist[k-1]-UB_hist[k])
					pp=(UB_hist[k-1]-(LB_hist[k]+γ*(UB_hist[k-1]-LB_hist[k])))
					println(r, ap, pp)
					if ap>=0 && pp>=0
						if r<=0.2
							γ=0.9-0.5*(0.9-γ)
							println("Increase γ: ", γ)
						elseif 0.2<r<0.8
							γ=γ
							println("Keep γ: ", γ)
						else
							γ=0.5*γ
							println("Decrease γ: ", γ)
						end
					end
				end

				start_stab_method = time()
				if  integer_investment==1 && integer_routine_flag==false
					unset_integer.(integer_variables)
					unset_binary.(binary_variables)
					for v in integer_variables
						fix(v,unst_planning_sol.values[name(v)];force=true)
					end
					for v in binary_variables
						fix(v,unst_planning_sol.values[name(v)];force=true)
					end
					println("Solving the interior level set problem with γ = $γ")

					planning_sol = solve_int_level_set_problem(planning_problem,linking_variables,unst_planning_sol,LB,UB,γ);
					unfix.(integer_variables)
					unfix.(binary_variables)
					set_integer.(integer_variables)
					set_binary.(binary_variables)
					set_lower_bound.(integer_variables,0.0)
					set_lower_bound.(binary_variables,0.0)
				else
					println("Solving the interior level set problem with γ = $γ")
					planning_sol = solve_int_level_set_problem(planning_problem,linking_variables,unst_planning_sol,LB,UB,γ);
				end
				cpu_stab_method = time()-start_stab_method;
				println("Solving the interior level set problem required $cpu_stab_method seconds")
			else
				planning_sol = deepcopy(unst_planning_sol);
			end

		end

	end

	return (planning_problem=planning_problem, planning_sol=planning_sol_best,
			LB_hist=LB_hist, UB_hist=UB_hist, multicut_hists=multicut_hists,
			cpu_time=cpu_time, subop_sol_hist=subop_sol_hist)
end


# Finds relative vector distance.
# Note: If wrap argument in `Ref` then will broadcast on the other argument
function relVector(vecA::Vector{<:AbstractFloat}, vecB::Vector{<:AbstractFloat})
	return norm(vecA .- vecB) / norm(vecA)
end
# The `op_cost` field is single float, so find its relative distance too
function relVector(A::AbstractFloat, B::AbstractFloat)
	# print(vecA); println(vecB)
	return abs(A - B)/abs(A)
end

function update_planning_problem_multi_cuts!(m::Model,subop_sol::Dict,
			subop_sol_hist::Vector{Dict{Any,Any}},
			planning_sol::NamedTuple, linking_variables_sub::Dict, k::Int,
			multicut_removal_method::Symbol, constraints_dict_tocheck_binding)
	println(k)
	W = keys(subop_sol);

	# Store a dictionary holding logs for later analysis
	logDict_perW = Dict()

	if k >= 2
		# Pass in number of times repeatedly nonbinding, and dual value for logs
		logDict_perW["dualvals_and_repeated_nonbinding" * string(k)] = [x[2:3] for x in values(constraints_dict_tocheck_binding)]

		REPEATED_NONBINDING_THRESH = 3
		# for (con, repeated_nonbinding, dualVal) in values(constraints_dict_tocheck_binding)
		for (conName, conDictVal) in constraints_dict_tocheck_binding
			if conDictVal[2] > REPEATED_NONBINDING_THRESH

				if multicut_removal_method == :repeated_nonbinding
					println("DELETING CON: Repeated Nonbinding")
					delete(m, conDictVal[1])
					delete!(constraints_dict_tocheck_binding, conName)
				end	
			end
		end
		for w in W
			avoid = false
			# QUESTION: 1:(k-1) or just check previous. Then why for loop?
			# QUESTION: why do I need to use MacroEnergySystemsDecomposition.relVector

			# TODO - doesn't shortcut early with this vectorization. Could be slower?
			# Note: `Ref` forces only to vectorize on first argument
			min_dist_lambda = minimum(
				MacroEnergySystemsDecomposition.relVector.(
					[subop_sol_hist[j][w].lambda for j in 1:(k-1)],
					Ref(subop_sol_hist[k][w].lambda))
			)
			min_dist_op_cost = minimum(
				MacroEnergySystemsDecomposition.relVector.(
					[subop_sol_hist[j][w].op_cost for j in 1:(k-1)],
					Ref(subop_sol_hist[k][w].op_cost))
			)

			logDict_perW["cut_dist_"*string(w)] = (; min_dist_lambda, min_dist_op_cost)

			TOO_CLOSE_DIST_THRESH = 0.2
			if multicut_removal_method == :similar_dist  &&
			   min_dist_lambda  <= TOO_CLOSE_DIST_THRESH &&
			   min_dist_op_cost <= TOO_CLOSE_DIST_THRESH

				println("DELETING CON: Too similar")
				avoid = true
			end
			if avoid == false
				newCon = @constraint(m,
					# QUESTION: if within the `w` for loop, only want to add this specific w?
					# Could also collect the non-avoid W's and `intersect` that list with W?
					# [w in W],
					subop_sol[w].theta_coeff * m[:vTHETA][w] >=
					subop_sol[w].op_cost +
					sum(
						subop_sol[w].lambda[i] *
						(variable_by_name(m, linking_variables_sub[w][i]) -
						 planning_sol.values[linking_variables_sub[w][i]])
						for i in 1:length(linking_variables_sub[w])))

				this_con_name = "multiCut_added_" * string(k) * "_" * string(w)
				set_name(newCon, this_con_name)
				# (con, number of times repeated nonbinding, last dual value)
				constraints_dict_tocheck_binding[this_con_name] = [newCon, 0, NaN]
				end   
		end
	else 
		firstRoundCons = @constraint(m,
			[w in W],
			subop_sol[w].theta_coeff * m[:vTHETA][w] >=
			subop_sol[w].op_cost +
			sum(
				subop_sol[w].lambda[i] *
				(variable_by_name(m, linking_variables_sub[w][i]) -
				 planning_sol.values[linking_variables_sub[w][i]])
				for i in 1:length(linking_variables_sub[w])
			))
		for (this_w, con) in enumerate(firstRoundCons)
			this_con_name = "multiCut_added_" * string(k) * "_" * string(this_w)
			# (con, number of times repeated nonbinding, last dual value)
			constraints_dict_tocheck_binding[this_con_name] = [con, 0, NaN]
			set_name(con, this_con_name)
		end
	end
	return (; logDict_perW, constraints_dict_tocheck_binding)

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