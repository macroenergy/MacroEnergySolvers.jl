
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
    integer_investment = false;
	stab_dynamic = true; #dynamic or fixed 
	cut_selection_method = "no";
	filter_planning_sol = true;
	filter_cuts = true;
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
    cpu_time = Float64[];

	subop_sol_hist = Vector{Dict{Any, Any}}()

	planning_sol_best = deepcopy(planning_sol);

    #### Run Benders iterations
    for k = 1:MaxIter
		
		start_subop_sol = time();

        subop_sol = solve_dist_subproblems(subproblems,planning_sol);

		if filter_cuts == true
			# for w in keys(subop_sol)
				# println(typeof(subop_sol), w)
				filter_cuts!(subop_sol)
			# end
		end
        
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
		update_planning_problem_multi_cuts!(planning_problem,subop_sol,subop_sol_hist, planning_sol,linking_variables_sub,k,cut_selection_method)
		
		time_planning_update = time()-time_start_update
		println("done (it took $time_planning_update s).")

		start_planning_sol = time()
		unst_planning_sol = solve_planning_problem(planning_problem,linking_variables);
		cpu_planning_sol = time()-start_planning_sol;
		println("Solving the planning problem required $cpu_planning_sol seconds")

		LB = max(LB,unst_planning_sol.LB);
		
		append!(LB_hist,LB)
        append!(UB_hist,UB)
        append!(cpu_time,time()-solver_start_time)

		if any(subop_sol[w].theta_coeff==0 for w in keys(subop_sol))
			println("***k = ", k,"      LB = ", LB,"     UB = ", UB,"       Gap = ", (UB-LB)/abs(LB),"       CPU Time = ",cpu_time[end])
		else
			println("k = ", k,"      LB = ", LB,"     UB = ", UB,"       Gap = ", (UB-LB)/abs(LB),"       CPU Time = ",cpu_time[end])
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
			if stab_method == "int_level_set" && (UB-LB)/abs(LB) > 0.01

				if stab_dynamic == true
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

		if filter_planning_sol==true
			filter_planning_sol!(planning_sol.values)
		end

    end

	return (planning_problem=planning_problem,planning_sol = planning_sol_best,LB_hist = LB_hist,UB_hist = UB_hist,cpu_time = cpu_time, subop_sol_hist = subop_sol_hist)
end

function update_planning_problem_multi_cuts!(m::Model,subop_sol::Dict,subop_sol_hist::Vector{Dict{Any, Any}}, planning_sol::NamedTuple,linking_variables_sub::Dict, k::Int, cut_selection_method::String)
    
	W = keys(subop_sol);

	if cut_selection_method == "norm"
		if k >= 2
			for w in W
				avoid=false
					for j in 1:k-1
						if norm(subop_sol_hist[j][w].lambda.-subop_sol_hist[k][w].lambda)/norm(subop_sol_hist[j][w].lambda) <= 0.2 && norm(subop_sol_hist[j][w].op_cost.-subop_sol_hist[k][w].op_cost)/norm(subop_sol_hist[j][w].op_cost) <= 0.2
							println("Repeated cut for week $(w) iteration $(k)!")
							avoid=true
							break
						end
					end
					if avoid==false
						@constraint(m,[w in W],subop_sol[w].theta_coeff*m[:vTHETA][w] >= subop_sol[w].op_cost + sum(subop_sol[w].lambda[i]*(variable_by_name(m,linking_variables_sub[w][i]) - planning_sol.values[linking_variables_sub[w][i]]) for i in 1:length(linking_variables_sub[w])));
					end   
			end
		else 
			@constraint(m,[w in W],subop_sol[w].theta_coeff*m[:vTHETA][w] >= subop_sol[w].op_cost + sum(subop_sol[w].lambda[i]*(variable_by_name(m,linking_variables_sub[w][i]) - planning_sol.values[linking_variables_sub[w][i]]) for i in 1:length(linking_variables_sub[w])));
		end
	else
		@constraint(m,[w in W],subop_sol[w].theta_coeff*m[:vTHETA][w] >= subop_sol[w].op_cost + sum(subop_sol[w].lambda[i]*(variable_by_name(m,linking_variables_sub[w][i]) - planning_sol.values[linking_variables_sub[w][i]]) for i in 1:length(linking_variables_sub[w])));
	end
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


function filter_planning_sol!(dict::Dict{String, Float64})

    for (key, value) in dict
        if abs(value) <= exp(-6)
            dict[key] = 0.0
        end
    end

    return dict

end


function filter_cuts!(data::Dict{Any, Any})

    for (key, value) in data
        if haskey(value, :lambda)
            value.lambda .= [abs(x) < exp(-6) ? 0.0 : x for x in value.lambda]
        end
    end

    return data
end