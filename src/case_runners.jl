function solve_model_with_benders(case_path::String,model::Module)

    system = model.load_system(case_path);

    planning_problem,linking_variables = initialize_planning_problem!(system,model);

    system_decomp = model.generate_decomposed_system(system);

    number_of_subperiods = length(system_decomp);

    start_distributed_processes!(number_of_subperiods,model)

    subproblems_dict,linking_variables_sub = initialize_dist_subproblems(system_decomp,model)

    result = benders(planning_problem,linking_variables,subproblems_dict,linking_variables_sub)
    
    return result
end

function solve_model_monolithic(case_path::String,model_type::Symbol)

    if model_type==:MACRO
        system = Macro.load_system(case_path);

        model = Macro.generate_model(system)
        
        set_optimizer(model,Gurobi.Optimizer);
        
        optimize!(model)

        macro_objval = Macro.objective_value(model)
        
    end

    
    return nothing
end