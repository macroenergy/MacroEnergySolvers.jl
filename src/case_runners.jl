function solve_model_with_benders(case_path::String,model_type::Symbol)

    if model_type==:MACRO
        system = Macro.load_system(case_path);

        system_decomp = Macro.generate_decomposed_system(system);
    end

    number_of_subperiods = length(system_decomp);

    planning_problem,linking_variables = initialize_planning_problem(system,model_type);

    start_distributed_processes!(number_of_subperiods,:MACRO)

    subproblems_dict,linking_variables_sub = initialize_dist_subproblems(system_decomp,model_type)

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