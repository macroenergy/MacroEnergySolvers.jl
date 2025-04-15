var documenterSearchIndex = {"docs":
[{"location":"","page":"Getting Started","title":"Getting Started","text":"CurrentModule = MacroEnergySolvers","category":"page"},{"location":"#MacroEnergySolvers","page":"Getting Started","title":"MacroEnergySolvers","text":"","category":"section"},{"location":"","page":"Getting Started","title":"Getting Started","text":"Documentation for MacroEnergySolvers.","category":"page"},{"location":"","page":"Getting Started","title":"Getting Started","text":"This repository is designed to collect solution algorithms for macro-energy system optimization models. In the future, we hope to support several algorithms that can be applied to solve these large-scale mixed integer linear programs. ","category":"page"},{"location":"","page":"Getting Started","title":"Getting Started","text":"tip: Peer-reviewed work\nCurrently, the repository includes the regularized Benders decomposition algorithm described in:F. Pecci and J. D. Jenkins (2025), \"Regularized Benders Decomposition for High Performance Capacity Expansion Models,\" in IEEE Transactions on Power Systems, doi: 10.1109/TPWRS.2025.3526413.","category":"page"},{"location":"","page":"Getting Started","title":"Getting Started","text":"note: Future plans\nOther algorithms are possible and could be added in the future (e.g., Nested Benders Decomposition). If you would like to develop a decomposition method for macro-energy system models, please open a pull request or get in touch with us, we welcome any contributions.","category":"page"},{"location":"#Installation","page":"Getting Started","title":"Installation","text":"","category":"section"},{"location":"","page":"Getting Started","title":"Getting Started","text":"using Pkg\nPkg.add(\"MacroEnergySolvers\")","category":"page"},{"location":"#Usage","page":"Getting Started","title":"Usage","text":"","category":"section"},{"location":"","page":"Getting Started","title":"Getting Started","text":"# Import the package\nusing MacroEnergySolvers\n\n# Create configuration\nsetup = Dict(\n    :MaxIter=> 50,\n    :MaxCpuTime => 7200,\n    :ConvTol => 1e-3,\n    :StabParam => 0.0,\n    :StabDynamic => false,\n    :IntegerInvestment => false,\n    :Distributed => false,\n    :IncludeAutomaticSlackPenalty => false\n)\n\n# Create a decomposed model (see below for details)\n# [... add code here ... ]\n\n# Run the algorithm\nresults = benders(planning_problem, linking_variables, subproblems, linking_variables_sub, setup)","category":"page"},{"location":"","page":"Getting Started","title":"Getting Started","text":"In the case of Benders decomposition, the user must generate the inputs for function benders(planning_problem,linking_variables,subproblems,linking_variables_sub,setup) where:","category":"page"},{"location":"","page":"Getting Started","title":"Getting Started","text":"planning_problem is a JuMP model describing a Benders planning problem whose objective function is given by: planning_problem[:eFixedCost] + planning_problem[:eApproximateVariableCost], where:\nplanning_problem[:eFixedCost] is a JuMP expression computing fixed costs\nplanning_problem[:eApproximateVariableCost] is a JuMP expression computing the approximation of variable cost based on the auxiliary variables in vector planning_problem[:vTHETA]\nsubproblems can be either a vector or a distributed array of Dict. Each dict has keys:\n:model => JuMP model of the subproblem (Model)\n:linking_variables_sub => linking variables that belong to the subproblem (Vector{String})\n:subproblem_index => index of the subproblem as it appears in the array of subproblems (Int64)\n:slack_penalty_value => penalty to be added to the objective functions when slacks are used to guarantee that the subproblems are feasible. When this is nothing, the algorithm will generate feasibility-cuts if a subproblem is infeasible (Float64 or nothing).\nlinking_variables is a Vector{String} with string names of the all linking variables (those coupling planning problem and subproblems)\nlinking_variables_sub is a Dict mapping each subproblem index to the corresponding Vector{String} of linking variables.\nsetup is a Dict containing the settings of the Benders run, whose defaults are:  Dict(      :MaxIter=> 50,      :MaxCpuTime => 7200,      :ConvTol => 1e-3,      :StabParam => 0.0,      :StabDynamic => false,      :IntegerInvestment => false,      :Distributed => false,      :IncludeAutomaticSlackPenalty => false  )","category":"page"},{"location":"#API-Reference","page":"Getting Started","title":"API Reference","text":"","category":"section"},{"location":"","page":"Getting Started","title":"Getting Started","text":"","category":"page"},{"location":"#benders","page":"Getting Started","title":"benders","text":"","category":"section"},{"location":"","page":"Getting Started","title":"Getting Started","text":"benders","category":"page"},{"location":"#MacroEnergySolvers.benders","page":"Getting Started","title":"MacroEnergySolvers.benders","text":"benders(planning_problem::Model, \n\tlinking_variables::Vector{String}, \n\tsubproblems::Union{Vector{Dict{Any, Any}}, DistributedArrays.DArray}, \n\tlinking_variables_sub::Dict, \n\tsetup::Dict\n)\n\nImplements a regularized Benders decomposition algorithm for solving large-scale energy systems planning problems.\n\nAlgorithm from:\n\nF. Pecci and J. D. Jenkins (2025). “Regularized Benders Decomposition for High Performance Capacity Expansion Models”. doi: 10.1109/TPWRS.2025.3526413\n\nIt's a regularized version of the Benders decomposition algorithm in:\n\nA. Jacobson, F. Pecci, N. Sepulveda, Q. Xu, and J. Jenkins (2024). “A computationally efficient Benders decomposition for energy systems planning problems with detailed operations and time-coupling constraints.” doi: https://doi.org/10.1287/ijoo.2023.0005\n\nArguments\n\nplanning_problem::Model: The master problem JuMP model representing the investment decisions\nlinking_variables::Vector{String}: Names of the variables linking the master and subproblems\nsubproblems::Union{Vector{Dict{Any, Any}},DistributedArrays.DArray}: Collection of operational subproblems\nlinking_variables_sub::Dict: Mapping between subproblems and their associated linking variables\nsetup::Dict: Algorithm parameters including:\n\n- `MaxIter`: Maximum number of iterations\n- `ConvTol`: Convergence tolerance\n- `MaxCpuTime`: Maximum CPU time allowed\n- `StabParam`: Stabilization parameter γ\n- `StabDynamic`: Boolean for dynamic stabilization adjustment\n- `IntegerInvestment`: Boolean for integer investment variables\n\nReturns\n\n@NamedTuple containing:\n\nplanning_problem: Updated master problem model\nplanning_sol: Best solution found\nLB_hist: History of lower bounds\nUB_hist: History of upper bounds\ncpu_time: CPU time history\nsol_hist: Solution history for linking variables\n\n\n\n\n\n","category":"function"},{"location":"#solve_planning_problem","page":"Getting Started","title":"solve_planning_problem","text":"","category":"section"},{"location":"","page":"Getting Started","title":"Getting Started","text":"solve_planning_problem","category":"page"},{"location":"#MacroEnergySolvers.solve_planning_problem","page":"Getting Started","title":"MacroEnergySolvers.solve_planning_problem","text":"solve_planning_problem(m::Model, linking_variables::Vector{String})\n\nSolves the planning (master) problem in the Benders decomposition algorithm.\n\nThis function attempts to solve the planning problem and handles potential numerical issues, particularly with negative capacities, by re-solving with different solver settings if needed.\n\nArguments\n\nm::Model: The JuMP model representing the planning problem\nlinking_variables::Vector{String}: Names of the variables linking the master and subproblems\n\nReturns\n\nA NamedTuple containing:\n\nLB: Lower bound (objective value) of the planning problem\nfixed_cost: Fixed cost component of the solution\nvalues: Dictionary mapping linking variable names to their optimal values\ntheta: Values of the theta variables representing subproblem costs\n\nNotes\n\nIf negative capacities are detected, the solver will be reconfigured with Crossover = 1  and the problem will be re-solved. If the solution fails, the function will compute and display conflicting constraints (if the solver supports it) before throwing an error.\n\n\n\n\n\n","category":"function"},{"location":"#solve_subproblems","page":"Getting Started","title":"solve_subproblems","text":"","category":"section"},{"location":"","page":"Getting Started","title":"Getting Started","text":"solve_subproblems","category":"page"},{"location":"#MacroEnergySolvers.solve_subproblems","page":"Getting Started","title":"MacroEnergySolvers.solve_subproblems","text":"solve_subproblems(\n    m_subproblems::DArray{Dict{Any, Any}, 1, Vector{Dict{Any, Any}}}, \n    planning_sol::NamedTuple\n)\n\nSolves subproblems in parallel using distributed computing capabilities.\n\nThis function coordinates the parallel solution of operational subproblems across multiple workers, using Julia's distributed computing framework. Each worker processes its local portion of the distributed array of subproblems.\n\nArguments\n\nm_subproblems::DArray: Distributed array containing the subproblems, where each element is a  dictionary representing a subproblem\nplanning_sol::NamedTuple: Current solution of the planning problem containing variable values  needed for the subproblem solutions\n\nReturns\n\nA merged dictionary containing results from all subproblems, where each entry contains:\n\nOptimal objective value\nDual variables\nOther solution information from each subproblem\n\nImplementation Details\n\nUses @sync and @async for coordinated parallel execution, with results fetched from each worker and merged into a single dictionary containing all subproblem solutions.\n\n\n\n\n\n","category":"function"},{"location":"#solve_int_level_set_problem","page":"Getting Started","title":"solve_int_level_set_problem","text":"","category":"section"},{"location":"","page":"Getting Started","title":"Getting Started","text":"solve_int_level_set_problem","category":"page"},{"location":"#MacroEnergySolvers.solve_int_level_set_problem","page":"Getting Started","title":"MacroEnergySolvers.solve_int_level_set_problem","text":"solve_int_level_set_problem(m::Model, \n\tlinking_variables::Vector{String}, \n\tplanning_sol::NamedTuple, \n\tLB, \n\tUB, \n\tγ\n)\n\nSolves the interior level set stabilization problem for the regularized Benders decomposition algorithm.\n\nThis stabilization technique helps improve convergence by restricting the master problem solution to lie within a level set defined by the current lower and upper bounds, controlled by the stabilization parameter γ.\n\nArguments\n\nm::Model: The JuMP model representing the planning problem\nlinking_variables::Vector{String}: Names of the variables linking the master and subproblems\nplanning_sol::NamedTuple: Current solution of the planning problem\nLB: Current lower bound\nUB: Current upper bound\nγ: Stabilization parameter controlling the size of the level set (0 ≤ γ ≤ 1)\n\nReturns\n\nA NamedTuple containing the solution of the stabilized problem with the same structure as the input planning_sol\n\n\n\n\n\n","category":"function"}]
}
