# MacroEnergySolvers.jl

This repository is designed to collect solution algorithms for macro-energy system optimization models. In the future, we hope to support several algorithms that can be applied to solve these large-scale mixed integer linear programs. 

Currently, the repositoy includes the regularized Benders decomposition algorithm described in:

_F. Pecci and J. D. Jenkins (2025), "Regularized Benders Decomposition for High Performance Capacity Expansion Models," in IEEE Transactions on Power Systems, doi: [10.1109/TPWRS.2025.3526413](https://ieeexplore.ieee.org/document/10829583)_.

Other algorithms are possible and could be added in the future (e.g., Dual Dynamic Programming). If you would like to develop a decomposition method for macro-energy system models, we welcome any contributions.

In the case of Benders decomposition, the user must generate the inputs for function `benders(planning_problem,linking_variables,subproblems,linking_variables_sub,setup)` where:

1. `planning_problem` is a JuMP model describing a Benders planning problem whose objective function is given by: `planning_problem[:eFixedCost] + planning_problem[:eApproximateVariableCost]`, where:
   
   -  `planning_problem[:eFixedCost]` is a JuMP expression computing fixed costs
   -  `planning_problem[:eApproximateVariableCost]` is a JuMP expression computing the approximation of variable cost based on the auxiliary variables in vector `planning_problem[:vTHETA]`
2. `subproblems` can be either a vector or a distributed array of `Dict`. Each dict has keys:
    - `:model =>` JuMP model of the subproblem (`Model`)
    - `:linking_variables_sub =>` linking variables that apply to the subproblem (`Vector{String}`)
    - `:subproblem_index =>` index of the subproblem as it appears in the array subproblems (`Int64`)
    - `:slack_penalty_value` => penalty to be added to the objective functions when slacks are used to guarantee that the subproblems are feasible. When this is `nothing`, the algorithm will generate feasibility-cuts if a subproblem is infeasible (`Float64` or `nothing`).
3. `linking_variables` is a `Vector{String}` with string names of the all linking variables (those coupling planning problem and subproblems)
4. `linking_variables_sub` is a `Dict` mapping each subproblem index to the corresponding `Vector{String}` of linking variables.
5. `setup` is a `Dict` containing the settings of the Benders run, whose defaults are:
    ```
        Dict(
        :MaxIter=> 50,
        :MaxCpuTime => 7200,
        :ConvTol => 1e-3,
        :StabParam => 0.0,
        :StabDynamic => false,
        :IntegerInvestment => false,
        :Distributed => false,
        :IncludeAutomaticSlackPenalty => false
    )
    ```
