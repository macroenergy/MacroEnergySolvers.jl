# MacroEnergySolvers.jl

This repository is designed to collect solution algorithms for macro-energy system optimization models. In the future, we hope to support several algorithms that can be applied to solve these large-scale mixed integer linear programs. 

Currently, the repository includes the regularized Benders decomposition algorithm described in:

_F. Pecci and J. D. Jenkins (2025), "Regularized Benders Decomposition for High Performance Capacity Expansion Models," in IEEE Transactions on Power Systems, doi: [10.1109/TPWRS.2025.3526413](https://ieeexplore.ieee.org/document/10829583)_.

Other algorithms are possible and could be added in the future (e.g., [Nested Benders Decomposition](https://www.sciencedirect.com/science/article/abs/pii/S0377221718304466)). If you would like to develop a decomposition method for macro-energy system models, please open a pull request or get in touch with us, we welcome any contributions.

In the case of Benders decomposition, the user must generate the inputs for function `benders(planning_problem,subproblems,linking_variables_sub,setup)` where:

1. `planning_problem` is a JuMP model describing a planning problem (without auxiliary variables to estimate operating costs). The planning problem must include:
    - A JuMP expression named `eLowerBoundOperatingCost` representing an initial global under-estimator of the operating cost in each subperiod (default is `0.0`).
2. `subproblems` can be either a vector or a distributed array of `Dict`. Each dict has keys:
    - `:model =>` JuMP model of the subproblem (`Model`)
    - `:linking_variables_sub =>` linking variables that belong to the subproblem (`Vector{String}`)
    - `:subproblem_index =>` index of the subproblem as it appears in the array of subproblems (`Int64`)
3. `linking_variables_sub` is a `Dict` mapping each subproblem index to the corresponding `Vector{String}` of linking variables.
4. `setup` is a `Dict` containing the settings of the Benders run, whose defaults are:
    ```
        Dict(
        :MaxIter=> 50,
        :MaxCpuTime => 7200,
        :ConvTol => 1e-3,
        :StabParam => 0.0,
        :StabDynamic => false,
        :IntegerInvestment => false,
        :Distributed => false
    )
    ```
