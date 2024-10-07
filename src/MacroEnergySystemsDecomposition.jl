module MacroEnergySystemsDecomposition

    using JuMP # used for mathematical programming
    using Distributed
    using DistributedArrays
    using ClusterManagers
    using Gurobi
    using LinearAlgebra
    using Pkg
    using Revise

    const GRB_ENV = Ref{Gurobi.Env}()
    function __init__()
        GRB_ENV[] = Gurobi.Env()
        return
    end


    include("manage_distributed_processes.jl")
    include("case_runners.jl")
    include("benders/planning_problem.jl")
    include("benders/distributed_subproblems.jl")
    include("benders/regularization.jl")
    include("benders/algorithms.jl")

    export start_distributed_processes!
    export solve_model_with_benders
    export solve_model_monolithic

end