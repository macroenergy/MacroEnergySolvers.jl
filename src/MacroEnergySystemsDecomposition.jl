module MacroEnergySystemsDecomposition

    export start_distributed_processes!

    using JuMP # used for mathematical programming
    using Distributed
    using DistributedArrays
    using ClusterManagers
    using Gurobi
    using Pkg
    
    const GRB_ENV = Ref{Gurobi.Env}()
    function __init__()
        GRB_ENV[] = Gurobi.Env()
        return
    end
    
    
    include("manage_distributed_processes.jl")


end