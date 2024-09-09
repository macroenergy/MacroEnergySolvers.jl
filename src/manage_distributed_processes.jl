

function start_distributed_processes!(number_of_subperiods::Int64,model_type::Symbol)

    if haskey(ENV,"SLURM_NTASKS")
        ntasks = min(number_of_subperiods,parse(Int, ENV["SLURM_NTASKS"]));
        cpus_per_task = parse(Int, ENV["SLURM_CPUS_PER_TASK"]);
        addprocs(ClusterManagers.SlurmManager(ntasks);exeflags=["-t $cpus_per_task"])
    else
        ntasks = min(number_of_subperiods,Sys.CPU_THREADS)
        cpus_per_task = 1;
        addprocs(ntasks)
    end

    project = Pkg.project().path

    @sync for p in workers()
        @async create_worker_process(p,project,model_type)
    end

    println("Number of subperiods:",number_of_subperiods)
    println("Number of procs: ", nprocs())
    println("Number of workers: ", nworkers())
end


function create_worker_process(pid,project,model_type)

    Distributed.remotecall_eval(Main, pid,:(using Pkg))

    Distributed.remotecall_eval(Main, pid,:(Pkg.activate($(project))))
                                 
    if model_type == :MACRO
        Distributed.remotecall_eval(Main, pid, :(using Macro))
    elseif model_type == :GenX
        Distributed.remotecall_eval(Main, pid, :(using GenX))
    end

    Distributed.remotecall_eval(Main, pid, :(using MacroEnergySystemsDecomposition))

end
