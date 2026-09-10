# Regression tests for the has_values() trap: JuMP reports a primal point after an infeasible
# solve (an INFEASIBLE_POINT, not NO_SOLUTION), so has_values() stays true once a model has been
# solved successfully once.

@testset "infeasible solve detection" begin

    @testset "an infeasible solve after a feasible one is not reported as solved" begin
        m = Model(HiGHS.Optimizer); set_silent(m)
        @variable(m, 0 <= x <= 10)
        @constraint(m, c, x >= 1)
        @objective(m, Min, x)

        optimize!(m)
        @test termination_status(m) == OPTIMAL
        @test MES.solved_and_feasible(m)
        @test MES.solved_with_duals(m)

        set_normalized_rhs(c, 20)   # x >= 20 with x <= 10 is infeasible
        optimize!(m)
        @test termination_status(m) == INFEASIBLE
        # has_values() is still true here - that is the bug being guarded against.
        @test has_values(m)
        @test !MES.solved_and_feasible(m)
        @test !MES.solved_with_duals(m)
    end

    @testset "a solved MIP is feasible but has no duals" begin
        # Why there are two helpers: requiring dual feasibility where only primal values are read
        # would report every successful integer solve as a failure.
        m = Model(HiGHS.Optimizer); set_silent(m)
        @variable(m, 0 <= x <= 10, Int)
        @constraint(m, x >= 1.5)
        @objective(m, Min, x)
        optimize!(m)
        @test termination_status(m) == OPTIMAL
        @test dual_status(m) == NO_SOLUTION
        @test MES.solved_and_feasible(m)
        @test !MES.solved_with_duals(m)
    end

    @testset "solve_subproblem still generates a feasibility cut after a feasible solve" begin
        # Serve demand 10 from a cheap generator capped by linking variable x and an expensive
        # generator capped at 3, so the subproblem is feasible iff x >= 7.
        m = Model(HiGHS.Optimizer); set_silent(m)
        @variable(m, x >= 0)
        @variable(m, g_cheap >= 0)
        @variable(m, 0 <= g_exp <= 3)
        @constraint(m, g_cheap <= x)
        @constraint(m, g_cheap + g_exp == 10)
        @objective(m, Min, 1*g_cheap + 5*g_exp)
        MES.add_slacks_to_subproblem!(m)

        feasible = MES.solve_subproblem(m, (values = Dict("x" => 8.0),), ["x"], false)
        @test feasible.theta_coeff == 1
        @test feasible.op_cost ≈ 18.0

        # Same model, now infeasible. Before the fix this took the optimality branch and returned
        # theta_coeff == 1 with duals read off an infeasible point.
        infeasible = MES.solve_subproblem(m, (values = Dict("x" => 0.0),), ["x"], false)
        @test infeasible.theta_coeff == 0
    end
end
