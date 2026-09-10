### JuMP's has_values() reports only that the solver returned a primal point, not that the model
### was solved to feasibility: after an infeasible solve the point handed back is an
### INFEASIBLE_POINT, which is not NO_SOLUTION, so has_values() stays true. Once a model has been
### solved successfully even once, every later infeasible solve on it still looks solved. The
### helpers below check the solution statuses directly instead.

"""
    solved_and_feasible(m::Model)

Return `true` when the last solve of `m` produced a feasible primal point.

Unlike `has_values`, this is `false` after an infeasible solve, including an infeasible solve that
follows a successful one on the same model.
"""
function solved_and_feasible(m::Model)
    return primal_status(m) == FEASIBLE_POINT
end

"""
    solved_with_duals(m::Model)

Return `true` when the last solve of `m` produced a feasible primal point and a feasible dual
point.

Use this wherever duals are read. Note that it is always `false` for a model whose integrality
constraints are still switched on, since duals are not available for a mixed-integer problem.
"""
function solved_with_duals(m::Model)
    return solved_and_feasible(m) && dual_status(m) == FEASIBLE_POINT
end
