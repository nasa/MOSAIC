# MOSAIC centralized time-invariant task allocator

The time-invariant task allocation algorithm presented in Rossi, Vaquero et al's paper under review.

```python
Scheduler = MOSAICTISolver.JSONSolver(
    JSONProblemDescription,  # A description of the problem in JSON format. See below for a detailed specification.
    solver,  # The solver to be used. Possible values: CPLEX, PuLP, SCIP, GLPK.
    Verbose,  # Whether the solver should print information.
    TimeLimit,  # A time limit for the MIP scheduler.
)
Schedule = Scheduler.schedule()
```

## References

[1] Federico Rossi\*, Tiago Stegun Vaquero\*, Marc Sanchez Net, Maíra Saboia da Silva, and Joshua Vander Hook, "The Pluggable Distributed Resource Allocator (PDRA): a Middleware for Distributed Computing in Mobile Robotic Networks," under review.