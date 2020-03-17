# Time-varying MILP MOSAIC scheduler

The scheduler presented in Hook et al's ICAPS 2019 paper [1].

## Usage

```python
Scheduler = MOSAICSolver.JSONSolver(
    JSONProblemDescription,  # A description of the problem in JSON format. See below for a detailed specification.
    solver,  # The solver to be used. Possible values: CPLEX, PuLP, SCIP, GLPK.
    Verbose,  # Whether the solver should print information.
    TimeLimit,  # A time limit for the MIP scheduler.
)
Schedule = Scheduler.schedule()
```

See the [schedulers README](../../../README.md) for a detailed description of the I/O format.

## References

[1] Joshua Vander Hook, Tiago Vaquero, Federico Rossi, Martina Troesch, Marc Sanchez Net, Joshua Schoolcraft, Jean-Pierre de la Croix, and Steve Chien, ["Mars On-Site Shared Analytics Information and Computing,"](https://aaai.org/ojs/index.php/ICAPS/article/view/3556) in Proceedings of the Twenty-Ninth International Conference on Automated Planning and Scheduling, vol. 29, no. 1, pp. 707-715, July 2019.
