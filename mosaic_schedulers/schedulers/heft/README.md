# HEFT scheduler

A very lightly modified version of the HEFT scheduler presented in [1].

## Usage

```python
Scheduler = HEFT.Scheduler(
    JSONProblemDescription,  # A description of the problem in JSON format. See below for a detailed specification.,
)
Schedule = Scheduler.schedule()
```

## Input-Output format

See the [README of the time-varying MILP scheduler](../tv_milp/README.md) for a detailed description of the I/O format.

## References

[1] H. Topcuoglu, S. Hariri and Min-You Wu, ["Performance-effective and low-complexity task scheduling for heterogeneous computing,"](https://ieeexplore.ieee.org/document/993206) in IEEE Transactions on Parallel and Distributed Systems, vol. 13, no. 3, pp. 260-274, March 2002.
doi: [10.1109/71.993206](https://dx.doi.org/10.1109/71.993206)