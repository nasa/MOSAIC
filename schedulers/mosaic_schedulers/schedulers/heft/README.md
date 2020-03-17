# HEFT scheduler

A very lightly modified version of the HEFT scheduler presented in [1].

The key modifications with respect to the HEFT scheduler are:

- Computation tasks take time and tie up agent resources. They are scheduled
  according to the insertion-based scheduler; longer communications are 
  scheduled first.

- The agent providing the earliest _finish_ time (as opposed to the earliest
  start time) is assigned to a task.

## Usage

```python
Scheduler = HEFT.Scheduler(
    JSONProblemDescription,  # A description of the problem in JSON format. See below for a detailed specification.,
)
Schedule = Scheduler.schedule()
```

See the [schedulers README](../../../README.md) for a detailed description of the I/O format.

## References

[1] H. Topcuoglu, S. Hariri and Min-You Wu, ["Performance-effective and low-complexity task scheduling for heterogeneous computing,"](https://ieeexplore.ieee.org/document/993206) in IEEE Transactions on Parallel and Distributed Systems, vol. 13, no. 3, pp. 260-274, March 2002.
doi: [10.1109/71.993206](https://dx.doi.org/10.1109/71.993206)