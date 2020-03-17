# Utilities

Tools and utilities for the MOSAIC solvers.

## MOSAIC Server

The MOSAIC server `MOSAIC_server.py` exposes the MOSAIC solvers' JSON interface as a web service.
The following endpoints are provided:

- `/`: solves the time-invariant MOSAIC problem (with HEFT heuristic for time) with CPLEX. Uses a 30-second timeout.
- `/ti/`: solves the time-invariant MOSAIC _task allocation_ problem (without HEFT heuristic for time) with CPLEX. Uses a 30-second timeout. Tasks will not be assigned a time.
- `/ti_heft`: alias for `/`.
- `/tv/`: solves the time-varying MOSAIC problem with CPLEX. Uses a 60-second timeout.

If the file is run as-is, a Gunicorn server with 10 workers is started and bound to `127.0.0.1:4000`.

## Network Partitioning

The network partitioning tools `NetworkPartitioning.py` is a tool to partition the network of agents according to criteria including available bandwidth, maximum number of hops, and latency.
For a usage example, see the `__main__` function.

## Contact plan handler

The contact plan handlers in `contact_plan_handler` provide tools to convert contact plans (i.e., lists of pairwise contacts with start time, end time, and bandwidth) to a format understood by the MOSAIC schedulers.

## Dependency Builder

The dependency builder utilities in `dependency_builder.py` compute the software network graph from a problem in MOSAIC format and export it in D3-friendly format for plotting.

