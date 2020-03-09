# MOSAIC Schedulers 

This repository contains MOSAIC schedulers and usage examples. The content of this repository can be installed with pip as the `mosaic_schedulers` Python package.

# Requirements

## Libraries

The project depends on the following libraries:

- [GLPK](https://www.gnu.org/software/glpk/);
- [SCIP](https://scip.zib.de) (optional);
- [CPLEX](https://www.ibm.com/products/ilog-cplex-optimization-studio) (optional);

GLPK and its dependencies can be installed by running `ubuntu_requirements.sh` (on Debian and derivatives, including Ubuntu) or `fedora_requirements.sh` (on Fedora and derivatives, including CentOS). This will also install some of the requirements to compile SCIP from source (see below).

SCIP offers academic licenses and provides [RPM and Deb binaries](https://scip.zib.de/index.php#download) for x86. On ARM, we recommend compiling from source (see below).
Place the SCIP source code in a folder (e.g., `scipoptsuite-6.0.1`), and then run:

```bash
cd scipoptsuite-6.0.1
mkdir build
cd build
cmake ..
make
make install
```

CPLEX is required by some of the schedulers. If CPLEX is not available, the solvers will fall back to GLPK.
An academic license for CPLEX can be obtained from [IBM](https://ibm.onthehub.com/WebStore/OfferingDetails.aspx?o=613c3d21-0ce1-e711-80fa-000d3af41938). Using the academic license is acceptable for non-commercial research: specifically, _"If the results are to be published in conference proceedings or technical journals, the use is acceptable."_ (per the [IBM Academic Initiative guidelines, retrieved on 2019-7-10](https://developer.ibm.com/academic/frequently-asked-questions/#faq1)).

# Installation

`pip install .`

# Docker

Dockerfiles are provided to package the MOSAIC schedulers in a Docker image. The Docker image exposes a web server that receives requests containing MOSAIC problems (in JSON format) and returns the computed schedules.

The Docker image is designed to use the SCIP scheduler and compile it from source. The SCIP source code should be placed in `docker_resources/scipoptsuite-6.0.1`. 

To build the MOSAIC docker image, run

```bash
docker build --tag=mosaic/schedulers_requirements -f Dockerfile_requirements .
docker build --tag=mosaic/schedulers -f Dockerfile .
```

When changes are made to the `mosaic_schedulers` package, only the second Docker image needs to be rebuilt.

To run the Docker image and expose the MOSAIC service on the local network,

```bash
docker run --network="host" mosaic/schedulers
```

The service exposes an instance of [MOSAIC_server.py](mosaic_schedulers/common/utilities/MOSAIC_server.py) at `localhost:4000`. The web interface accepts PUT requests containing a JSON description of the problem and returns computed schedules.


# Usage

## Schedulers

The MOSAIC schedulers can be found under `schedulers/` and can be imported with
`import mosaic_schedulers.schedulers.scheduler_name`.

## Examples

Usage examples and sample problems for the schedulers are contained in common.examples.

## Common tools

Several plotting tools are available. They can be imported as `import mosaic_schedulers.common.plotting`.
