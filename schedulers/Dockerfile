# To build, run 'docker build --tag=mosaic/schedulers -f Dockerfile .'

# Use an official ROS build as a parent image
FROM mosaic/schedulers_requirements

# Path for SCIP
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Set the working directory to /home
WORKDIR /home

# Copy the current directory contents into the container at /app
RUN mkdir -p /home/schedulers/
COPY . /home/schedulers/

# Install any needed packages
RUN /home/schedulers/docker_resources/install_scripts/05_install_schedulers.sh

# Run a web server exposing the MOSAIC JSON interface on startup
CMD ["/bin/bash", "-c", "gunicorn -w 10 -b 127.0.0.1:4000 mosaic_schedulers.common.utilities.MOSAIC_server:app"]
