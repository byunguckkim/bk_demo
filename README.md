# Overview

This repository contains a C++ multirate ADP bridge implementation, along with supporting sources from Applied Intuition and third parties.

The typical mode of operation for the ADP product line is that you will have your customer interface running in its own Docker container.
This has several advantages:
* The Docker container ensures that the environment for the customer stack is independent from the environment of your host machine, eliminating several failure modes.
* This also means that executing the customer stack on different machines will have the same results, since Docker will ensure a controlled environment.
* ADP can access the customer container and bring up and shut down the customer stack server between each simulation.
  This ensures that different simulation runs will not influence each other.

# Building and running inside the Docker container

## Build the Interface
These steps should be run from the root-level of this repository:

1. Run `docker/remove.sh` to remove any running docker container.
2. Run `docker/build.sh` to create and build the docker image.
3. Run `./build_and_update.sh` to start the container and build the interface code.

NOTE: Building the docker image and the interface code for the first time will take >10 minutes.
From there on, successive builds are incremental and therefore much quicker (~30 seconds)!

After following these 3 steps, you should see the docker container actively running when you run `docker ps -a`.
Now you should be able to run simulations to engage the interface.

## Update the Interface and re-build
The primary interface code is within `adp_bridge.h` and `adp_bridge.cc`.
Any time you make changes to this code or its dependencies, you must compile the interface in the docker container.

To do so, run `./build_and_update.sh` from the root level of this repository:
* Will start the container if no container is up and running.
* Will invoke a sequence of build commands to re-build the interface code.

# Update the ADP protos and libraries every release
1. Download the latest release zip, unzip it and verify that you see `adp_interface/public_API_release_unified_bridge_<ADP_version>.zip`.
2. Run `./perform_ADP_release_update.sh --api_release_zip_path=/path/to/api/zip --applied_path=/path/to/existing/applied/folder/`

On rare occasions, you may also need to overwrite files the implement the interface's C ABI layer and base classes to use a new interface feature.
When required for specific releases, you can instead run `./perform_ADP_release_update.sh --api_release_zip_path=/path/to/api/zip --applied_path=/path/to/existing/applied/folder/ --replace_glue_stack_base` to update.

# Running a simulation

1. Make sure that the following simulation flags are specified before running a simulation: `--container_name <container name here> --v2api`. One way to verify and modify simulation flags is by navigating to "File > Configure Simulation Flags" when you open a scenario.

2. Click the "Run" button for a simulation.

3. You should see the simulation actively running and outputs being sent to the `stdout`. There should also be an example drawing that is outputted to the frontend.

# Building and running without a Docker container

This section is only relevant if you need to run the customer stack straight on your host machine instead of inside a Docker container.
It will be a more complicated mode of operation, so if you can, using the Docker container is highly recommended.

First, you need to ensure that all the necessary dependencies are installed.
If you are running a version of Ubuntu or a similar Linux distribution, the file `install_build_dependencies.sh` should set everything up as you need it.
If you are using a different Linux flavor, you might have to adjust this file to your needs.

To execute it, simply call:
```
sudo install_build_dependencies.sh -y
```

When that finishes successfully, you can build the customer interface like this:
```
mkdir build
cd build
cmake ..
make
```

This will generate the `customer_interface.so` file.
You can then manually start the customer stack server from the ADP installation on your machine and point it to the .so file:
```
/path/to/adp/simian_interface/simian/public/customer_stack_server_cc \
  --customer_interface_so /path/to/customer_interface.so --v2api
```

You should be getting a `Listening on [::]:10006` output to show that the customer stack server is up and running.

You can now run simulations against it using the `--no_customer_container` sim flag:
https://home.applied.co/manual/adp/v1.33/#/using_adp/usage_fundamentals/simulation_flags?id=-no_customer_container

Note that Simian will not automatically start and stop your customer stack server, so you need to ensure that everything is cleaned up properly between running different simulations.
Otherwise a previous sim run could affect the output of a later run.
