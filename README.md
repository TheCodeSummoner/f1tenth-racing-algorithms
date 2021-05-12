# F1TENTH racing algorithms

A collection of F1/10 racing algorithms developed as part of my Bachelor's thesis.

This library provides an implementation of three control methods utilising Follow The Gap (FTG) and Model Predictive
Control (MPC), as well as a reference FTG algorithm and a motion planner capable of following a pre-defined trajectory.
Every sub-package within the `f1tenth` folder can be inspected for further details.

## Getting started

These instructions will help you understand how to get the project up and running for development and testing purposes.
Note that the environment is containerised, and as such you can run it on any operating system supporting `docker`.

### Prerequisites

I recommend using `docker`.

Otherwise, if you want to install your package locally, you need to:

- Install ROS Melodic
- Install F1TENTH simulator
- Install this package
- Move the assets folder to where the package got installed
- Adjust shell scripts and/or system configuration to make the library work with your local setup

Please refer to the `Dockerfile` definition for an example system configuration.

### Installation

You should create a new `docker` container. Installing the package using `pip` is not supported, but a basic
`setup` file is included for type-hinting purposes.

#### pip

When using `pip`, first you should install the package as follows:

```commandline
pip install git+https://github.com/TheCodeSummoner/f1tenth-racing-algorithms.git
```

Then, after moving the `assets` folder to where that package is installed, you can verify the setup by running:

```commandline
python -c "import f1tenth; print(f'{f1tenth.__title__} v{f1tenth.__version__}')"
```

It should return the package name and version, e.g.:

```commandline
f1tenth v0.0.0
```

### Quick run

To verify the entire setup is correct, consider running some of the shell scripts provided with the `assets/scripts`
folder. For example, to run the simulator on `Vegas` racing map, execute `launch-vegas.sh`. Note that shell scripts
are meant to work with the provided `Dockerfile`, and running them locally requires slight changes.

#### docker

See the `assets/scripts` folder, where two shell scripts can be seen for building and running the container.
More precisely, `docker-build.sh` explains how to build the image, and `docker-run` explains how to create a
container. Subsequent usage can be achieved with `docker container start`.

`docker-build.sh`
```commandline
docker build -t f1tenth-racing-algorithms .
```

`docker-run.sh`
```
docker run -p 6080:80 -v /dev/shm:/dev/shm f1tenth-racing-algorithms
```

The associated `Dockerfile` is included at the root level of the library.

## Usage

There are two usage scenarios:

- Run available code
- Run your own code

Essentially, running code shared with this library is as simple as running the associated shell scripts in the `docker`
container. To run your own code, you must make sure all simulator-related commands are available in your shell session.

### Setup worskpace

You can source all simulator commands by running:

```commandline
source /root/workspace/setup-workspace.sh
```

Refer to the `Dockerfile` definition to understand what this script does.

### Run existing code

You can either run the existing shell scripts (in which case you don't even have to setup the workspace) or run the
files directly. Since running a shell script is rather trivial, I will only provide an example of running things with
Python. Consider a use-case scenario when I have modified some of the existing code, and want to run Middle Point FTG:

```commandline
python3 f1tenth/middle_point_ftg
```

The reason this works is due to the `__main__.py` file being present in that package. Alternatively, you could execute
the main file directly (`python3 f1tenth/middle_point_ftg/__main__.py`).

### Run new code

Running new code is similar to running the existing code manually. First, setup your workspace, and then execute your
code. I recommend creating packages in a similar structure, where each folder contains both `__init__.py` and
`__main__.py`. It simplifies rapidly testing new algorithms.

## Contributing
Pull requests are welcome. For modifications of existing code, please open an issue first to discuss what you would like
to change. Otherwise, since there are no tests, you can contribute anything (as long as it's thematically related).
However, please make sure to provide a `README.md` file explaining your setup.

## License
[MIT](https://choosealicense.com/licenses/mit/)