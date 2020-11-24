## Newcastle race

Test racing code based on [newcastle_race][1].

### Prerequisites

F1TENTH simulator master node must be running. You can follow the instructions [here][2] to get ROS, and start the node
by executing `roslaunch f1tenth_simulator simulator.launch`.

You can tune the algorithm parameters by adjusting the values in `constants.py`.

### Usage

This sub-package follows the standard usage instructions of the parent package.

#### Run from python console

```python
from f1tenth import newcastle_race

newcastle_race.run()
```

#### Run main directly

```
python <path>/f1tenth/newcastle_race
```

or

```
python <path>/f1tenth/newcastle_race/__main__.py
```

#### Run installed files

```
python -m f1tenth.newcastle_race
```

[1]: https://github.com/pastankaitis/newcastle_race
[2]: https://f1tenth.org/build.html
