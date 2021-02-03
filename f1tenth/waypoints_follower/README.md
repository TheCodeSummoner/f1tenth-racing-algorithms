## Waypoints Follower

Reference trajectory following racer utilising MPC to compute optimal paths.

As the vehicle progresses on the track, the next waypoint is picked as the target position, and the vehicle starts
steering towards it. Additionally, the vehicle tries to use as high of a velocity as possible.

### Prerequisites

F1TENTH simulator master node must be running.

The algorithm has been tuned to run on the `skirk` racing map. You can re-tune the algorithm parameters by adjusting
the values in `constants.py`.

### Usage

This sub-package follows the standard usage instructions of the parent package.

#### Run from python console

```python
from f1tenth import waypoints_follower

waypoints_follower.run()
```

#### Run main directly

```
python <path>/f1tenth/waypoints_follower
```

or

```
python <path>/f1tenth/waypoints_follower/__main__.py
```

#### Run installed files

```
python -m f1tenth.waypoints_follower
```

[1]: https://f1tenth.org/build.html
