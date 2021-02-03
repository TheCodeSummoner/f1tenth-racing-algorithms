## Follow the Gap

Follow-the-gap racing code transferred from [newcastle_race][1].

This is a known working solution, used as a reference for developing MPC-based algorithms.

### Prerequisites

F1TENTH simulator master node must be running.

The algorithm has been tuned to run on the `vegas` racing map. You can re-tune the algorithm parameters by adjusting
the values in `constants.py`.

### Usage

This sub-package follows the standard usage instructions of the parent package.

#### Run from python console

```python
from f1tenth import follow_the_gap

follow_the_gap.run()
```

#### Run main directly

```
python <path>/f1tenth/follow_the_gap
```

or

```
python <path>/f1tenth/follow_the_gap/__main__.py
```

#### Run installed files

```
python -m f1tenth.follow_the_gap
```

### Notes

Since the transferal of the code was meant to provide a good reference algorithm for future development, most of the
constants have not been interpreted and transferred to the relevant `constants` module.

It is therefore the least documented method.

[1]: https://github.com/pastankaitis/newcastle_race
[2]: https://f1tenth.org/build.html
