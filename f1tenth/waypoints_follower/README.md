## Waypoints Follower

*Taken from the thesis write-up*:

Following a pre-defined trajectory consisting of a collection of cartesian coordinates is an example use-case scenario
demonstrating the distance-based point following.
In order to utilise MPC, a custom way to continuously choose
consecutive points in the reference trajectory, as the vehicle progresses, has been designed and implemented.

![Waypoints Follower screenshot](../../assets/images/waypoints-follower-screenshot.png?raw=true "Waypoints Follower")

Preferring simplistic solutions, the Euclidean distance between the vehicle and the
current target reference point has been used.
Upon reaching some threshold `N`, the next waypoint in the reference trajectory is chosen
(the list of waypoints is sorted).

The pseudocode for this algorithm is:

```
waypoints := read_from_file(waypoints.csv)
target_waypoint := waypoints.first()
while True:
    distance := compute_distance_to_waypoint(target_waypoint)
    if (distance < THRESHOLD) then
        target_waypoint := get_next_waypoint()
    endif
endwhile
```

Naturally, this solution depends entirely on the value of the threshold, as it can be parametrised differently for
different race tracks.
A more verbose solution should for example consider distances to intermediate points within the trajectory.