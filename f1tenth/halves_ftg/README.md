## Halves FTG

*Taken from the thesis write-up*:

Halves FTG is a custom, proposed method for selecting the target reference point.

Let `L` denote LiDAR scan ranges to the left-hand side of the vehicle, and `R` denote LiDAR scan
ranges to the right-hand side of the vehicle.
First, to refine the final results, all scan ranges are passed through a filter, such that any values above
some threshold `N` will be ignored.
This is to ensure the results are not (nearly) the same as if the farthest point FTG algorithm was run.
Then, the Farthest Point FTG is run for `L` and `R` to find a pair of target reference
points.
Finally, these target points are combined (by averaging the coordinates) to form a single target reference
point, which is then selected as the final target reference point, used in point-following MPC.
In a case when either of the halves fails to produce a target point (for example because all points are too far
away from the vehicle), a default position for the target point is selected based on the tuning parameters.

![Halves FTG graphic](../../assets/images/halves-ftg.png?raw=true "Halves FTG")

The pseudocode for this algorithm is:

```
while True:
    points := filter_by_max_distance(process_lidar(lidar_scan))
    left_points, right_points := split_into_halves(points)
    left_target_point := farthest_point_ftg(left_points)
    right_target_point := farthest_point_ftg(right_points)
    sum_of_x_coordinates := left_target_point.x + right_target_point.x
    sum_of_y_coordinates := left_target_point.y + right_target_point.y
    target_point := (sum_of_x_coordinates / 2, sum_of_y_coordinates / 2)
endwhile
```