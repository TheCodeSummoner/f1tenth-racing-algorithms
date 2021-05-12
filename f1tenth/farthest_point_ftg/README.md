## Farthest Point FTG

*Taken from the thesis write-up*:

Farthest Point FTG works by selecting the farthest point within the largest gap in the current LiDAR scan area.
Additionally, a safety bubble is defined around the point closest to the vehicle, to avoid driving near the obstacles.
More precisely, the closest LiDAR point within the scan range is first selected, and the scan ranges of all points
within some radius `r` away from it (including the point itself) are marked with some ignore distance
value (typically `0`).
Then, a longest sequence of non-ignored scan ranges is found, and the farthest (largest
scan range value) point within such sequence will be the trajectory reference point.

![Farthest Point FTG graphic](../../assets/images/farthest-point-ftg.png?raw=true "Farthest Point FTG")

Naturally, there exist more advanced versions of this algorithm, which for example further break down the sequences of
non-ignored scan ranges by
finding all "jumps" in consecutive LiDAR readings.
A jump is defined as a difference between two consecutive ranges, such that it is higher
than some threshold `N`.

The pseudocode for this algorithm is:

```
while True:
    points := process_lidar(lidar_scan)
    closest_point := min(points, key=lambda point: point.range)
    mark_safety_radius(closest_point, points)
    sequence := find_longest_sequence_of_safe_points(points)
    target_point := max(sequence, key=lambda point: point.range)
endwhile
```