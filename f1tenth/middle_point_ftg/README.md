## Middle Point FTG

*Taken from the thesis write-up*:

Middle Point FTG works by selecting the middle point within the largest gap in the current LiDAR scan area.
Similarly to the Farthest Point FTG, a safety bubble is defined to avoid driving into the obstacles.
However, in this case, the bubble is created around the vehicle, rather than around the point closest to it.
Just like in the referenced algorithm, all points within the safety radius will be marked with `0`.
Then, upon finding the longest sequence of non-ignored scan ranges, the middle point is selected.

![Middle Point FTG graphic](../../assets/images/middle-point-ftg.png?raw=true "Middle Point FTG")

The pseudocode for this algorithm is:

```
while True:
    points := process_lidar(lidar_scan)
    points := filter_by_safety_radius(points)
    sequence := find_longest_sequence_of_safe_points(points)
    target_point := sequence[len(sequence)//2]
endwhile
```