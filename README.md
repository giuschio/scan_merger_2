# ROS2 laser scan merger (ROS2 Humble)
Converts two messages of type `sensor_msgs/msg/LaserScan` from topics `front_scan` and `rear_scan` into a single laser scan of the same type, published under topic `scan` and/or a point cloud of type `sensor_msgs/msg/PointCloud2`, published under topic `pcl`.

## Acknowledgements
The code in this repository is essentially a ROS2 port (limited to the scan_merger node) of the code in [https://github.com/jk-ethz/obstacle_detector](https://github.com/jk-ethz/obstacle_detector), which itself is a fork of [https://github.com/tysik/obstacle_detector](https://github.com/tysik/obstacle_detector). You can find the original LICENSE [here](/ORIGINAL_LICENSE).


## Laser scan merger node description and parameters
This node converts two messages of type `sensor_msgs/msg/LaserScan` from topics `front_scan` and `rear_scan` into a single laser scan of the same type, published under topic `scan` and/or a point cloud of type `sensor_msgs/msg/PointCloud2`, published under topic `pcl`. The difference between both is that the resulting laser scan divides the area into finite number of circular sectors and put one point (or actually one range value) in each section occupied by some measured points, whereas the resulting point cloud simply copies all of the points obtained from sensors.

The input laser scans are firstly rectified to incorporate the motion of the scanner in time (see `laser_geometry` package). Next, two PCLs obtained from the previous step are synchronized and transformed into the target coordinate frame at the current point in time.

The resulting messages contain geometric data described with respect to a specific coordinate frame (e.g. `robot`). Assuming that the coordinate frames attached to two laser scanners are called `front_scanner` and `rear_scanner`, both transformation from `robot` frame to `front_scanner` frame and from `robot` frame to `rear_scanner` frame must be provided. The node allows to artificially restrict measured points to some rectangular region around the `robot` frame as well as to limit the resulting laser scan range. The points falling behind this region will be discarded.

Even if only one laser scanner is used, the node can be useful for simple data pre-processing, e.g. rectification, range restriction or recalculation of points to a different coordinate frame. The node uses the following set of local parameters:

* `~active` (`bool`, default: `true`) - active/sleep mode,
* `~publish_scan` (`bool`, default: `false`) - publish the merged laser scan message,
* `~publish_pcl` (`bool`, default: `true`) - publish the merged point cloud message,
* `~ranges_num` (`int`, default: `1000`) - number of ranges (circular sectors) contained in the 360 deg laser scan message,
* `~min_scanner_range` (`double`, default: `0.05`) - minimal allowable range value for produced laser scan message,
* `~max_scanner_range` (`double`, default: `10.0`) - maximal allowable range value for produced laser scan message,
* `~min_x_range` (`double`, default: `-10.0`) - limitation for points coordinates (points with coordinates behind these limitations will be discarded),
* `~max_x_range` (`double`, default: `10.0`) - as above,
* `~min_y_range` (`double`, default: `-10.0`) - as above,
* `~max_y_range` (`double`, default: `10.0`) - as above,
* `~fixed_frame_id` (`string`, default: `map`) - name of the fixed coordinate frame used for scan rectification in time,
* `~target_frame_id` (`string`, default: `map`) - name of the coordinate frame used as the origin for the produced laser scan or point cloud.

In the original implementation, the `~target_frame_id` is the `robot` frame. However, with a moving robot, this causes the Kalman filters later on to think that the points are moving even though the robot is moving w. r. t. the map. That's why we enhance the point cloud with "range" (distance) information in the `scans_merger` node. This means that even though the origin of the target frame does not match the lidar scan origin, we still preserve the distance of each point to the scanner that produced the point. For that reason, it is absolutely essential to run the scans through the `scans_merger` node, even if not merging scans and to set `publish_pcl` to `true`.
