# nav2_collision_monitor backported from Jazzy (agv-humble-custom)

- Source: https://github.com/ros-navigation/navigation2 branch `jazzy`, commit d5d768b7 (2026-10-27), whole package copied.
- Reason: native state publishing (`CollisionMonitorState`), STOP on invalid source, `VelocityPolygon`, dynamic polygons via
  `polygon_sub_topic`, LIMIT action, `~/toggle` service; the AGV navigation uses it as its stop backstop (DESIGN.md phase 5).
  Approved under the AGENTS.md leaf-package backport exception (2026-10-31).
- Additive backports outside this package (no existing Humble interface changed):
  - `nav2_util`: `array_parser.hpp/.cpp`, `twist_publisher.hpp`, `twist_subscriber.hpp` (from Jazzy),
    `validateTwist(const TwistStamped &)` overload in `robot_utils`, `setSoftRealTimePriority()` in `node_utils`;
  - `nav2_msgs`: `msg/CollisionDetectorState.msg`, `srv/Toggle.srv`.
- Humble adaptation inside this package: `collision_detector_node.cpp` uses `rclcpp::create_timer(...)` (Humble
  `rclcpp_lifecycle::LifecycleNode` has no `create_timer`).
- Humble adaptation (2026-11-02): `geometry_msgs/PolygonInstanceStamped` is missing from some Humble geometry_msgs builds
  (the RK3588 vehicle image ships ros-humble-geometry-msgs 4.9.1 without it; the PC has 4.9.0 with it). CMake checks for
  the header; without it `polygon_source.cpp` is not built (`NAV2_COLLISION_MONITOR_HAS_POLYGON_SOURCE` undefined, a
  `type: polygon` observation source then fails configuration as an unknown source type) and the three tests that use the
  message (sources_test, collision_monitor_node_test, collision_detector_node_test) are skipped. The node headers no longer
  include `polygon_source.hpp` (the node sources do, under the macro). Scan / pointcloud / range sources are unaffected.
- Tests on Humble (2026-10-31, x86_64): collision_monitor_node 20/20, collision_detector_node 14/14, polygons 27/27,
  velocity_polygons 7/7, sources 6/6, kinematics 2/2. uncrustify reports a style diff in `src/pointcloud.cpp` (Humble vs Jazzy
  uncrustify configuration; upstream file left unchanged). One full `colcon test` run had collision_monitor_node_test produce no
  result file; three reruns passed (not reproduced).
- Local patch (agv-humble-custom, 2026-11-01, not upstream): exclusion polygon in `CollisionMonitor`
  (`collision_monitor_node.hpp/.cpp`, marked `(agv-humble-custom)`). Parameters `exclusion_polygon` (string of points in
  `base_frame_id`, default empty) and `exclusion_polygon_topic` (`PolygonStamped` in `base_frame_id`, transient_local,
  default empty; an empty polygon disables it). In `process()` the points of every source that lie inside it are dropped
  before the polygons are checked: the robot body or its carried load seen by the sensors (the AGV navigation publishes
  the body / loaded footprint). Default behaviour is unchanged when neither parameter is set. Covered by
  `agv_nav/test/test_collision_monitor_host.cpp` (ExcludesPointsInsideFootprint).
- Local fix (agv-humble-custom, 2026-11-02): `on_configure` of `CollisionMonitor` and `CollisionDetector` creates the
  `tf2_ros::TransformListener` at the end of a successful configuration instead of at the start. On Humble, destroying a TransformListener whose dedicated thread
  has not started spinning yet hangs forever (`executor->cancel()` before `spin()` is lost and `join()` never returns);
  a failed configuration called `on_cleanup()` right after creating it. Symptom: collision_monitor_node_test timed out
  under `colcon test` (hung in "Cleaning up" after a failed configure, e.g. testPolygonsNotSet): 2 of 4 runs without the
  exclusion patch, so it was pre-existing. After the monitor fix, collision_monitor_node_test and the agv_nav host test
  (which hung the same way) passed 10 of 10 `colcon test` runs, but collision_detector_node_test still timed out once
  in 10. After the same fix in the detector, collision_monitor_node_test and collision_detector_node_test passed 10 of 10
  further runs. A timed-out run can leave the test binary orphaned (ctest only kills the run_test.py
  wrapper); kill it before rerunning, otherwise the next run collides with it over DDS.
- Updating: re-copy the package from `upstream/jazzy` and re-apply the adaptation and the local patch above; do not merge
  it from `upstream/humble`.
