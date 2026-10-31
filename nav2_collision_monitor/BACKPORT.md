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
- Tests on Humble (2026-10-31, x86_64): collision_monitor_node 20/20, collision_detector_node 14/14, polygons 27/27,
  velocity_polygons 7/7, sources 6/6, kinematics 2/2. uncrustify reports a style diff in `src/pointcloud.cpp` (Humble vs Jazzy
  uncrustify configuration; upstream file left unchanged). One full `colcon test` run had collision_monitor_node_test produce no
  result file; three reruns passed (not reproduced).
- Updating: re-copy the package from `upstream/jazzy` and re-apply the adaptation above; do not merge it from `upstream/humble`.
