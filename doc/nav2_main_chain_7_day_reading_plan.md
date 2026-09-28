# Nav2 主链 7 天阅读清单

这份清单只追一条纵向主链：

```text
launch -> yaml -> bt_navigator -> BtActionServer -> BehaviorTreeEngine
  -> BT action node -> planner/controller server -> nav2_core plugin
```

目标不是读完 Navigation2 的所有包，而是先搞清楚一次导航请求是怎么从外部目标一路流到规划器、控制器和插件实现里的。

## 第 1 天：启动入口和参数

目标：搞清 Nav2 启动了哪些节点，以及参数从哪里来。

阅读文件：

- `nav2_bringup/launch/bringup_launch.py`
- `nav2_bringup/launch/navigation_launch.py`
- `nav2_bringup/launch/localization_launch.py`
- `nav2_bringup/params/nav2_params.yaml`

重点问题：

- `navigation_launch.py` 启动了哪些 server？ control pathsmooth vel_smooth planner behavior bt_navigator follow_watpoint lifecycle_manager
- 哪些节点是 lifecycle node？
- 这里的 `Node` 和 `ComposableNode` 有什么区别？前者每一个node一个独立进程，后者是所有node在同一个进程，后者好处在于各节点之间的数据交换效率
- `parameters=[configured_params]` 是怎么把 launch 和 `nav2_params.yaml` 连起来的？
- `bt_navigator`、`planner_server`、`controller_server` 分别在 YAML 的哪里配置？ plugin

完成标准：

- 你能说出主要导航 server 的名字和各自职责。
- 你能在 `nav2_params.yaml` 里找到每个 server 对应的配置段。

## 第 2 天：bt_navigator 入口

目标：搞清高层导航请求是谁接收的。

阅读文件：

- `nav2_bt_navigator/src/bt_navigator.cpp`
- `nav2_bt_navigator/include/nav2_bt_navigator/bt_navigator.hpp`
- `nav2_bt_navigator/include/nav2_bt_navigator/navigator.hpp`
- `nav2_bt_navigator/src/navigators/navigate_to_pose.cpp`
- `nav2_bt_navigator/src/navigators/navigate_through_poses.cpp`

重点问题：

- `bt_navigator` 对外提供了哪些 action 接口？
- `NavigateToPoseNavigator` 和 `NavigateThroughPosesNavigator` 有什么区别？
- 默认行为树 XML 是在哪里选择的？
- `BtActionServer` 是在哪里创建的？
- `plugin_lib_names`

完成标准：

- 你能解释一个 `NavigateToPose` 请求是怎么进入行为树的。
- 你能指出单目标导航和多目标导航各自默认使用哪个 XML 文件。

## 第 3 天：行为树运行框架

目标：搞清 XML 是怎么变成一棵正在运行的行为树的。

阅读文件：

- `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server.hpp`
- `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp`
- `nav2_behavior_tree/include/nav2_behavior_tree/behavior_tree_engine.hpp`
- `nav2_behavior_tree/src/behavior_tree_engine.cpp`

重点问题：

- `BehaviorTreeEngine` 是在哪里构造的？
- BT 插件库是在哪里加载的？
- blackboard 是在哪里创建的？
- `tree->tickRoot()` 是在哪里周期性运行的？
- 导航目标被取消时，行为树是怎么停下来的？

完成标准：

- 你能解释 `BtActionServer`、`BehaviorTreeEngine`、XML 和 blackboard 的关系。
- 你能说清楚是谁在周期性调用 `tickRoot()`。

## 第 4 天：通用 BT Action 节点

目标：搞清 Nav2 里很多 action 型 BT 节点共用的父类。

阅读文件：

- `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp`

重点问题：

- `BtActionNode` 为什么要创建 action client？
- 它为什么要创建 callback group 和本地 executor？
- 当节点状态是 `IDLE` 时，`tick()` 会做什么？
- `future_goal_handle_`、`goal_handle_`、`result_` 分别表示什么？
- 为什么这里需要 `spin_some()`？
- `halt()` 是怎么取消正在运行的 action 的？

完成标准：

- 你能描述这个状态流：

```text
IDLE -> 发送 goal -> 等 server 接收 -> 等 result -> 返回 BT 状态
```

- 你能解释为什么 action 的 feedback/result 回调仍然需要 executor 去 spin。


## 第 5 天：具体 BT 节点和 blackboard 数据流

目标：搞清 XML 标签如何对应到 C++ BT 节点，以及数据如何在 blackboard 里流动。

阅读文件：

- `nav2_behavior_tree/include/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp`
- `nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp`
- `nav2_behavior_tree/include/nav2_behavior_tree/plugins/action/compute_path_through_poses_action.hpp`
- `nav2_behavior_tree/plugins/action/compute_path_through_poses_action.cpp`
- `nav2_behavior_tree/include/nav2_behavior_tree/plugins/action/follow_path_action.hpp`
- `nav2_behavior_tree/plugins/action/follow_path_action.cpp`
- `nav2_behavior_tree/CMakeLists.txt`

重点问题：

- 每个节点通过 `BT_REGISTER_NODES` 注册了哪个 XML 标签？ 
- 哪些 input port 是通过 `getInput()` 读取的？
- 哪些 output port 是通过 `setOutput()` 写回去的？
- `planner_id="GridBased"` 是怎么传到 `planner_server` 的？
- `{path}` 是怎么从规划节点传到控制节点的？ 
- 每个 BT 节点是怎么变成 `plugin_lib_names` 里的动态库的？

完成标准：

- 你能解释 `<ComputePathToPose>` 是怎么调用 `planner_server` 的。
- 你能解释 `<FollowPath>` 是怎么拿到规划结果 `path` 的。

## 第 6 天：XML 控制流

目标：搞清默认行为树 XML 里写的导航逻辑。

阅读文件：

- `nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
- `nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`
- `nav2_behavior_tree/plugins/control/pipeline_sequence.cpp`
- `nav2_behavior_tree/plugins/control/recovery_node.cpp`
- `nav2_behavior_tree/plugins/control/round_robin_node.cpp`
- `nav2_behavior_tree/plugins/decorator/rate_controller.cpp`

重点问题：

- 为什么导航流程里先规划，再跟踪路径？
- `RateController` 限制了什么？
- `RecoveryNode` 什么时候会执行恢复动作？
- `RoundRobin` 为什么要轮流执行恢复动作？
- 哪些子树每轮都会被 tick，哪些不会？

完成标准：

- 你能从根节点开始讲一遍默认 `NavigateToPose` XML 的执行路径。
- 你能区分正常导航路径和恢复路径。

## 第 7 天：Server 和 Plugin 的边界

目标：把 BT 节点调用 server action 的过程，继续追到具体算法插件。

阅读文件：

- `nav2_planner/src/planner_server.cpp`
- `nav2_planner/include/nav2_planner/planner_server.hpp`
- `nav2_controller/src/controller_server.cpp`
- `nav2_controller/include/nav2_controller/controller_server.hpp`
- `nav2_core/include/nav2_core/global_planner.hpp`
- `nav2_core/include/nav2_core/controller.hpp`
- `nav2_navfn_planner/src/navfn_planner.cpp`
- `nav2_theta_star_planner/src/theta_star_planner.cpp`

重点问题：

- `ComputePathToPose` 调用的是 `planner_server` 的哪个 action？
- `planner_server` 是在哪里加载 `planner_plugins` 的？
- `planner_id` 是怎么选择具体 planner 实例的？
- `createPlan()` 最终在哪里被调用？
- `FollowPath` 调用的是 `controller_server` 的哪个 action？
- `controller_server` 是在哪里加载 `controller_plugins` 的？

完成标准：

- 你能完整追踪下面这条链：

```text
NavigateToPose goal
  -> bt_navigator
  -> behavior tree XML
  -> ComputePathToPose BT node
  -> planner_server action
  -> GlobalPlanner plugin
  -> FollowPath BT node
  -> controller_server action
  -> Controller plugin
  -> cmd_vel
```

## 7 天之后

做一个小练习，把阅读变成真正修改：

- 在 `nav2_params.yaml` 里把全局规划器从 NavFn 换成 Theta Star。在yaml里改一下 GridBased 下的 plugin 即可，改为nav2_theta_star_planner/ThetaStarPlanner
- 运行时确认日志里显示 `planner_server` 加载了 `nav2_theta_star_planner/ThetaStarPlanner`。
- 再尝试写一个最小自定义 `GlobalPlanner` 插件，让它返回一条简单路径。

这个练习能把主链从“看懂”变成“真的改得动”。
