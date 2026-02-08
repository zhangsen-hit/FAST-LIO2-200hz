# 高频里程计方案分析：方案 A 与方案 B 对比

## 一、项目现状简述

- **已有逻辑**：`laserMapping.cpp` 主循环中，每同步到一帧 LiDAR + 对应 IMU 后：
  1. 调用 `p_imu->Process(Measures, kf, feats_undistort)`，内部在 **IMU_Processing.hpp** 的 `UndistortPcl()` 里：
     - 从**上一帧 LiDAR 校正后的状态**出发，按**每个 IMU 时刻**做 ESKF 前向传播（`kf_state.predict(dt, Q, in)`），
     - 并把每个 IMU 时刻的 (pos, vel, rot) 存入 **IMUpose**（世界系，即 `camera_init`）。
  2. 随后做 LiDAR 的 ICP / 迭代 EKF 更新：`kf.update_iterated_dyn_share_modified(...)`，得到本帧末尾的精确状态 `state_point`。
  3. **仅**在此时发布一次低频里程计：`publish_odometry(pubOdomAftMapped)`，时间戳为 `lidar_end_time`。

因此：**高频状态（每个 IMU 时刻的位姿）在现有代码里已经算过**，只是没有对外发布，且中间时刻是“仅 IMU 预测”，只有帧末经过 LiDAR 校正。

---

## 二、方案 A：在现有节点内新增话题发布 IMU 频率里程计

### 2.1 思路

- 不改变现有**估计与更新**的任何逻辑，仅在**发布层**做加法。
- 利用已有 **IMUpose**（每个 IMU 时刻的 pos/vel/rot）和 LiDAR 更新后的 **state_point**（帧末精确估计），新增一个话题，按 IMU 时刻发布高频里程计。

### 2.2 实现要点

1. **暴露 IMUpose 与时间基准**  
   - `IMUpose` 在 `ImuProcess` 中为 **private**，需在 **IMU_Processing.hpp** 中增加接口，例如：
     - `get_IMUpose()`：返回本帧的 `IMUpose`（或 const 引用）。
     - 本帧时间基准：在 `UndistortPcl` 中使用的 `pcl_beg_time`（一般等于 `meas.lidar_beg_time`）需一并暴露，以便为每个 pose 构造时间戳：  
       `t_i = pcl_beg_time + IMUpose[i].offset_time`。

2. **在 laserMapping.cpp 中新增发布**  
   - 在 `kf.update_iterated_dyn_share_modified` 之后、现有 `publish_odometry(pubOdomAftMapped)` 之后（或与之并列）：
     - 取 `p_imu->get_IMUpose()` 与时间基准；
     - 对 IMUpose 中每个位姿：用其 pos/rot 填 `nav_msgs::Odometry`，时间戳为 `pcl_beg_time + pose.offset_time`，发布到**新话题**（如 `/Odometry_high_freq`）；
     - 再发**一条**时间戳为 `lidar_end_time`、位姿为当前 `state_point` 的 Odometry（与主里程计一致），这样整段轨迹在时间上连续且最后一刻与低频里程计一致。

3. **不触碰的部分**  
   - 不修改 `Process`/`UndistortPcl` 内部计算；
   - 不修改 `kf` 的 predict/update 调用；
   - 原有 `/Odometry` 与 TF 的发布逻辑、频率、内容均保持不变。

### 2.3 特性与注意点

- **一致性**：高频位姿与现有系统完全一致——同一套 ESKF、bias、重力、外参，无额外模型或参数。
- **延迟**：高频轨迹是“每来一帧 LiDAR 才发布这一段”的，存在约**一帧 LiDAR 的延迟**（例如 10 Hz LiDAR 则约 100 ms）。
- **精度**：中间 IMU 时刻为“仅 IMU 预测”，未含当前帧 LiDAR 的校正；帧末一点与低频里程计一致。若下游需要“尽量准 + 与主里程计一致”，可只使用帧末那一条；若需要“平滑的高频轨迹”，可接受中间为预测值。

---

## 三、方案 B：独立节点订阅低频里程计 + 原始 IMU，ESKF 插值

### 3.1 思路

- 不修改 Fast-LIO 源码，单独写一个节点：
  - 订阅 `/Odometry`（低频）和 `/imu`（或当前项目使用的 IMU 话题）；
  - 在每两个低频位姿之间，用 IMU 做前向传播（ESKF 或等效积分），得到 IMU 频率的位姿并发布。

### 3.2 实现要点

- 需要自行实现或复现：
  - 与 Fast-LIO 一致的**运动模型**（见 `use-ikfom.hpp` 中 `get_f` 等）和**过程噪声/协方差**；
  - bias、重力在传播中的处理方式；
  - 时间对齐（低频 odom 时间戳与 IMU 时间戳）、插值或外推策略。
- 若实现与 Fast-LIO 完全一致，理论上可得到与方案 A 相近的高频轨迹；若简化（如忽略 bias 或使用不同 Q），则存在**模型/参数不一致**，可能带来微小偏差或不一致。

### 3.3 特性与注意点

- **无侵入**：不碰 Fast-LIO，部署灵活，可独立更新。
- **实时性**：可以在收到每条 IMU 时立即发布当前 propagated 位姿，延迟可做到很小。
- **一致性与工作量**：与主滤波完全一致取决于你对 IKFoM/ESKF 和参数的复现程度；实现与调试工作量相对更大。

---

## 四、综合对比与建议

| 维度           | 方案 A（本节点内发布）           | 方案 B（独立节点 ESKF 插值）     |
|----------------|----------------------------------|----------------------------------|
| 与现有精度/逻辑 | 完全一致，仅读已有结果并发布     | 取决于复现程度，易产生细微差异   |
| 对原代码影响   | 仅增接口 + 发布，不改估计流程    | 无                               |
| 实现难度       | 低（暴露 IMUpose + 时间 + 循环发布） | 中高（运动模型、Q、时间同步等）  |
| 发布延迟       | 约一帧 LiDAR 延迟                | 可做到接近实时                   |
| 维护与一致性   | 与主滤波天然一致                 | 需随 Fast-LIO 参数/模型变更而维护 |

**结论与建议：**

- **若优先保证“与当前系统完全一致、且不影响原有逻辑与精度”**：  
  **推荐方案 A**。项目里已有完整的高频前向传播与 IMUpose，只需只读式地暴露并多发布一条话题即可，对原估计零影响，可行性高、实现快。

- **若必须“严格实时、IMU 一到就发”（且不愿在 Fast-LIO 内改任何发布逻辑）**：  
  可考虑在**本节点内**做方案 A 的变体：在 IMU 回调中，用**上一帧 LiDAR 校正后的状态**做一次前向传播并发布当前时刻位姿（需拷贝状态与传播逻辑或调用可复用的 predict），这样仍与主滤波一致，且延迟小。这与“外部订阅 + ESKF”的 B 相比，一致性和可维护性更好。

- **若坚持不改 Fast-LIO 且接受独立实现**：  
  方案 B 可行，能获得 IMU 频率的插值轨迹，但需要在文档中说明：bias/重力/过程噪声需与 Fast-LIO 对齐，否则存在细微不一致风险。

---

## 五、方案 A 的最小改动清单（便于实现）

1. **include/IMU_Processing.hpp**  
   - 在 `ImuProcess` 中增加：
     - `double get_pcl_beg_time() const { return last_pcl_beg_time_; }`（或等价接口）；
     - `const vector<Pose6D> & get_IMUpose() const { return IMUpose; }`（若保持 IMUpose 为 private）；
   - 在 `UndistortPcl` 末尾（或合适处）给 `last_pcl_beg_time_` 赋值：`last_pcl_beg_time_ = pcl_beg_time;`（需增加成员 `double last_pcl_beg_time_;`）。

2. **src/laserMapping.cpp**  
   - 声明并创建新 publisher：`ros::Publisher pubOdomHighFreq = nh.advertise<nav_msgs::Odometry>("/Odometry_high_freq", 100000);`
   - 在 `publish_odometry(pubOdomAftMapped)` 之后：
     - 取 `pcl_beg = p_imu->get_pcl_beg_time()`，`const auto & imu_poses = p_imu->get_IMUpose();`
     - 对 `imu_poses` 中每个元素，用 `pos/rot` 填 `nav_msgs::Odometry`，时间戳 `pcl_beg + pose.offset_time`，发布到 `pubOdomHighFreq`；
     - 再发一条时间戳 `lidar_end_time`、位姿为当前 `state_point` 的 Odometry 到 `pubOdomHighFreq`。

3. **配置/launch**  
   - 可选：为“是否发布高频里程计”增加一个 rosparam，默认 true，便于关闭。

按上述清单实现即可在**不影响原有逻辑和精度**的前提下，得到与现有系统一致的 IMU 频率里程计输出。

---

## 六、已实现说明（方案 A）

- **高频里程计话题**：`/Odometry_high_freq`（`nav_msgs/Odometry`），与 `/Odometry` 同坐标系（`camera_init` → `body`）。
- **开关参数**：`publish/high_freq_odom_en`（bool，默认 `true`）。在 launch 或 yaml 中设为 `false` 可关闭高频发布。
- **发布内容**：每处理完一帧 LiDAR，先按 IMUpose 中每个时刻发布一条（时间戳 = `pcl_beg_time + offset_time`，含位姿与速度），再发布一条时间戳为 `lidar_end_time` 的 LiDAR 校正后状态，与低频 `/Odometry` 一致。
