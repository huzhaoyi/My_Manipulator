# M5机械臂 MoveIt Setup Assistant 配置指南

## 机器人结构分析

### 链接（Links）结构：
```
dummy (虚拟链接)
  └── base_link (基座)
      ├── BottomLink (固定)
      └── Link1 (关节1)
          └── Link2 (关节2)
              └── Link3 (关节3)
                  └── Link4 (关节4)
                      ├── LinkGG (固定，可能是末端执行器基座)
                      ├── LinkGL (左手指，可动)
                      └── LinkGR (右手指，可动)
```

### 关节（Joints）分析：

**可动关节（Revolute）：**
1. **Joint1**: 绕Z轴旋转 (base_link -> Link1)
   - 范围: -3.14 到 3.14 弧度（±180°）
   - 作用：基座旋转

2. **Joint2**: 绕-X轴旋转 (Link1 -> Link2)
   - 范围: -1.57 到 0 弧度（-90° 到 0°）
   - 作用：肩部俯仰

3. **Joint3**: 绕-X轴旋转 (Link2 -> Link3)
   - 范围: -1.57 到 0 弧度（-90° 到 0°）
   - 作用：肘部俯仰

4. **Joint4**: 绕Z轴旋转 (Link3 -> Link4)
   - 范围: -3.14 到 3.14 弧度（±180°）
   - 作用：腕部旋转

5. **JointGL**: 绕-Y轴旋转 (Link4 -> LinkGL)
   - 范围: -1.01 到 1.01 弧度（约±58°）
   - 作用：左手指开合

6. **JointGR**: 绕-Y轴旋转 (Link4 -> LinkGR)
   - 范围: -1.01 到 1.01 弧度（约±58°）
   - 作用：右手指开合

**夹爪与实机轴映射**：运行时 axis5 约定为 **0=闭合**、**-1100=全开**；JointGL/JointGR 弧度与 axis5 线性对应，由 m5_hardware 与 m5_grasp 夹爪直接 UDP 控制（无轨迹，见项目 README）。

**固定关节（Fixed）：**
- dummy_joint, BottomJoint, JointGG

---

## MoveIt Setup Assistant 配置步骤

### 1. 启动 Setup Assistant

```bash
cd /home/huzy/grasp_perception
source /opt/ros/humble/setup.bash  # 如果还没source
colcon build
source install/setup.bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

### 2. 创建新的MoveIt配置包

- 选择 **"Create New MoveIt Configuration Package"**
- URDF文件路径：`/home/huzy/grasp_perception/src/m5/urdf/m5_updated_from_csv.urdf`
- 配置包保存路径：建议 `src/m5_moveit_config`

### 3. Self-Collisions（自碰撞矩阵）

**推荐配置：**
- 点击 **"Generate Collision Matrix"** 自动生成
- 检查并手动调整以下可能冲突的链接对：
  - `Link1` 与 `Link3`
  - `Link2` 与 `Link4`
  - `LinkGL` 与 `LinkGR`（手指之间）
  - `base_link` 与 `Link2`, `Link3`, `Link4`（如果机器人会折叠）

**建议：**
- 保留自动生成的碰撞对
- 如果某些链接明显不会碰撞，可以禁用以提升性能

### 4. Virtual Joints（虚拟关节）

**配置：**
- **Joint Name**: `virtual_joint`
- **Child Link**: `base_link`
- **Parent Frame**: `world` (或 `odom`，根据你的需求)
- **Type**: `fixed` (如果机器人基座固定) 或 `planar`/`floating` (如果可移动)

**说明：**
- 虚拟关节定义了机器人相对于世界坐标系的位置
- 对于固定基座机械臂，通常使用 `fixed` 类型

### 5. Planning Groups（规划组）

这是最重要的配置部分！

#### 5.1 主机械臂组（Arm Group）

**详细配置步骤：**

1. **创建新的Planning Group**
   - 在左侧面板找到 "Planning Groups" 标签页
   - 点击右上角的 **"Add Group"** 按钮
   - 会弹出 "Add Planning Group" 对话框

2. **填写基本信息**
   - **Group Name**: 输入 `arm_group` 或 `manipulator`
   - 点击 **"Next"** 进入下一步

3. **选择运动学求解器（Kinematic Solver）**
   - 在 "Kinematic Solver" 下拉菜单中选择：
     - `kdl_kinematics_plugin/KDLKinematicsPlugin` (默认，推荐先使用)
     - 或 `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` (更鲁棒，但需要额外安装)
   - **Kin. Search Resolution**: 保持默认值 `0.005` (单位：弧度)
   - **Kin. Search Timeout**: 保持默认值 `0.005` (单位：秒)
   - 点击 **"Next"**

4. **添加组件到组（Add Components to Group）**
   - 点击 **"Add Components to Group"** 按钮
   - 会看到多个添加选项：`add kin.chain`、`add joints`、`add subgroups`、`add links`

5. **使用 "add kin.chain" 添加运动学链（推荐方法）**
   
   这是最简单和推荐的方法，会自动添加从Base到Tip的所有链接和关节：
   
   a. **点击 "add kin.chain" 按钮**
   
   b. **选择Base Link（基座链接）**
      - 在弹出的对话框中，在 "Base Link" 下拉菜单中，选择：`base_link`
      - 这是整个机械臂的根链接
   
   c. **选择Tip Link（末端链接）**
      - 在 "Tip Link" 下拉菜单中，选择：`Link4`
      - **重要**：选择 `Link4` 而不是 `LinkGG`、`LinkGL` 或 `LinkGR`
      - `Link4` 是腕部链接，是主机械臂的末端
      - 手指（LinkGL, LinkGR）应该单独配置在夹爪组中
   
   d. **自动添加的关节**
      - 系统会自动识别并添加从 `base_link` 到 `Link4` 之间的所有关节
      - 应该自动包含以下4个旋转关节：
        - ✅ `Joint1` (基座旋转，绕Z轴)
        - ✅ `Joint2` (肩部，绕-X轴)
        - ✅ `Joint3` (肘部，绕-X轴)
        - ✅ `Joint4` (腕部旋转，绕Z轴)
      
      - **注意**：如果自动包含了固定关节（如 `dummy_joint`、`BottomJoint`、`JointGG`），需要手动移除它们
      - **注意**：不应该包含手指关节（`JointGL`、`JointGR`）
   
   e. **确认添加**
      - 检查右侧的3D预览窗口，应该能看到从 `base_link` 到 `Link4` 的完整运动链
      - 确认所有4个关节都显示为可动关节
      - 点击 **"Save"** 或确认按钮完成添加

6. **替代方法：使用 "add joints" 手动添加关节**
   
   如果 "add kin.chain" 方法添加了不需要的关节，可以：
   
   a. **先使用 "add links" 添加链接**
      - 点击 **"add links"** 按钮
      - 勾选以下链接：`base_link`, `Link1`, `Link2`, `Link3`, `Link4`
      - 不要勾选：`dummy`, `BottomLink`, `LinkGG`, `LinkGL`, `LinkGR`
   
   b. **然后使用 "add joints" 添加关节**
      - 点击 **"add joints"** 按钮
      - 在关节列表中，**勾选**以下4个旋转关节：
        - ✅ `Joint1`
        - ✅ `Joint2`
        - ✅ `Joint3`
        - ✅ `Joint4`
      - **不要勾选**任何固定关节或手指关节

7. **完成配置**
   - 添加组件后，返回到Planning Groups主界面
   - 在左侧列表中会显示新创建的 `arm_group`
   - 可以点击它进行编辑或删除
   - 确认配置无误后，继续下一步或保存整个配置

**配置验证要点：**
- ✅ 应该包含4个旋转关节（Joint1-4）
- ✅ Base Link 是 `base_link`
- ✅ Tip Link 是 `Link4`
- ❌ 不应该包含任何固定关节
- ❌ 不应该包含手指关节（JointGL, JointGR）

#### 5.2 夹爪组（Gripper Group）

**详细配置步骤：**

1. **创建新的Planning Group**
   - 在 "Planning Groups" 标签页中，再次点击 **"Add Group"** 按钮
   - 创建第二个规划组用于夹爪

2. **填写基本信息**
   - **Group Name**: 输入 `gripper_group` 或 `gripper`
   - 点击 **"Next"**

3. **选择运动学求解器**
   - **重要**：对于夹爪，通常不需要逆运动学求解器
   - 在 "Kinematic Solver" 下拉菜单中，选择：`None` 或 `No kinematics plugin`
   - 如果下拉菜单中没有 "None" 选项，可以选择 `kdl_kinematics_plugin/KDLKinematicsPlugin`（虽然不会用到）
   - 点击 **"Next"**

4. **添加组件到组（Add Components to Group）**
   - 点击 **"Add Components to Group"** 按钮
   - 会看到多个添加选项：`add kin.chain`、`add joints`、`add subgroups`、`add links`

5. **使用 "add joints" 添加手指关节（推荐方法）**
   
   对于夹爪组，推荐直接使用 "add joints" 方法，因为手指关节是独立的：
   
   a. **点击 "add joints" 按钮**
   
   b. **选择关节（Joints）**
      - 在关节列表中，**勾选**以下2个手指关节：
        - ✅ `JointGL` (左手指，绕-Y轴旋转)
        - ✅ `JointGR` (右手指，绕-Y轴旋转)
      
      - **不要勾选**：
        - ❌ `JointGG` (这是固定关节，连接Link4到LinkGG)
        - ❌ 任何主机械臂的关节（Joint1-4）
        - ❌ 任何固定关节
   
   c. **确认添加**
      - 检查3D预览窗口，应该能看到两个手指（LinkGL, LinkGR）
      - 确认两个手指关节都显示为可动
      - 点击 **"Save"** 或确认按钮完成添加

6. **使用 "add links" 添加链接（如果需要）**
   
   如果需要显式添加链接：
   
   a. **点击 "add links" 按钮**
   
   b. **选择链接**
      - 勾选以下链接：
        - ✅ `Link4` (夹爪基座)
        - ✅ `LinkGG` (夹爪中心，固定)
        - ✅ `LinkGL` (左手指)
        - ✅ `LinkGR` (右手指)
      
      - **不要勾选**主机械臂的链接（Link1-3, base_link等）

7. **替代方法：使用 "add kin.chain"**
   
   如果你想使用运动学链方法：
   
   a. **点击 "add kin.chain" 按钮**
   
   b. **选择Base Link**: `Link4`
   
   c. **选择Tip Link**: `LinkGG` 或 `LinkGL` 或 `LinkGR`
      - 如果选择 `LinkGG`，系统会自动添加 `JointGG`（固定关节），需要手动移除
      - 如果选择 `LinkGL`，只会包含到左手指的链
      - 如果选择 `LinkGR`，只会包含到右手指的链
   
   d. **手动添加另一个手指**
      - 由于两个手指是并行的，可能需要分别添加
      - 或者使用 "add joints" 方法更简单

8. **完成配置**
   - 添加组件后，返回到Planning Groups主界面
   - 在左侧列表中会显示新创建的 `gripper_group`
   - 可以点击它进行编辑或删除

**配置验证要点：**
- ✅ 应该包含2个旋转关节（JointGL, JointGR）
- ✅ Base Link 是 `Link4`
- ✅ Tip Link 是 `LinkGG`（或 LinkGL/LinkGR）
- ❌ 不应该包含固定关节 `JointGG`
- ❌ 不应该包含主机械臂的关节（Joint1-4）

**界面操作提示：**
- 在关节选择界面，你可以：
  - 使用搜索框快速查找关节名称
  - 点击关节名称查看其详细信息（类型、限制等）
  - 使用 "Select All" 和 "Deselect All" 按钮快速选择/取消选择
- 3D预览窗口会实时显示你选择的运动链
- 如果看不到机器人模型，检查URDF文件路径是否正确

**常见问题：**

1. **找不到某个链接或关节？**
   - 确保URDF文件已正确加载
   - 检查链接名称大小写是否匹配（区分大小写）
   - 在Setup Assistant中，点击 "Reload" 重新加载URDF

2. **关节列表为空？**
   - 检查Base Link和Tip Link是否在同一个运动链中
   - 确保Tip Link是Base Link的子链接（在URDF树结构中）

3. **选择了错误的关节？**
   - 可以点击已创建的组进行编辑
   - 或删除后重新创建

4. **如何确认配置正确？**
   - 查看3D预览窗口，应该只显示从Base到Tip的链接
   - 检查关节数量：arm_group应该有4个，gripper_group应该有2个
   - 在后续步骤中测试预设姿态，看是否能正确运动

**可选配置：**
- 如果两个手指需要同步运动，可以在夹爪控制器中配置同步，或创建虚拟关节连接两个手指

#### 5.3 完整机器人组（可选）

**配置：**
- **Group Name**: `robot_group` 或 `whole_body`
- **Kinematic Solver**: 同主机械臂组
- **Subgroups**: 
  - `arm_group`
  - `gripper_group`

### 6. Robot Poses（机器人预设姿态）

**推荐创建以下预设姿态：**

1. **home** (初始位置)
   - Joint1: 0
   - Joint2: -0.785 (约-45°)
   - Joint3: -0.785 (约-45°)
   - Joint4: 0
   - JointGL: 0
   - JointGR: 0

2. **extended** (伸展姿态)
   - Joint1: 0
   - Joint2: -0.5
   - Joint3: -0.5
   - Joint4: 0
   - JointGL: 0
   - JointGR: 0

3. **folded** (折叠姿态，用于存储)
   - Joint1: 0
   - Joint2: -1.0 (避免完全折叠到-1.57，防止碰撞)
   - Joint3: -1.57 (避免完全折叠到-1.57，防止碰撞)
   - Joint4: 0
   - JointGL: 0
   - JointGR: 0
   
   **⚠️ 碰撞问题说明**：
   - 如果设置为-1.57（最大角度），MoveIt会提示 **"robot in collision state"**（机器人处于碰撞状态）
   - **原因**：当Joint2和Joint3都达到-1.57时，机器人完全折叠，Link1和Link3可能会发生自碰撞
   - **解决方案**：
     1. **调整角度**：使用-1.2到-1.3之间的值（推荐）
     2. **调整自碰撞矩阵**：在Self-Collisions配置中，禁用Link1和Link3之间的碰撞检测（不推荐，因为这是真实的碰撞）
     3. **使用不同的折叠姿态**：让Joint2和Joint3不完全相同，例如Joint2=-1.2, Joint3=-1.0
   
   **推荐值**：
   - Joint2: -1.2 到 -1.3 弧度（约-69°到-75°）
   - Joint3: -1.2 到 -1.3 弧度（约-69°到-75°）
   - 这样既能达到折叠效果，又能避免碰撞警告

### 7. End Effectors（末端执行器）

**配置：**
- **Name**: `gripper` 或 `end_effector`
- **Group**: `gripper_group`
- **Parent Link**: `Link4`
- **Parent Group**: `arm_group`

**说明：**
- 定义末端执行器有助于MoveIt理解完整的运动链
- 这对于抓取任务很重要

### 8. Passive Joints（被动关节）

**配置：**
- 通常不需要配置
- 如果你的机器人有被动关节（如弹簧关节），在这里添加

### 9. ROS 2 Controllers（ROS 2控制器）

**配置（选择默认的自动配置）：**

#### 9.1 机械臂控制器

```yaml
arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints:
    - Joint1
    - Joint2
    - Joint3
    - Joint4
```

#### 9.2 夹爪控制器

```yaml
gripper_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints:
    - JointGL
    - JointGR
```

**注意：**
- 控制器配置取决于你的硬件接口
- 如果使用Gazebo仿真，可能需要不同的控制器类型

### 10. 3D Perception（3D感知，可选）

如果使用点云或深度相机：
- 配置传感器插件
- 设置点云话题
- 配置Octomap参数

### 11. Author Information（作者信息）

填写包的基本信息：
- 作者名称
- 邮箱
- 描述

### 12. 生成配置文件

- 点击 **"Generate Configuration Files"**
- 检查是否有错误
- 保存配置

---

## 配置后的验证

### 1. 编译配置包

```bash
cd /home/huzy/grasp_perception
colcon build
source install/setup.bash
```

### 2. 启动MoveIt

**注意**：根据你在MoveIt Setup Assistant中设置的配置包名称启动。

如果配置包名称是 `m5_configure`（默认）：
```bash
ros2 launch m5_bringup rviz.launch.py
```

如果配置包名称是 `m5_moveit_config`：
```bash
ros2 launch m5_bringup rviz.launch.py
```

**查找你的配置包名称**：
```bash
# 查看src目录下的配置包
ls src/
# 查看package.xml中的包名
cat src/m5_configure/package.xml | grep "<name>"
```

**重要**：配置包的目录名必须与package.xml中的包名一致，否则编译会失败。

### 3. 在RViz中测试

- 使用Motion Planning插件
- 测试预设姿态
- 测试路径规划
- 检查碰撞检测

---

## 常见问题与建议

### 1. 碰撞状态问题（Robot in Collision State）

**问题描述**：
- 在配置预设姿态时，可能会看到 "robot in collision state" 警告
- 这表示机器人在该姿态下会发生自碰撞

**常见原因**：
- Joint2 和 Joint3 都设置为接近 -1.57（完全折叠）
- Link1 和 Link3 在折叠时可能碰撞
- Link2 和 Link4 在极端位置可能碰撞

**解决方案**：
1. **调整预设姿态角度**（推荐）
   - 避免使用最大角度值
   - Joint2 和 Joint3 使用 -1.2 到 -1.3 而不是 -1.57
   
2. **检查自碰撞矩阵**
   - 在 Self-Collisions 配置中，确认相关链接对的碰撞检测已启用
   - 如果某些链接确实不会碰撞，可以禁用以提高性能
   
3. **使用不同的关节组合**
   - 尝试不同的 Joint2 和 Joint3 组合
   - 例如：Joint2=-1.2, Joint3=-1.0（不完全对称折叠）

**注意**：碰撞警告是MoveIt的保护机制，确保规划的安全性。不要简单地禁用碰撞检测来消除警告，而应该调整姿态使其在物理上可行。

### 2. 奇异位置问题

你的机器人可能在以下位置出现奇异：
- Joint2 和 Joint3 都接近 -1.57 时（完全折叠）
- 建议在规划时避开这些位置

### 2. 手指同步

如果两个手指需要同步运动：
- 在夹爪控制器中配置同步
- 或创建虚拟关节连接两个手指

### 3. 工作空间限制

根据关节限制，计算实际工作空间：
- Joint2 和 Joint3 的限制（-1.57 到 0）限制了机器人的伸展范围
- 考虑在实际应用中可能需要调整这些限制

### 4. 碰撞检测优化

- 如果性能有问题，可以简化碰撞模型
- 使用包围盒（bounding boxes）代替精确的STL网格

---

## 下一步

配置完成后，你可以：
1. 集成真实硬件控制器
2. 配置Gazebo仿真环境
3. 开发抓取和操作任务
4. 设置感知和规划管道
