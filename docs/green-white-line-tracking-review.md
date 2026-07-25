# 绿底白线 I 型巡线方案评审与收敛记录

## 1. 范围

本文只讨论当前仓库中完整可读、可独立运行的绿底白线 I 型巡线模式：

```bash
./pi_service/run_green_white_scanline_i_console.sh
```

对应运行模式为 `scanline_i_green_white`。本文不讨论：

- 黑线 / Otsu 巡线；
- 缺少 `third_party/DeskMate-Advance` 源码的 generic 路线；
- 更换相机采集链路；
- 深度学习；
- 本轮不必要的复杂状态估计或硬件升级。

当前目标是在现有相机、现有 OpenCV 和现有电机接口上，优先解决：

1. 车辆经常冲出白线；
2. 转弯角度不精确；
3. 车辆冲出或转错后很难恢复；
4. 方案过度依赖车辆始终处于正确初始位置和正确朝向。

## 2. 已确认不处理的部分

### 2.1 相机取帧

相机取帧链路已经完成实测，本轮不修改、不作为问题来源继续展开。

涉及代码：

- `pi_service/robot_web/camera.py`
- `pi_service/robot_web/scanline_i_route.py`

## 3. 当前绿底白线控制链

```text
run_green_white_scanline_i_console.sh
    -> app.py 选择 scanline_i_green_white
    -> GreenWhiteScanlineIShapeRouteTracker
    -> GreenWhiteHybridScanlineAnalyzer
    -> HSV 绿底 / 白线 / 红色标记分割
    -> 近端连通域选择
    -> 三行扫描线 + 横线 / 骨架证据
    -> IShapeTurnaroundPlanner
    -> 直行差速 PWM 或右向原地掉头
    -> RobotController.set_direct_drive()
    -> Arduino M1/M2
```

主要代码：

- 启动：`pi_service/run_green_white_scanline_i_console.sh`
- 模式装配：`pi_service/robot_web/app.py`
- 绿底白线和红色识别：
  `pi_service/experiments/i_shape_green_white_turnaround_validation/green_white_scanline_i_logic.py`
- 扫描线、横线和状态机：
  `pi_service/experiments/i_shape_scanline_turnaround_validation/scanline_i_logic.py`
- 实车 PWM：
  `pi_service/robot_web/scanline_i_route.py`
- 直行 PWM 公式：
  `pi_service/experiments/straight_line_stop_validation/straight_motor_control.py`
- 串口电机输出：`pi_service/robot_web/controller.py`

## 4. 三行扫描线与工字端点

### 4.1 当前实现

直线跟踪采样画面底部三行：

```python
track_rows = (0.92, 0.86, 0.80)
```

横向端点使用单独的宽线检测逻辑。宽度超过
`narrow_width_ratio` 的横线不会直接作为直线目标。因此，工字端点横线的设计意图是
“作为端点证据，不参与左右纠偏”。

视觉层允许至少两条窄线扫描结果就形成有效路线：

```python
minimum_track_rows = 2
```

但电机层存在另一条更严格的规则：

```python
if offset is None or valid_bands < 3 or abs(offset) <= deadband:
    return straight_pwm, straight_pwm
```

因此：

- 三行全部有效：允许按偏差差速纠偏；
- 只有两行有效：视觉仍可认为路线有效，但电机强制双轮同速直行；
- 没有中心偏差：同样强制双轮同速直行。

### 4.2 对问题的判断

`valid_bands < 3` 不是门槛太低，而是纠偏门槛太严格。

工字横线进入近场后，可能遮盖、合并或截断纵向白线，使三个窄线采样退化为两行或
更少。此时当前控制不会根据剩余有效偏差纠正方向，反而输出直行 PWM。这会造成：

- 接近端点时继续冲直线；
- 车体已经偏斜时不再修正；
- 端点触发位置严重依赖速度、摩擦和车辆初始姿态。

### 4.3 收敛建议

本轮不引入复杂曲线拟合，先做保守规则：

1. 三行有效：正常 P 纠偏；
2. 两行有效：允许纠偏，但降低基础 PWM，并限制最大修正；
3. 一行有效：视为退化状态，低速或短时保持，不允许正常巡航；
4. 零行有效：进入明确的丢线处理，禁止把 `offset=None` 当作直行。

两行有效时应使用两行中心的中位数，并增加连续帧确认，避免一帧横线碎片引发强转向。

## 5. 骨架与前视点

### 5.1 当前实现

当前代码已经计算骨架、路口、路径和前视点；骨架默认每两帧更新一次。现有直行 PWM
并未使用前视点，只使用底部三行的 `line_center_x`。

### 5.2 本轮决定

骨架不是当前最小修复的必要条件。本轮不把骨架前视点接入核心电机控制，原因是：

- 调试成本较高；
- 骨架毛刺、分支和缓存仍需要额外处理；
- 当前最严重问题可以通过扫描线、红色触发窗口和状态机修复；
- 需要优先得到可解释、可实车标定的行为。

骨架继续保留用于：

- 横线 / 路口辅助检测；
- 调试预览；
- 日志；
- 后续确有需要时再评估是否用于前视控制。

## 6. 普通丢线问题

### 6.1 当前严重缺口

`FOLLOW_STRAIGHT` 没有普通丢线分支。

当前实际链路是：

```text
白线丢失
    -> planner 仍处于 FOLLOW_STRAIGHT
    -> FOLLOW_STRAIGHT 仍属于前进状态
    -> line_center_x 为 None
    -> drive_pwm_for_offset() 把 None 映射为双轮同速
    -> 车辆继续以 straight_pwm 前进
```

这是冲出线路后继续冲、并且无法自动恢复的直接原因，属于必须优先修复的安全问题。

### 6.2 丢线帧数

“丢线 1～2 帧立即决定转弯”不稳定，不应仅凭短暂白线丢失触发掉头。普通丢线与工字
端点掉头必须使用不同证据：

- 普通路线丢线：降速、停止或有限恢复；
- 工字端点掉头：必须由红色标记窗口和白色端点共同授权。

### 6.3 收敛建议

普通 `FOLLOW_STRAIGHT` 中：

1. 一帧丢线：不触发掉头，立即降低或保持很低的前进输出；
2. 连续少量丢线：停车，不允许继续正常直行；
3. 没有红色窗口授权时，任何丢线都不能进入 180° 掉头；
4. 重新看到白线后，需要连续确认再恢复正常巡航。

具体确认帧数和低速 PWM 必须通过录像与实车标定，作为网页可调参数保存。

## 7. 双红色标记定义转弯窗口

### 7.1 目标

不使用“丢线 1～2 帧”直接判断掉头，而在工字端点附近贴两段独立红色标记，用它们的
可见性组合定义一个可标定的转弯区间。

预期原则：

```text
红色标记组合尚未进入标定窗口：只巡线，不允许掉头
进入标定窗口：允许白色横线 / 纵线结束触发停车和掉头
离开标定窗口或组合不符合预期：不允许掉头，执行降速或停车
```

用户提出的核心触发关系是：两段红色中，一段应已经不在视野内，另一段仍在视野内。
这一组合用于表示车辆进入了一个可调整的物理位置范围。

### 7.2 与当前红色检测的区别

当前代码寻找的是“同一高度上、分列白线左右两侧的两个红色碎片”，并将它们合并成
一个红色横带。

用户提出的“两段红色”可能是沿行驶方向分开的两个独立位置标记。二者不是同一个
概念，实施前必须明确贴法：

- 方案 A：两段红色沿行驶方向前后排列，用先后可见性定义窗口；
- 方案 B：每个位置仍由白线左右两块红色组成，共设置前后两个红色横带。

若采用方案 A/B 的前后窗口，检测器需要分别维护 `near_marker` 和 `far_marker`，不能
继续把所有同高度红色碎片只合并成一个 marker。

### 7.3 独立红色 ROI

红色识别应使用独立 ROI，不与白线扫描行完全共用。建议参数化：

- 红色 ROI 的顶部和底部；
- 近红标记允许的 y 范围；
- 远红标记允许的 y 范围；
- 最小红色面积和跨度；
- 连续确认帧数；
- 标记消失确认帧数。

只有红色组合进入窗口后，状态机才设置 `turn_authorized=True`。

红色 ROI 的目的：

- 排除画面远处红色物体；
- 排除教室环境中的红色椅子等干扰；
- 防止红色标记刚进入远端视野就过早授权掉头；
- 将触发位置变成可现场调节的像素范围。

## 8. 收敛后的端点状态机

建议本轮使用以下简单状态，不引入复杂全局定位：

```text
FOLLOW
    正常三行巡线

DEGRADED
    只有一到两行，或短暂丢线；降速，不允许掉头

TURN_WINDOW_ARMED
    双红色可见性组合进入标定窗口；继续低速接近

BRAKE
    红色窗口已授权，白色横线 / 纵向线结束确认；停车

PULSE_PIVOT
    小步进右转

CHECK_ALIGNMENT
    每次步进后停车并重新识别

REACQUIRE
    白线重新进入合适位置，低速对准

FOLLOW
    连续确认后恢复

SAFE_STOP
    超时、红色组合矛盾或一直找不到白线
```

普通丢线绝不能直接跳到 `PULSE_PIVOT`；只有
`TURN_WINDOW_ARMED + 白色端点确认` 才能开始掉头。

## 9. 小步进转向、停车识别

### 9.1 对方案的理解

用户提出的方案是：

```text
短时间转动
    -> 停车
    -> 获取稳定图像并识别
    -> 未对准则继续下一次短转
    -> 已对准则结束掉头
```

这比当前“PWM 200 连续盲转至少 2.5 秒”更适合现有系统，且不需要额外硬件。

### 9.2 建议控制变量

建议将以下参数做成网页可调：

- `pivot_pulse_pwm`
- `pivot_pulse_seconds`
- `pivot_settle_seconds`
- `pivot_max_pulses`
- `reacquire_confirm_frames`
- `reacquire_center_tolerance`
- `reacquire_spread_tolerance`

### 9.3 每次停车后的识别条件

本轮可先只使用现有扫描线，不强制使用骨架：

1. 至少两行或三行窄白线有效；
2. 白线中心位于可调中心范围；
3. 三行中心离散程度小，说明白线大致沿车身纵向；
4. 连续多帧满足；
5. 当前画面不再是宽横线占主导。

满足这些条件后，先低速直行对准，再恢复正常巡航。

### 9.4 注意事项

停车后需要等待短暂稳定时间再识别，否则车身惯性、相机振动和运动模糊会使第一帧
不可靠。转向脉冲也不能过短到电机尚未克服静摩擦，必须通过实车标定最小有效脉冲。

## 10. 本轮暂不采用的复杂方案

为保持实现可解释、适配现有硬件，本轮暂不采用：

- Kalman Filter；
- MPC；
- 完整 Pure Pursuit；
- 鸟瞰二次曲线控制；
- 骨架前视点直接控制电机；
- 新增传感器；
- 重构相机采集链；
- 自动全局定位。

这些方案不是永久否定，而是在基础安全状态机、双红色窗口和步进掉头验证前不优先。

## 11. 必须补充的测试

### 11.1 直行与丢线

- `FOLLOW` 中 `line_center_x=None` 不得输出正常直行 PWM；
- 两行有效时进入降速纠偏，不得无条件双轮 120；
- 一帧丢线不触发掉头；
- 未进入红色窗口时，持续丢线最终必须停车；
- 白线重新出现后需要连续确认再恢复。

### 11.2 红色转弯窗口

- 两段红色均在错误 ROI 时不授权；
- 只有远端红色时不授权；
- 只有符合标定组合时才授权；
- 短暂漏检一帧不改变授权状态；
- 授权必须在离开窗口或完成掉头后清除；
- 红色授权本身不能直接驱动电机掉头，仍需白色端点确认。

### 11.3 脉冲掉头

- 每次只执行一个有上限的转向脉冲；
- 脉冲后必须进入停车识别；
- 未对准时继续下一脉冲；
- 对准连续确认后结束转向；
- 超过最大脉冲数必须 `SAFE_STOP`；
- 看到斜线或偏在画面边缘时不得直接恢复高速巡线。

## 12. 当前优先级

1. 先建立第 15 节的版本化运行日志，使每次失败可追溯；
2. 修复普通 `FOLLOW_STRAIGHT` 丢线仍直行；
3. 拆开 `offset=None`、两行有效和真正居中三种情况；
4. 确定两段红色的实际贴法和可见性组合；
5. 实现独立红色 ROI 与 `TURN_WINDOW_ARMED`；
6. 用脉冲转向 + 停车识别替代连续 2.5 秒盲转；
7. 增加对应单元测试和离线帧序列测试；
8. 实车低速标定红色窗口、转向脉冲和停车稳定时间。

## 13. 待讨论项

在进入实现前，需要最终确定：

1. 两段红色是沿行驶方向前后排列，还是每个位置各由左右两块红色组成；
2. 转弯窗口期望的确切组合是“近标记可见、远标记不可见”，还是相反；
3. 两段红色之间的实际距离；
4. 相机中红色标记从远端进入到离开底部的大致 y 范围；
5. 步进掉头允许的总耗时和场地空间；
6. 掉头过程中始终向右转，还是需要根据场地端点决定方向。

## 14. 两个外部循迹仓库的可借鉴项

参考仓库：

- `CRM-UAM/VisionRace`
- `DexterTaha/WRO-2024-FUTURE-ENGINEERS`

本节结论来自源码核对，不直接采用外部仓库 README 中未经代码实现支持的描述。

### 14.1 CRM-UAM/VisionRace

#### 源码中的实际方法

`line_follow.py` 使用一条水平扫描线：

1. 对灰度图做阈值处理；
2. 对扫描行执行 `np.diff()`；
3. 用一对正负跳变边沿确定线的左右边界；
4. 取两个边沿中点作为横向目标；
5. 用中点偏差直接形成左右轮差速；
6. 当前扫描行找不到边沿时，向上移动扫描行并逐步降低速度；
7. 多次找不到线后退出并停车。

#### 不可直接复制

该仓库最后提交时间较早，`line_follow.py` 本身存在明显问题：

- `tresh` / `thresh` 拼写不一致；
- `img` 未定义；
- 缩进混乱；
- 丢线计数在重获后没有重置；
- 清理函数在调用点之后才定义；
- 只取最先出现的两个边沿，存在多目标误配风险。

因此不能把它当作可直接运行或已经验证稳定的实现。

#### 对当前项目真正有价值的部分

1. **缺失时降速而不是继续正常直行。**

   这直接支持当前评审结论：`offset=None` 不能映射为
   `straight_pwm, straight_pwm`。

2. **在固定底部扫描失败后，向更远处补充扫描。**

   当前项目不需要照搬循环移动单条扫描线，可以在 `DEGRADED` 状态增加若干
   recovery rows。它们只能用于重新寻找纵向窄线，不能直接触发正常高速巡航。

3. **给丢线恢复设置明确次数上限。**

   连续失败计数必须在可靠重获后重置；达到上限后必须 `SAFE_STOP`。

4. **边沿成对验证。**

   当前项目的 `_row_run()` 已经把连续白色区间分组并取得左右边界，本质上比
   VisionRace 的“取前两个跳变点”更完整。因此无需改成 `np.diff()`，只需继续验证
   区间宽度和跨行一致性。

#### 不建议借鉴

- 单条扫描线作为唯一巡线依据；
- 直接选取最前两个边沿；
- 从底部一直循环扫描整个画面；
- 该仓库的直接差速公式和代码结构。

### 14.2 DexterTaha/WRO-2024-FUTURE-ENGINEERS

#### 源码中的实际方法

`src/imageProcessing/utils.py` 的颜色目标检测使用：

1. HSV 独立颜色 mask；
2. 中值滤波；
3. 膨胀和腐蚀；
4. `findContours()` 获得颜色连通域；
5. 用相对画面面积过滤小目标；
6. 用 `boundingRect()` 得到目标中心、宽度和高度；
7. 根据已知目标尺寸和像素高度粗略估算距离；
8. 根据目标横向位置区分目标在左侧还是右侧。

#### 不可直接复制

该仓库使用阿克曼转向、墙线和额外传感器，不是当前差速 I 型白线场景。源码还存在：

- 视觉函数返回值与主循环解包数量不一致；
- 距离参数和像素到厘米换算是固定近似；
- `atan` 转向角公式针对其彩色障碍和阿克曼机构；
- README 提到的 crash handling、Kalman 等没有形成可复用的完整闭环。

因此不能直接移植其转角计算、主循环或恢复逻辑。

#### 对当前项目真正有价值的部分

1. **红色检测输出候选列表，不要只输出一个 best marker。**

   当前项目为了双红色前后窗口，需要为每个红色候选保留：

   ```text
   center_x
   center_y
   width
   height
   area
   span
   ```

   然后按 y 位置排序和跨帧关联，而不是在检测器内过早合并成一个红色结果。

2. **使用相对 ROI 面积过滤。**

   红色最小面积应相对红色 ROI 计算，使分辨率改变时阈值仍有一致含义。

3. **使用目标框高度 / 面积作为接近程度的辅助量。**

   对尺寸固定的红色胶带，越靠近相机，通常像素高度和面积越大。当前项目不应把它
   换算成声称精确的厘米距离，但可以作为：

   - 远标记；
   - 进入窗口；
   - 接近底部；
   - 已经过车；

   的粗粒度证据。

4. **颜色目标与路线检测分层。**

   红色只负责授权转弯窗口；白色横线继续负责端点确认；电机状态机最终决定动作。
   这与当前收敛方案一致。

#### 不建议借鉴

- 直接使用仓库中的 `atan` 结果控制当前差速掉头；
- 把边界框像素尺寸直接当作准确物理距离；
- Hough 墙线导航；
- 阿克曼转向命令；
- README 中没有被当前源码落实的 crash recovery 描述。

### 14.3 合并后适合当前项目的最小改进

两个参考仓库没有提供比当前项目更成熟的完整循迹闭环，但支持以下低复杂度改进：

1. 保留现有三行扫描和 `_row_run()`；
2. 三行退化时增加少量、更高位置的 recovery rows；
3. recovery rows 只允许低速恢复，不允许直接高速巡航或触发掉头；
4. 丢线时逐级降速，并设置连续失败上限；
5. 红色检测返回多个经过面积过滤的候选框；
6. 使用红色候选的 y、框高度和面积做远近分区；
7. 用跨帧历史确认“A 已经过、B 正在指定 ROI”；
8. 红色窗口授权后仍需白色端点确认；
9. 掉头继续采用本项目计划的“脉冲转动 -> 停车 -> 扫描线确认”，不照搬外部仓库。

### 14.4 对当前优先级的影响

外部参考没有改变第 12 节的总体优先级，只补充了两点：

- `DEGRADED` 状态可用额外扫描行扩大有限恢复范围；
- 双红色检测应从“单一最佳红带”改为“候选列表 + 面积/位置/尺寸 + 时序关联”。

其余关键问题仍必须由当前项目自身解决，特别是：

- `FOLLOW_STRAIGHT` 丢线仍直行；
- 两行有效时强制双轮同速；
- 连续 2.5 秒盲转；
- 转弯结束缺少停车对准确认。

## 15. 运行日志与调试规范

### 15.1 目标

每次电机输出必须能够反向追溯到：

1. 同一帧的 OpenCV 视觉证据；
2. 状态机状态、计数器和跳转原因；
3. 当时生效的参数版本；
4. PWM 计算分支和最终下发值；
5. 控制器报告的串口及电机输出状态。

日志采用 UTF-8 JSON Lines。每行是一个完整 JSON 对象，禁止依靠解析自由文本来恢复状态。

### 15.2 公共字段

所有记录必须包含：

```text
schema_version
event
run_id
drive_session_id
time_utc
monotonic_seconds
frame_id
config_revision
```

- `time_utc` 用于人工定位；
- 状态机计时和帧间耗时只使用 `monotonic_seconds`；
- `run_id` 标识进程内的一次视觉运行；
- 每次按 M 开始自动行驶生成新的 `drive_session_id`；
- 网页参数每成功应用一次，`config_revision` 加一。

单位必须进入字段名：

- 像素：`*_px`；
- 秒：`*_seconds`；
- 比例：`*_ratio`；
- PWM：`*_pwm`；
- 帧计数：`*_frames`。

空值语义固定为：

- `null`：本帧无法计算；
- `false`：执行了检测且结果为否；
- `0`：测得或计数确实为零；
- 字段缺失：当前 schema 没有定义该字段。

### 15.3 事件类型

当前必须支持：

- `session_start`：记录模式、画面尺寸、软件版本和完整参数快照；
- `session_end`：记录正常停止或异常结束原因；
- `drive_enabled` / `drive_disabled`：记录 M 门控变化；
- `frame_observation`：逐帧视觉、planner、控制和控制器状态；
- `state_transition`：状态变化时额外记录稳定原因码和证据摘要；
- `tuning_changed`：记录参数旧值、新值、应用帧和新 revision；
- `error`：记录异常类型、消息、traceback、状态和最后帧号。

暂停预览时可降采样为约每秒一条；自动行驶时每个处理帧都必须记录。状态跳转、调参和错误不得采样丢弃。

### 15.4 当前逐帧字段

`frame_observation` 分成四组：

1. `vision`
   - `confidence`、`valid_line`、`line_lost`；
   - `line_center_x_px`、`valid_bands`；
   - 每个有效扫描行的 `y_px`、`row_ratio`、`center_x_px`、`width_px`；
   - endpoint、junction、red band 和 lookahead。
2. `planner`
   - `state`；
   - 稳定枚举 `reason_code` 与说明性 `reason_detail`；
   - endpoint、丢线、重获、junction 帧计数和掉头耗时；
   - 红色底部授权、红色消失帧数及快速丢线授权状态。
3. `control`
   - 控制分支；
   - 归一化偏差、死区、增益；
   - 原始和限幅后的修正 PWM；
   - 基础 PWM、左右轮指令及限幅原因。
4. `controller`
   - 串口是否打开、Arduino 是否在线；
   - 最近接收数据年龄；
   - Arduino 报告的电机输出和错误。

当前 `ScanlineEvidence` 只保留被接受的扫描行。因此日志可以可靠记录“该行有效及其中心/宽度”，但不能推测被拒绝候选的原因。后续修改视觉数据模型时，应为每个配置扫描行增加：

```text
detected
accepted
candidate bounds
reject_reason
```

建议的拒绝原因枚举：

```text
NO_COMPONENT
WIDTH_TOO_NARROW
WIDTH_TOO_WIDE
TRANSVERSE_BAR_DOMINANT
DISCONNECTED_FROM_NEAR_ROUTE
OUTSIDE_ROUTE_CORRIDOR
LOW_COLOR_CONFIDENCE
```

### 15.5 稳定原因码

状态跳转使用固定枚举，例如：

```text
WHITE_BAR_CONFIRMED
EARLY_BAR_PREDICTED
RED_EXIT_CONFIRMED
LINE_LOST_CONFIRMED
REACQUIRE_CONFIRMED
BAR_TIMEOUT
PIVOT_STARTED
PIVOT_LIMIT_REACHED
USER_ENABLED
USER_PAUSED
UNHANDLED_EXCEPTION
```

动态坐标和解释放入 `reason_detail`，禁止把坐标拼接进原因码。

### 15.6 双红色窗口的后续日志契约

实现双红色标记时，检测器必须输出候选列表，而不是只输出合并后的最佳红带。每个候选至少记录：

```text
x_px, y_px, width_px, height_px, area_px
roi_class
accepted
reject_reason
```

状态机另行记录：

```text
near_marker_visible
far_marker_visible
near_confirm_frames
far_missing_frames
turn_window_authorized
authorization_reason
```

未实现双红色候选前，不得在运行日志中伪造这些字段。

### 15.7 脉冲掉头的后续日志契约

每次脉冲必须产生一个 `pivot_cycle` 事件，至少包括：

```text
pulse_index
phase
pulse_pwm
requested_pulse_seconds
actual_command_seconds
settle_seconds
valid_bands
center_offset
center_spread_px
alignment_confirm_frames
alignment_accepted
reject_reason
```

未实现脉冲掉头前，当前连续掉头统一记录为 `control.mode=PIVOT_CONTINUOUS`，不得误称为脉冲控制。

### 15.8 图像证据与人工标记

下一阶段应增加 2～3 秒原始帧环形缓冲，只在以下事件保存前后帧：

- `FOLLOW -> DEGRADED`；
- 普通丢线；
- 红色窗口授权；
- 刹车和每次掉头脉冲；
- 重获成功或失败；
- `SAFE_STOP`；
- 异常；
- 操作者点击“标记问题”。

同时保存原图和标注图，并用 `run_id + frame_id + event` 命名。该功能只依赖已有相机/OpenCV，不要求新增硬件。
