# OpenMV 基地装甲板识别项目计划（M1–M6）

> 目标平台：OpenMV Pro Plus  
> 核心路线：**传统视觉负责候选检测，Tiny CNN 负责基地装甲板判别，时序逻辑负责稳定输出**  
> 最终目标：在实时视频流中稳定识别基地装甲板，降低纯模板匹配对视角、光照、模糊和局部遮挡的敏感性，并尽量保持 **>20 FPS**。

---

## 1. 总体系统架构

整个系统采用“两阶段视觉”结构：

```text
摄像头采集
   ↓
曝光 / 增益控制
   ↓
红蓝灯条提取
   ↓
灯条几何筛选
   ↓
左右灯条配对
   ↓
候选装甲板 ROI
   ↓
ROI 归一化
   ↓
Tiny CNN
   ↓
base / non-base
   ↓
几何分数 + CNN 分数融合
   ↓
连续帧确认 / 目标跟踪
   ↓
输出基地装甲板信息
```

其中：

- **传统视觉**负责“目标大概在哪里”；
- **Tiny CNN**负责“这个候选是不是基地装甲板”；
- **时序逻辑**负责“这个识别结果是否足够稳定，可以输出”。

---

# M1：图像采集与灯条稳定提取

## 对应 Phase

- Phase 0：环境与图像采集
- Phase 1：颜色差分与灯条提取

---

## 目标

获得稳定、低曝光、背景较暗、灯条清晰的图像，并可靠提取红蓝灯条候选区域。

这一阶段暂时不考虑：

- 灯条配对；
- 装甲板识别；
- CNN；
- 时序跟踪。

只解决：

> **“能不能稳定找到灯条。”**

---

## 1.1 摄像头初始化

建议第一版从以下配置开始：

```python
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320 × 240
```

曝光建议起始值：

```text
1 ~ 5 ms
```

例如：

```text
EXPOSURE_US = 3000
GAIN_DB = 3 ~ 6
```

现场调参原则：

- 灯条明显高亮；
- 背景尽量暗；
- 基地装甲内部图案仍保留一定灰度信息；
- 避免灯条大面积过曝扩散。

---

## 1.2 灯条提取方法

优先采用：

### 方法 A：LAB + find_blobs

使用 OpenMV 自带 `find_blobs()` 提取红蓝灯条。

```python
RED_THRESHOLD = (...)
BLUE_THRESHOLD = (...)
```

通过 OpenMV IDE 的阈值编辑器现场标定。

### 方法 B：RGB 通道差分

如果现场红蓝灯条颜色特征明显，可以进一步实验：

```text
RedScore  = R - B
BlueScore = B - R
```

再通过二值化提取灯条。

第一版建议优先完成 LAB 方案，保证开发速度。

---

## 1.3 灯条形态过滤

针对每一个 blob，应用基础几何过滤。

建议初始条件：

```text
最小高度：
h >= 8 px

宽高比：
1.5 < h / w < 15

最小面积：
area >= 15 px
```

后续可以增加：

- solidity；
- elongation；
- rotation；
- 面积占比。

---

## 1.4 输出数据结构

建议灯条统一保存为：

```python
light = {
    "color": "red",
    "x": x,
    "y": y,
    "w": w,
    "h": h,
    "cx": cx,
    "cy": cy,
    "angle": angle,
    "area": area
}
```

---

## M1 验收标准

在典型比赛场景中：

- 1 m；
- 2 m；
- 3 m；
- 4 m；

移动目标并改变角度。

要求：

```text
真实灯条召回率 > 95%
```

允许存在少量误检。

原则：

> **M1 宁可多检，不要漏检。**

---

# M2：灯条几何配对与候选装甲板生成

## 对应 Phase

- Phase 2：灯条几何配对
- Phase 5 中的几何评分部分

---

## 目标

从灯条集合中找出可能属于同一装甲板的左右灯条。

输入：

```text
light_list
```

输出：

```text
armor_candidate_list
```

---

## 2.1 灯条两两组合

基本结构：

```python
for i in range(len(lights)):
    for j in range(i + 1, len(lights)):
        left = lights[i]
        right = lights[j]
```

随后执行几何过滤。

---

## 2.2 几何约束

建议第一版至少使用以下条件。

### 高度比例

```text
max(h1, h2) / min(h1, h2) < 1.4
```

后续可以逐渐收紧到：

```text
< 1.3 ~ 1.35
```

---

### 角度差

```text
abs(angle1 - angle2) < 10°
```

后续现场稳定后可尝试：

```text
< 6° ~ 8°
```

---

### 垂直中心差

定义：

```text
dy = abs(cy1 - cy2)
avg_h = (h1 + h2) / 2
```

要求：

```text
dy / avg_h < 0.5
```

后期可收紧到：

```text
< 0.3
```

---

### 水平间距

定义：

```text
dx = abs(cx1 - cx2)
```

第一版：

```text
1.0 < dx / avg_h < 6.0
```

该参数需要根据基地装甲尺寸和现场距离调整。

---

### 左右关系

必须保证：

```text
left.cx < right.cx
```

并建议增加：

```text
水平距离 > 垂直距离
```

降低上下灯条错误配对概率。

---

## 2.3 Geometry Score

不要只使用硬阈值。

建议为每个候选建立几何评分：

```python
geometry_score = (
    0.25 * height_score +
    0.25 * angle_score +
    0.20 * vertical_score +
    0.30 * distance_score
)
```

范围：

```text
0.0 ~ 1.0
```

建议第一版：

```python
MIN_GEOMETRY_SCORE = 0.55
```

只要：

```text
geometry_score >= 0.55
```

即可进入下一阶段。

---

## 2.4 候选结构

```python
candidate = {
    "color": "red",
    "left_light": left,
    "right_light": right,
    "geometry_score": 0.82,
    "center": (cx, cy)
}
```

---

## M2 验收标准

典型场景下：

- 正常装甲应形成正确灯条对；
- 单灯不得生成装甲；
- 大部分明显错误组合应被排除；
- 允许少量错误候选进入 CNN。

目标：

```text
真实装甲候选召回率 > 95%
```

这一阶段仍然以召回率优先。

---

# M3：ROI 归一化与训练数据采集

## 对应 Phase

- Phase 3：ROI 提取与校正
- Phase 4：数据采集部分

---

## 目标

从灯条对生成稳定的装甲板 ROI，并建立真实运行条件下的训练数据集。

这一阶段的核心不是 CNN，而是：

> **保证 CNN 以后看到的输入足够一致。**

---

## 3.1 ROI 内容

推荐 ROI 包含：

```text
左灯条 + 中间图案 + 右灯条
```

不要只截中间数字 / 图案。

原因：

CNN 可以利用：

- 灯条相对位置；
- 灯条长度；
- 灯条角度；
- 灯条间距；
- 图案结构；
- 图案与灯条之间的位置关系。

---

## 3.2 ROI 裁剪

第一版建议：

```text
候选灯条整体包围框
+
左右扩展约 10%
+
上下扩展约 15% ~ 20%
```

例如：

```python
x1 = left.x
x2 = right.x + right.w

y1 = min(left.y, right.y)
y2 = max(left.y + left.h, right.y + right.h)
```

再加 margin。

---

## 3.3 是否旋转校正

第一版建议：

> **暂时不进行复杂旋转校正。**

原因：

- 简化 OpenMV 运算；
- 减少图像拷贝；
- Tiny CNN 可以通过数据增强学习 ±10° ~ ±15° 小角度变化。

如果后期实际发现：

```text
大倾角明显降低 CNN 准确率
```

再加入：

```text
rotation_corr()
```

或者更完整的透视校正。

---

## 3.4 CNN 输入尺寸

推荐第一版：

```text
48 × 32 × 1
```

即：

```text
48×32 灰度图
```

优点：

- 保留装甲板横向比例；
- 比 48×48 计算量低；
- 灰度输入减少 RAM 和模型计算量；
- 红蓝颜色已经由传统视觉负责判断。

如果 FPS 不足，可进一步尝试：

```text
40 × 28
32 × 24
32 × 32
```

---

## 3.5 Dataset Mode

在 M3 阶段加入专门的数据采集模式：

```text
摄像头
 ↓
检测灯条
 ↓
灯条配对
 ↓
生成 ROI
 ↓
resize
 ↓
保存图片
```

数据目录：

```text
/dataset/
    base/
    non_base/
```

---

## 3.6 数据集规模

建议初始目标：

```text
base:
500 ~ 1000

non_base:
1000 ~ 2000
```

更理想：

```text
base:
2000+

non_base:
3000+
```

---

## 3.7 Non-base 样本组成

不要只保存普通背景。

真正重要的负样本包括：

```text
普通装甲板
错误灯条配对
其他机器人 LED
高亮反光
不完整装甲
远距离装甲
运动模糊装甲
严重倾斜目标
曝光异常目标
只有部分基地进入 ROI
```

最重要原则：

> **程序实际误检过什么，就把什么加入 non-base 数据集。**

---

## M3 验收标准

要求：

- ROI 中装甲板整体结构基本完整；
- 灯条检测轻微抖动不会导致 ROI 大幅变化；
- ROI 尺寸始终固定；
- 自动采集数据能够正常保存；
- 数据集包含不同距离、角度、亮度和运动状态。

---

# M4：Tiny CNN 训练与离线验证

## 对应 Phase

- Phase 4：中间图案识别
- 删除模板匹配方案
- Tiny CNN 成为唯一主要分类器

---

## 目标

训练一个轻量二分类模型：

```text
base
non-base
```

不再使用模板匹配。

---

## 4.1 推荐网络结构

建议第一版使用小型自定义 CNN：

```text
Input
48 × 32 × 1

↓

Conv 3×3
8 channels
ReLU

↓

MaxPool 2×2

↓

Conv 3×3
16 channels
ReLU

↓

MaxPool 2×2

↓

Conv 3×3
32 channels
ReLU

↓

MaxPool 2×2

↓

Global Average Pooling

↓

Dense 2

↓

Softmax
```

输出：

```text
P(base)
P(non_base)
```

---

## 4.2 模型目标

建议：

```text
INT8 quantization
```

目标大小：

```text
< 150 KB
```

理想：

```text
30 ~ 100 KB
```

---

## 4.3 数据增强

训练时建议加入：

```text
旋转：
-15° ~ +15°

亮度：
0.6 ~ 1.4

对比度变化

平移：
±10%

缩放：
0.8 ~ 1.2

轻微透视变化

Gaussian Noise

Motion Blur

Random Crop

Random Occlusion
```

---

## 4.4 ROI Jitter

重点加入 ROI 偏移增强。

模拟实际灯条定位误差：

```text
±2 px
±3 px
±5 px
```

保证 CNN 不依赖“完美居中”。

---

## 4.5 训练指标

不能只看 Accuracy。

重点关注：

```text
Base Precision
Base Recall
False Positive Rate
```

目标建议：

```text
Validation Accuracy > 95%

Base Recall > 95%

Base Precision > 97%
```

其中基地识别系统尤其要关注：

```text
False Positive
```

因为错误锁定基地往往比偶尔漏一帧更危险。

---

## 4.6 测试集要求

测试集必须来自：

> **完全未参与训练的数据和视频。**

建议单独采集：

- 不同光照；
- 不同距离；
- 不同角度；
- 快速移动；
- 部分遮挡；
- 极端曝光。

---

## M4 验收标准

离线测试满足：

```text
Accuracy > 95%
Precision(base) > 97%
Recall(base) > 95%
```

且真实误检样本能够明显被模型区分。

---

# M5：CNN 部署、评分融合与实时输出

## 对应 Phase

- Phase 4：模型部署
- Phase 5：融合与输出

---

## 目标

将 Tiny CNN 部署到 OpenMV，并与传统视觉几何结果融合。

---

## 5.1 推理原则

CNN 只对：

```text
通过灯条几何筛选的候选 ROI
```

进行推理。

不要整帧运行 CNN。

流程：

```text
find_blobs
 ↓
没有候选？
 ↓
跳过 CNN

存在候选？
 ↓
生成 ROI
 ↓
CNN inference
```

---

## 5.2 CNN Score

CNN 输出：

```python
cnn_score = P(base)
```

例如：

```text
P(base) = 0.91
```

---

## 5.3 综合评分

第一版建议：

```python
final_score = (
    0.20 * geometry_score +
    0.80 * cnn_score
)
```

CNN 作为主要判别依据。

后续也可调整为：

```text
0.3 geometry + 0.7 CNN
```

---

## 5.4 初始阈值

建议：

```python
CNN_THRESHOLD = 0.80
FINAL_THRESHOLD = 0.80
```

第一版宁可略严格。

---

## 5.5 数据结构

统一使用：

```python
armor_info = {
    "detected": True,
    "color": "red",
    "type": "base",

    "center": (cx, cy),

    "geometry_score": 0.82,
    "cnn_score": 0.93,
    "score": 0.91,

    "roi": (x, y, w, h)
}
```

---

## 5.6 串口输出

建议后期输出简化格式：

```text
BASE,R,162,108,62,34,0.91
```

字段：

```text
TYPE
COLOR
CX
CY
W
H
SCORE
```

---

## M5 验收标准

实时运行中：

- 基地候选能正确分类；
- 普通装甲误识别率明显下降；
- CNN 不对整图运行；
- 稳态 FPS 尽量达到：

```text
> 20 FPS
```

---

# M6：时序跟踪、现场标定与性能优化

## 对应 Phase

- Phase 5：连续帧确认
- Phase 6：现场标定与优化

---

## 目标

降低单帧误判、减少识别闪烁，并完成比赛现场参数标定。

---

## 6.1 连续帧确认

不要单帧：

```text
score > threshold
```

就立即输出。

建议基础策略：

```text
连续 3 帧识别为基地
```

才确认目标。

---

## 6.2 分数平滑

建议使用指数平滑：

```python
smooth_score = (
    0.7 * old_score +
    0.3 * current_score
)
```

或者：

```python
ALPHA = 0.35
```

根据实际响应速度调整。

---

## 6.3 Hysteresis 滞回

建议设置：

```python
BASE_ENTER_THRESHOLD = 0.82
BASE_KEEP_THRESHOLD = 0.62
```

逻辑：

```text
未锁定目标：

连续 3 帧
score > 0.82

→ 确认基地
```

已锁定：

```text
score > 0.62

→ 保持
```

只有连续若干帧低于阈值才解除。

这样可以避免：

```text
base
non-base
base
non-base
```

快速跳变。

---

## 6.4 目标跟踪

可以通过中心位置进行简单匹配：

```text
当前候选中心
与
上一帧目标中心
```

距离小于某阈值：

```text
认为是同一个目标
```

后期可加入：

- ROI IoU；
- 中心距离；
- 颜色一致性；
- 尺寸变化限制。

---

## 6.5 参数集中管理

所有可调参数统一放在程序顶部：

```python
# Camera
EXPOSURE_US = 3000
GAIN_DB = 4

# Blob
MIN_LIGHT_HEIGHT = 8
MIN_LIGHT_AREA = 15

# Pairing
MAX_HEIGHT_RATIO = 1.4
MAX_ANGLE_DIFF = 10
MAX_VERTICAL_RATIO = 0.5

# CNN
CNN_THRESHOLD = 0.80

# Tracking
BASE_ENTER_THRESHOLD = 0.82
BASE_KEEP_THRESHOLD = 0.62
CONFIRM_FRAMES = 3
```

方便现场快速修改。

---

## 6.6 性能优化

建议按优先级逐项优化。

### 1. 限制 blob 数量

不要让大量噪声进入配对。

---

### 2. 只有候选出现时运行 CNN

这是最重要的优化之一。

---

### 3. 缩小搜索区域

如果装甲板通常只出现在画面中心区域，可使用：

```python
sensor.set_windowing(...)
```

降低图像处理范围。

---

### 4. 降低 CNN 输入尺寸

如果 FPS 不足：

```text
48×32
 ↓
40×28
 ↓
32×24
```

重新评估准确率与速度。

---

### 5. 减少图像复制

尽量复用 ROI / framebuffer。

避免循环中频繁创建大对象。

---

## 6.7 现场标定流程

现场推荐固定执行顺序：

### Step 1

调曝光与增益。

目标：

```text
灯条清楚
背景偏暗
内部图案可见
```

### Step 2

调 LAB 阈值。

确保灯条召回稳定。

### Step 3

调灯条几何参数。

重点观察：

```text
height ratio
angle diff
vertical diff
distance ratio
```

### Step 4

观察 ROI。

确保基地结构没有被裁掉。

### Step 5

观察 CNN Score。

统计：

```text
真实基地分数
普通装甲分数
错误灯条组合分数
```

### Step 6

调最终阈值和连续帧策略。

---

## M6 验收标准

最终系统需要达到：

```text
基地识别稳定
误锁概率低
连续移动时不明显闪烁
不同距离下工作正常
不同光照下参数可快速调整
```

性能目标：

```text
> 20 FPS
```

如果不能达到，优先优化：

```text
ROI 数量
CNN 输入尺寸
find_blobs 搜索区域
候选上限
```

---

# 2. M1–M6 与 Phase 对应关系

| Milestone | 对应 Phase | 主要任务 |
|---|---|---|
| M1 | Phase 0 + Phase 1 | 图像采集、曝光控制、红蓝灯条提取 |
| M2 | Phase 2 + Phase 5 部分 | 灯条配对、几何过滤、Geometry Score |
| M3 | Phase 3 + Phase 4 数据采集 | ROI 生成、归一化、采集训练数据 |
| M4 | Phase 4 | Tiny CNN 训练与离线验证 |
| M5 | Phase 4 部署 + Phase 5 | CNN 推理、分数融合、数据输出 |
| M6 | Phase 5 + Phase 6 | 连续帧确认、跟踪、标定、性能优化 |

---

# 3. 推荐开发顺序

严格按照：

```text
M1
 ↓
M2
 ↓
M3
 ↓
M4
 ↓
M5
 ↓
M6
```

不要同时开发所有模块。

推荐调试方式：

```text
M1：
画灯条框

M2：
画灯条 + 装甲候选框

M3：
显示 / 保存 CNN ROI

M4：
PC / Edge Impulse 完成模型

M5：
OpenMV 显示 CNN score

M6：
关闭大部分 debug 绘图并测试 FPS
```

---

# 4. 第一版参数基线

| 参数 | 建议初始值 |
|---|---|
| 主图分辨率 | 320×240 RGB565 |
| Exposure | 3000 us |
| Gain | 3~6 dB |
| 最小灯条高度 | 8 px |
| 最小面积 | 15 px |
| 灯条高度比 | < 1.4 |
| 灯条角度差 | < 10° |
| 垂直中心差 | < 0.5 × avg_h |
| ROI margin X | 10% |
| ROI margin Y | 15~20% |
| CNN 输入 | 48×32×1 |
| CNN 类别 | base / non-base |
| CNN Channels | 8 → 16 → 32 |
| 模型 | INT8 TFLite |
| Geometry threshold | 0.55 |
| CNN threshold | 0.80 |
| Base enter | 0.82 |
| Base keep | 0.62 |
| 连续确认 | 3 frames |
| 目标帧率 | >20 FPS |

---

# 5. 推荐最终代码结构

```python
# =========================
# CONFIG
# =========================

CAMERA_CONFIG = ...
COLOR_THRESHOLDS = ...
LIGHT_CONFIG = ...
PAIR_CONFIG = ...
CNN_CONFIG = ...
TRACK_CONFIG = ...


# =========================
# CAMERA
# =========================

def init_camera():
    pass


# =========================
# LIGHT DETECTION
# =========================

def detect_lights(img):
    pass


# =========================
# LIGHT PAIRING
# =========================

def pair_lights(lights):
    pass


def calc_geometry_score(left, right):
    pass


# =========================
# ROI
# =========================

def make_armor_roi(img, left, right):
    pass


# =========================
# CNN
# =========================

def classify_base(roi):
    pass


# =========================
# FUSION
# =========================

def calc_final_score(geometry_score, cnn_score):
    pass


# =========================
# TRACKER
# =========================

def update_tracker(candidate):
    pass


# =========================
# OUTPUT
# =========================

def output_target(target):
    pass


# =========================
# MAIN
# =========================

while True:

    img = sensor.snapshot()

    lights = detect_lights(img)

    pairs = pair_lights(lights)

    candidates = []

    for pair in pairs:

        roi = make_armor_roi(
            img,
            pair["left"],
            pair["right"]
        )

        cnn_score = classify_base(roi)

        final_score = calc_final_score(
            pair["geometry_score"],
            cnn_score
        )

        if final_score >= FINAL_THRESHOLD:
            candidates.append(...)

    target = update_tracker(candidates)

    if target:
        output_target(target)
```

---

# 6. 项目最终成功标准

系统最终应满足以下条件：

### 检测层

```text
真实灯条召回率 > 95%
```

### CNN 层

```text
Validation Accuracy > 95%
Base Recall > 95%
Base Precision > 97%
```

### 系统层

```text
基地可稳定框选
普通装甲误锁率低
连续运动无明显闪烁
不同光照条件可现场快速标定
实时 FPS > 20
```

---

# 7. 核心设计原则

整个项目始终坚持：

> **传统视觉负责“在哪里”，Tiny CNN 负责“是不是”，时序算法负责“是否稳定”。**

对于只需要识别基地装甲板的任务，这种架构相比模板匹配更加适合现场环境，也比直接进行整图 CNN 目标检测更适合 OpenMV 的嵌入式算力条件。
