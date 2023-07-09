# GAMES104笔记


## 引擎架构 {#引擎架构}


### 平台层 Platform {#平台层-platform}

提供操作系统/平台相关的底层功能。


#### 文件系统 file_service {#文件系统-file-service}


#### 路径 path {#路径-path}


### 核心层 Core {#核心层-core}

提供软件系统常用模块。


#### 基础库 base（宏、哈希） {#基础库-base-宏-哈希}


#### 色彩 color {#色彩-color}


#### 数学库 math {#数学库-math}


#### 元数据系统 meta {#元数据系统-meta}

<!--list-separator-->

-  反射 reflection

<!--list-separator-->

-  序列化/反序列化 serializer


#### 日志系统 log {#日志系统-log}


### 资源层 resource {#资源层-resource}

提供资产加载、保存功能，资产的结构化数据定义和相关路径配置等。


#### 资产系统 asset_manager {#资产系统-asset-manager}


#### 配置系统 config_manager {#配置系统-config-manager}


#### 结构化数据定义 res_type {#结构化数据定义-res-type}

<!--list-separator-->

-  全局数据 global

    <!--list-separator-->

    -  全局粒子设置 global_particle

    <!--list-separator-->

    -  全局渲染配置 global_rendering

<!--list-separator-->

-  通用数据 common

    <!--list-separator-->

    -  世界 world

    <!--list-separator-->

    -  关卡 level

    <!--list-separator-->

    -  对象 object

<!--list-separator-->

-  组件数据 components

    <!--list-separator-->

    -  动画 animation

    <!--list-separator-->

    -  相机 camera

    <!--list-separator-->

    -  粒子发射器 emitter

    <!--list-separator-->

    -  网格 mesh

    <!--list-separator-->

    -  运动 motor

    <!--list-separator-->

    -  刚体 rigid_body

<!--list-separator-->

-  其他数据 data

    <!--list-separator-->

    -  动画片短 animation_clip

    <!--list-separator-->

    -  动画骨骼节点 animation_skeleton_node_map

    <!--list-separator-->

    -  基本形状 basic_shape

    <!--list-separator-->

    -  动画混合状态 blend_state

    <!--list-separator-->

    -  相机配置 camera_config

    <!--list-separator-->

    -  材质 material

    <!--list-separator-->

    -  网格数据 mesh_data

    <!--list-separator-->

    -  骨骼 skeleton_data

    <!--list-separator-->

    -  骨骼掩膜 skeleton_mask


### 功能层 function {#功能层-function}

提供引擎功能模块。分为框架和子系统两部分。


#### 框架 framework {#框架-framework}

运行时功能和新框架。核心框架采用世界\`world\`-关卡\`level\`-GameObject\`obejct\`-组建\`component\`的层级架构。
世界管理器 world_manager 负责管理世界的加载、卸载、保存，和tick下属当前关卡。 关卡 level 负责加载、卸载、保存关卡。同时关卡也管理下属GO的tick、创建和删除。 游戏对象 object 负责加载、保存GO。同时GO也管理下属组件。
组件包括：

<!--list-separator-->

-  动画 animation

<!--list-separator-->

-  相机 camera

<!--list-separator-->

-  网格 mesh

<!--list-separator-->

-  运动 motor

<!--list-separator-->

-  粒子 particle

<!--list-separator-->

-  刚体 rigidbody

<!--list-separator-->

-  变换 transform


#### 动画 animation {#动画-animation}


#### 角色 character {#角色-character}


#### 控制器 controller {#控制器-controller}


#### 全局上下文 global {#全局上下文-global}


#### 输入 input {#输入-input}


#### 粒子 particle {#粒子-particle}


#### 物理 physics {#物理-physics}


#### 渲染 render {#渲染-render}


#### UI ui {#ui-ui}


## 渲染（Rendering) {#渲染-rendering}


### 实践 {#实践}


#### GPU Batch Rendering {#gpu-batch-rendering}

合批处理，集中管理资源，使用索引获取目标资源


#### 可见性裁剪 {#可见性裁剪}

用包围盒判断是否在事件锥里

-   包围球 (sphere)
-   AABB 矩形盒（对角线长等于物体长度）
-   OBB 矩形盒（高度等于物体长度）
-   8-DOP
-   CONVEX HULL（凸包）

<!--list-separator-->

-  用BVH的方法进行性能优化

<!--list-separator-->

-  PVS(Potential Visibility Set) 简单的可见区域估计（也可用于资源加载）

<!--list-separator-->

-  GPU Culling 让显卡来算

    做深度测试，先绘制深度图，再绘制其他，有看不见的就跳过


#### Texture Compression （纹理压缩） {#texture-compression-纹理压缩}

<!--list-separator-->

-  Block Compression

    ASTC算法（Mobile）
    ， BC7算法（PC）


#### Authoring Tools of Modeling {#authoring-tools-of-modeling}

<!--list-separator-->

-  Polymodeling

    -   BLENDER
    -   MAX
    -   MAYA

<!--list-separator-->

-  Sculpting （雕刻性的工具）(z-brush)

<!--list-separator-->

-  Scanning（实体扫描）

<!--list-separator-->

-  Procedural Modeling

    -   Houdini
    -   Unreal


#### Cluster-Based Mesh Pipeline （现代） {#cluster-based-mesh-pipeline-现代}

<!--list-separator-->

-  GPU-Driven Rendering Pipeline(2015)

<!--list-separator-->

-  Geometry Rendering Pipeline Architecture(2021) （一个完整的物体被拆分成多个小集群）

    -   Nanite


### 绘制 {#绘制}


#### Lighting {#lighting}

The Rendering Equation (BRDF)

<!--list-separator-->

-  Simple light + Ambient

<!--list-separator-->

-  Blinn-Phong Materials

    = Ambient + Diffuse + Specular（光可叠加原理）
    cons: 能量不守恒，震荡被放大，做什么都像塑料

<!--list-separator-->

-  Shadow

    Shadow Map
    cons: 取决于采样率

<!--list-separator-->

-  Pre-computed Global Illumination

    空间换时间，预计算全局光照。
    全局光照 = 直接光照和间接光照都有。
    用Fourier Transform（傅里叶变换）来处理。
    Spherical Harmonics（球谐函数）。

    SH Lightmap: 预先计算光照贴图
    （预计算时间长，但是运行时很高效，能烘焙很多很好的细节）（前景不好）

    Light Probe:＋ Reflection Probe 放足够多的采样点，计算目标最近的采样点的光照，再用插值计算


#### Material {#material}

<!--list-separator-->

-  Physical-Based Material (PBR)

    -   Microfacet Theory （微平面理论）
    -   GGX模型
    -   Fresnel Equation
    -   Disney Principled BRDF

    <!--list-separator-->

    -  SG模型(Specular Glossiness)

        -   Diffuse
        -   Specular
        -   Glossiness

    <!--list-separator-->

    -  MR模型(Metallic Roughness)

        -   Base Color
        -   Roughness
        -   Metallic

<!--list-separator-->

-  Image-Based Lighting (IBL)

    -   Diffuse Irradiance Map
    -   Specular Approximation

<!--list-separator-->

-  Classic Shadow Solution

    -   Cascade Shadow
    -   PCF - Percentage Closer Filter
        -   PCSS - Percentage Closer Soft Shadow（软阴影）
    -   VSSM - Variance Soft Shadow Map
    -   Virtual Shadow Maps


#### Shader {#shader}

Uber Shader and Vaiants


### 地形中大气和云的渲染 {#地形中大气和云的渲染}

-   Sky and Cloud
-   Vegetation
-   Terrain


#### Simple Idea - Heightfield {#simple-idea-heightfield}

<!--list-separator-->

-  Adaptive Mesh Tessellation

    -   Triangle-Based Subdivision
    -   QuadTree-Based Subdivision (more popular)
        cons: T-Junctions（格子间会有空隙）

<!--list-separator-->

-  Triangulated Irregular Network

    需要预处理

<!--list-separator-->

-  GPU-Based Tessellation

    -   Mesh Shader Pipeline

    real-time deformable terrain


#### Non-Heightfield Terrain {#non-heightfield-terrain}


#### Volumetric Representation (Crazy idea) {#volumetric-representation--crazy-idea}

-   Marching Cubes


#### Paint Terrain Materials {#paint-terrain-materials}

-   Parallax and Displacement Mapping
-   Virtual Texture
    -   DirectStorage &amp; DMA (硬盘到显存，跳过cpu、内存)
-   Floating-point Precision Error (when map is too big) (IEEE 754 float double)
    -   Camera-Relative Rendering

<!--list-separator-->

-  Tree Rendering

<!--list-separator-->

-  Decorator Rendering

<!--list-separator-->

-  Road and Decals Rendering


#### Atmosphere and Cloud {#atmosphere-and-cloud}

<!--list-separator-->

-  Atmoshpere

    -   Analytic Atmosphere Appearance Modeling

    Participating Media （有关介质）
    Radiative Transfer Equation
    Volume Rendering Equation(VRE)

    -   Scattering Types
        -   Rayleigh Scattering
        -   Mie scattering
    -   Variant Air Molecules Absorption
    -   Single Scattering vs Multi Scattering （单次 vs 多次散射）
    -   Ray Marching
        -   Precomputed Atmospheric Scattering
    -   Production Friendly Quick Sky and Atmosphere Rendering

<!--list-separator-->

-  Cloud

    -   Mesh-Based Cloud Modeling
    -   Billboard Cloud
    -   Volumetric Cloud Modeling
        -   Weather Texture
        -   Noise Functions
            -   Perlin Noise
            -   Worley Noise
        -   Cloud Density Model


### 渲染管线,后处理及其他 {#渲染管线-后处理及其他}


#### Ambient Occlusion （环境光遮蔽） {#ambient-occlusion-环境光遮蔽}

<!--list-separator-->

-  Screen Space Ambient Occlusion (SSAO), SSAO+

<!--list-separator-->

-  HBAO - Horizon-based Ambient Occlusion

<!--list-separator-->

-  GTAO - Ground Truth-based Ambient Occlusion

<!--list-separator-->

-  Ray-Tracing Ambient Occlusion


#### Fog Everything {#fog-everything}

<!--list-separator-->

-  Depth Fog

<!--list-separator-->

-  Height Fog

<!--list-separator-->

-  Voxel-based Volumetric Fog


#### Anti-aliasing（抗锯齿） {#anti-aliasing-抗锯齿}

多采样几个，然后计算加权。问题一般出现在边缘上。

-   Super-sample AA (SSAA) and Mutli-sample AA (MSAA)
-   FXAA(Fast Approximate Anti-aliasing)
    -   用边缘检测来寻找需要更多采样的地方
-   TAA(Temporal Anti-aliasing) 时序上用前一帧的数据


#### Post-process {#post-process}

<!--list-separator-->

-  Bloom Effect (光晕效果)

    -   Detect Bright Area by Threshold
    -   Gaussian Blur
    -   Pyramid Gaussian Blur

<!--list-separator-->

-  Tone Mapping （色调映射）解决曝光问题

    -   Tone Mapping Curve 拟合出来的
    -   ACES 奥斯卡搞的

<!--list-separator-->

-  Color Grading

    -   Lookup Table (LUT)（色彩分级）


#### Rendering Pipeline {#rendering-pipeline}

<!--list-separator-->

-  Forward Rendering

    透明物最后绘制，绘制多个透明物时，由远及近地绘制

<!--list-separator-->

-  Deferred Rendering （延迟渲染）

<!--list-separator-->

-  Tile-based Rendering

    -   Light Culling by Tiles
    -   Depth Range Optimization

<!--list-separator-->

-  Tile-based Deferred Rendering

<!--list-separator-->

-  Forward+（Tile-based Forward） Rendering

<!--list-separator-->

-  Cluster-based Rendering (椎体)

<!--list-separator-->

-  Visibility Buffer

    -   G-Buffer
    -   V-Buffer 当几何数多于像素数时，用V-Buffer好，把几何和材质分离。

<!--list-separator-->

-  Frame Graph

    use DAG(Directed Acyclic Graph) to check the rightness of every module in the
    project

<!--list-separator-->

-  Render to Monitor

    -   Screen Tearing
        -   use V-Sync to solve （垂直同步）
        -   Variable Refresh Rate （可变刷新率）


## 动画 (Animation) {#动画--animation}


### Basics of Animation Technology {#basics-of-animation-technology}

Hand Draw Animation -&gt; Cel Animation -&gt; Computer Animation


#### 2D Animation {#2d-animation}

<!--list-separator-->

-  Sprite animation

<!--list-separator-->

-  Live2D

    set animation "key frame"


#### 3D Animation {#3d-animation}

Dof(Degrees of Freedom)（自由度）

<!--list-separator-->

-  Rigid Hierarchical Animation

<!--list-separator-->

-  Per-vertex Animatioin

    离线渲染，然后作为材质载入

<!--list-separator-->

-  Morph Target Animation

<!--list-separator-->

-  Skinned Animation

<!--list-separator-->

-  Physics-based Animation

    Ragdoll 布娃娃系统
    Cloth and Fluid
    Inverse Kinematics(IK)

<!--list-separator-->

-  Animation Content Creation

    -   Digital Content Creator + Animator
    -   Motion Capture （动捕）


#### Skinned Animation Implementation {#skinned-animation-implementation}

存骨骼关节数据，其他顶点跟着关节动
蒙皮动画(主流)

<!--list-separator-->

-  How to Animate a Mesh

<!--list-separator-->

-  Different Spaces

    -   local space
    -   model space
    -   world space

<!--list-separator-->

-  Skeleton for Creatures

    -   Non-humanoid Skeleton
    -   Humanoid Skeleton

    <!--list-separator-->

    -  Joint vs. Bone （关节 和 骨骼）

        一般指的都是关节
        Joints for gameplay (eg. weapon)
        mount

    <!--list-separator-->

    -  Root Joint

    <!--list-separator-->

    -  Bind Animation for Objects

        bind point

    <!--list-separator-->

    -  Bind Pose - T-pose vs. \*A-pose

    <!--list-separator-->

    -  Skeleton Pose

        <!--list-separator-->

        -  Joint Pose (9 DoFs)

            -   Orientation (3 DoFs) 旋转
            -   Position (3 DoFs) 平移
            -   Scale (3 DoFs) 放缩

<!--list-separator-->

-  Math of 3D Rotation

    <!--list-separator-->

    -  2D Orientation Math

    <!--list-separator-->

    -  3D Orientation Math

        <!--list-separator-->

        -  Euler Angle

            !Order Dependence
            case: Gimbal Lock 万向结、陀螺仪、云台

            <!--list-separator-->

            -  Yaw 航向偏转

            <!--list-separator-->

            -  Pitch 俯仰

            <!--list-separator-->

            -  Roll 翻转

            <!--list-separator-->

            -  Problems: 退化（当沿着Y轴旋转90度后，x、z共轴）

        <!--list-separator-->

        -  Quaternion （四元数）

        <!--list-separator-->

        -  Joint Pose

            <!--list-separator-->

            -  Affine Matrix

            <!--list-separator-->

            -  Local space to Model space

            <!--list-separator-->

            -  Skinning Matrix

                （目标点根据绑定的关节变化进行变化）

            <!--list-separator-->

            -  Skinning Matrix Palette

                预计算完一个模型所有的蒙皮矩阵后存成表

            <!--list-separator-->

            -  Weighted Skinning with Multi-joints

                顶点一般最多绑定4个关节
                加权蒙皮矩阵
                Weighted Skinning Blend 发生在model space

    <!--list-separator-->

    -  Clip

        <!--list-separator-->

        -  Quaternion Interpolation (NLERP)

            Shortest path fixing of NLERP
            cons: 角度变化速度不稳定，首尾快，中间慢

        <!--list-separator-->

        -  SLERP: Uniform Rotation Interpolation

<!--list-separator-->

-  Simple Animation Runtime Pipeline

    [Updated Animation Pipeline with Blending and IK](#updated-animation-pipeline-with-blending-and-ik)
    Clip + time -&gt; previous frame - current frame -&gt; local current pose -&gt; model current pose -&gt; skinning matrix palette -&gt; mesh vertex position


#### Animation Compression {#animation-compression}

clip storage
animation data size
Distinction among Animation Tracks
Distinction among Joints

<!--list-separator-->

-  Simplest Compression - DoF Reduction

    <!--list-separator-->

    -  Keyframe 计算关键帧，其余帧用插值计算

        Catmull-Rom Spline 用于旋转插值

    <!--list-separator-->

    -  Float Quantization

    <!--list-separator-->

    -  Quaternion Quantization

    <!--list-separator-->

    -  Error Propagation 错误扩散

        Joint Sensitivity to Error
        Measuring Accuracy  - Data Error

        -   Visual Error

        Error Compensation - In Place Correction


#### Animation DCC {#animation-dcc}

<!--list-separator-->

-  Mesh

<!--list-separator-->

-  Skeleton binding

<!--list-separator-->

-  Skinning

<!--list-separator-->

-  Animation creation

<!--list-separator-->

-  Exporting

    跳跃时，动画和位移曲线应该分开存


### Advanced Animation Technology {#advanced-animation-technology}


#### Animation Blending {#animation-blending}

means that allows more one animation clip to contribute to the final pose of the character

<!--list-separator-->

-  Math of Blending: LERP 做线性插值

    在clip之间插值

    <!--list-separator-->

    -  Calculate Blend Weight

    <!--list-separator-->

    -  Align Blend Timeline

<!--list-separator-->

-  Blend Space

    <!--list-separator-->

    -  1D Blend Space: Directional Movement

    <!--list-separator-->

    -  Directional Walking and Running

    <!--list-separator-->

    -  2D Blend Space

<!--list-separator-->

-  Skeleton Masked Blending 分上下半身和该动作结合

<!--list-separator-->

-  Additive Blending

    只存动画的变化量，不存绝对量，然后应用到任何clip上
    可能产生Abnormal Bone Results

<!--list-separator-->

-  Action State Machine

    -   state
    -   transition: when and how

    <!--list-separator-->

    -  Cross Fades

        <!--list-separator-->

        -  Smooth transition

        <!--list-separator-->

        -  Frozen transition

    <!--list-separator-->

    -  Cross Fades Curve

    <!--list-separator-->

    -  Layered ASM

        模型的不同部分分开控制

    <!--list-separator-->

    -  Animation Blend Tree

        <!--list-separator-->

        -  LERP Blend Node

        <!--list-separator-->

        -  Additive Blend Node

        <!--list-separator-->

        -  Blend Tree Nodes


#### Inverse Kinematics {#inverse-kinematics}

<!--list-separator-->

-  Basic Concepts

    <!--list-separator-->

    -  End-effector （末端效果器）

    <!--list-separator-->

    -  IK(Inverse Kinematics) （反向运动学）

    <!--list-separator-->

    -  FK(Forward Kinematics)

<!--list-separator-->

-  Two Bones IK

<!--list-separator-->

-  Complexity of Multi-Joint IK Solving

    <!--list-separator-->

    -  Check Reachability of the Target

    <!--list-separator-->

    -  Constraints of Joints

        <!--list-separator-->

        -  Heuristics Algorithm

            <!--list-separator-->

            -  CCD (Cyclic Coordinate Decent)

                Optimized: add tolerance region
                use under-damped angle

            <!--list-separator-->

            -  FABRIK (Forward and Backward Reaching Inverse Kinematics)

<!--list-separator-->

-  Multiple End-Effectors

    <!--list-separator-->

    -  IK with Multiple End-Effectors

    <!--list-separator-->

    -  Jacobian Matrix

<!--list-separator-->

-  Other IK Solutions

    <!--list-separator-->

    -  Physics-based Method

    <!--list-separator-->

    -  PBD(Position Basesd Method)

    <!--list-separator-->

    -  Fullbody IK in UE5

<!--list-separator-->

-  IK is still Chanllenge

<!--list-separator-->

-  Updated Animation Pipeline with Blending and IK

    [Simple Animation Runtime Pipeline](#simple-animation-runtime-pipeline)
    Clip + time -&gt; Blend Pose -&gt; Model Pose -&gt; World Pose -&gt; IK -&gt; World Current Pose -&gt; skinning matrix palette -&gt; mesh vertex position


#### Animation Pipeline {#animation-pipeline}


#### Animation Graph {#animation-graph}


#### Facial Animation {#facial-animation}

<!--list-separator-->

-  Facial Action Coding System

<!--list-separator-->

-  Action Units Combination

    <!--list-separator-->

    -  28 core units

    <!--list-separator-->

    -  Key Pose Blending

    <!--list-separator-->

    -  FACS in Morph Target Animation

<!--list-separator-->

-  Complex Facial Skeleton

<!--list-separator-->

-  UV Texture Facial Animation

<!--list-separator-->

-  Muscle Model Animation

<!--list-separator-->

-  Metahuman


#### Retargeting {#retargeting}

<!--list-separator-->

-  Share Animation among Characters

<!--list-separator-->

-  Terminology

    <!--list-separator-->

    -  Source Character

    <!--list-separator-->

    -  Target Character

    <!--list-separator-->

    -  Source Aniamtion

    <!--list-separator-->

    -  Target Animation

<!--list-separator-->

-  Ignore Offset Between Source and Target Joints

<!--list-separator-->

-  Keep Orientation in Different Binding Pose

    存相对变化而非绝对变化

<!--list-separator-->

-  Process Tracks

<!--list-separator-->

-  Align Movement by Pelvis Height

<!--list-separator-->

-  Lock Feet by IK after Retargeting

    离线做比较好

<!--list-separator-->

-  Retargeting with Different Skeleton Hierarchy

    都归一化，再重新互相对应

<!--list-separator-->

-  Unresolved Problems of Retargeting

    <!--list-separator-->

    -  self mesh penetration

    <!--list-separator-->

    -  self contact constrains

<!--list-separator-->

-  Morph Animation Retargeting


## 物理系统 (Physics System) {#物理系统--physics-system}


### Basic Concepts {#basic-concepts}


#### Physics Actors and Shapes {#physics-actors-and-shapes}

<!--list-separator-->

-  Actors

    <!--list-separator-->

    -  static

    <!--list-separator-->

    -  dynamic

    <!--list-separator-->

    -  triggers

    <!--list-separator-->

    -  kinematics

<!--list-separator-->

-  Actor Shapes

    <!--list-separator-->

    -  Spheres

    <!--list-separator-->

    -  Capsules

    <!--list-separator-->

    -  Boxes

    <!--list-separator-->

    -  Convex Meshes

    <!--list-separator-->

    -  Triangle Meshes

        only static actors

    <!--list-separator-->

    -  Height Fields

<!--list-separator-->

-  Shape Properties

    <!--list-separator-->

    -  Mass

        Gomboc Shape

    <!--list-separator-->

    -  Density

    <!--list-separator-->

    -  Center of Mass

    <!--list-separator-->

    -  Friction &amp; Restitution （摩擦力和弹性）


#### Forces {#forces}

<!--list-separator-->

-  Gravity

<!--list-separator-->

-  Drag

<!--list-separator-->

-  Friction

<!--list-separator-->

-  Impulse ...


#### Movements {#movements}

<!--list-separator-->

-  Explicit(Forward) Euler's Method

    过去决定现在
    取决于delta_t的精度，不收敛

<!--list-separator-->

-  Implicit(Backward) Euler's Method

    现在决定过去
    衰减的

<!--list-separator-->

-  Semi-implicit Euler's Method

    显式和隐式结合一下。速度用显式的，距离用隐式的
    稳定

<!--list-separator-->

-  Particle Dynamics （质点动力学，都没考虑形状、旋转）


#### Rigid Body Dynamics {#rigid-body-dynamics}

除了质点动力学的性质，还有:

<!--list-separator-->

-  Orientation

<!--list-separator-->

-  Angular velocity

<!--list-separator-->

-  Angular acceleration

<!--list-separator-->

-  Inertia tensor

    Rotational Inertia （转动惯量）

<!--list-separator-->

-  Angular momentum （角动量）

<!--list-separator-->

-  Torque （力矩）


#### Collision Resolution {#collision-resolution}

<!--list-separator-->

-  Two Phases

    <!--list-separator-->

    -  Broad phase

        <!--list-separator-->

        -  Objectives

            <!--list-separator-->

            -  find intersected rigid body AABBs

            <!--list-separator-->

            -  Potential overlapped rigid body pairs

        <!--list-separator-->

        -  Methods

            <!--list-separator-->

            -  BVH

            <!--list-separator-->

            -  Sort and Sweep

    <!--list-separator-->

    -  Narrow phase

        <!--list-separator-->

        -  Objectives

            <!--list-separator-->

            -  Detect overlapping precisely

            <!--list-separator-->

            -  Gererate contact information

        <!--list-separator-->

        -  Methods

            <!--list-separator-->

            -  Basic Shape Intersection Test

            <!--list-separator-->

            -  Minkowski Difference-based Methods

                <!--list-separator-->

                -  GJK Algorithm

            <!--list-separator-->

            -  Separating Axis Theorem （分离轴定理）

<!--list-separator-->

-  Resolution

    <!--list-separator-->

    -  Applying Penalty Force

    <!--list-separator-->

    -  Solving constrainsk 约束求解


#### Scene Query {#scene-query}

<!--list-separator-->

-  Raycast

<!--list-separator-->

-  Sweep

<!--list-separator-->

-  Overlap


### Efficiency, Accuracy, and Determinism {#efficiency-accuracy-and-determinism}

<!--list-separator-->

-  Collision Group

    可碰撞的物品放一个组里存

<!--list-separator-->

-  Simulation Optimization

    -   Island -&gt; sleep

    把物理世界分成一个个island，不活动的就让它sleep

<!--list-separator-->

-  Continuous Collision Detection

    运动速度过快时，发生类似隧穿效应的现象。在检测碰撞前，Actor已经穿过了障碍物。

<!--list-separator-->

-  Deterministic Simulation

    多人游戏中，同样的错误应当对全体玩家产生同样的效果。确定的输入，应当产生确定的结果。
    浮点型运算会有影响
    same old states + same inputs = same new states
    如果能做到确定性模拟，多人游戏可以不同步服务器状态，只同步输入。


### Applications {#applications}


#### Character Controller {#character-controller}

Kinematic Actor

-   Shape: Capsule（两层，一层保护，一层检测碰撞

<!--list-separator-->

-  Collide with environment

<!--list-separator-->

-  Auto Stepping and its Problem

<!--list-separator-->

-  Slope Limits and Force Sliding Down

<!--list-separator-->

-  Controller Volume Update

<!--list-separator-->

-  Controller Push Objects

    <!--list-separator-->

    -  Problem case: Standing on Moving Platform


#### Ragdoll {#ragdoll}

<!--list-separator-->

-  Map Skeleton to Rigid Bodies

<!--list-separator-->

-  Human Joint Constraint

    <!--list-separator-->

    -  Constraints

        Properties

<!--list-separator-->

-  Animating Skeleton by Ragdoll

<!--list-separator-->

-  Blending between Animation and Ragdoll

<!--list-separator-->

-  Powered Ragdol - Physics-Animation Blending


#### Cloth {#cloth}

<!--list-separator-->

-  Animation-based Cloth Simulation

<!--list-separator-->

-  Rigid Body-based Cloth Simulation

<!--list-separator-->

-  Mesh-based Cloth Simulation

    <!--list-separator-->

    -  Render Mesh vs. Physical Mesh

    <!--list-separator-->

    -  Paint Cloth Simulation Constraints

    <!--list-separator-->

    -  Set Cloth Physical Material

    <!--list-separator-->

    -  Cloth Solver - Mass-Spring System

        <!--list-separator-->

        -  Verlet Integration （半隐式欧拉积分的变式）

    <!--list-separator-->

    -  Cloth Solver - Position Based Dynamics (PBD) 约束求解

    <!--list-separator-->

    -  Self Collision

        <!--list-separator-->

        -  make the cloth thicker

        <!--list-separator-->

        -  use many substeps

        <!--list-separator-->

        -  enforce maximal velocity

        <!--list-separator-->

        -  introduce contact constraints and friction constraints


#### Destruction {#destruction}

<!--list-separator-->

-  Chunk Hierarchy

<!--list-separator-->

-  Connectivity Graph

    <!--list-separator-->

    -  Connectivity Value

    <!--list-separator-->

    -  Damage Calculation

<!--list-separator-->

-  Destruction with / without Support Graph

<!--list-separator-->

-  Build Chunks by Voronoi Diagram

    <!--list-separator-->

    -  Fracturing with Voronoi Diagram Mesh

<!--list-separator-->

-  Different Fracture Patterns with Voronoi Diagram

<!--list-separator-->

-  Destruction in Physics System

    handle destruction after collision

<!--list-separator-->

-  Make it realistic

    提供一些回调函数

<!--list-separator-->

-  Popular Destruction

    <!--list-separator-->

    -  NVIDIA APEX Destruction

    <!--list-separator-->

    -  NVIDIA Blast

    <!--list-separator-->

    -  Havok Destruction

    <!--list-separator-->

    -  Chaos Destruction


#### Vehicle {#vehicle}

<!--list-separator-->

-  Vehicle Implementation Spectrum

    Stylized -&gt; Realistic

<!--list-separator-->

-  Vehicle Mechanism Modeling

    <!--list-separator-->

    -  Traction Force （牵引力）

    <!--list-separator-->

    -  Suspension Force （悬挂力）

    <!--list-separator-->

    -  Tire Forces （轮胎力）

    <!--list-separator-->

    -  Center of Mass

    <!--list-separator-->

    -  Weight Transfer

    <!--list-separator-->

    -  Steering angles

        <!--list-separator-->

        -  Ackermann steering

    <!--list-separator-->

    -  Advanced Wheel Contact

        <!--list-separator-->

        -  Single Raycast

        <!--list-separator-->

        -  Spherecast


#### Advanced Physics: PBD/XPBD {#advanced-physics-pbd-xpbd}

<!--list-separator-->

-  Lagrange

    <!--list-separator-->

    -  Circling Constraint

    <!--list-separator-->

    -  String Constraint

<!--list-separator-->

-  PBD - Constraints Projection

    Position Based Dynamics

<!--list-separator-->

-  XPBD - Extended Position Based Dynamics

    Compliance matrix 服从度矩阵


### <span class="org-todo done DONE">DONE</span> homework3 animation system and physics system {#homework3-animation-system-and-physics-system}


## 粒子和声效系统 Effects {#粒子和声效系统-effects}


### Particle System {#particle-system}


#### Particle's Life Cycle {#particle-s-life-cycle}

spawn -&gt; aging -&gt; reaction -&gt; death


#### Particle Emitter {#particle-emitter}

a particle system is a collection of individual emitters


#### Particle Spawn Position {#particle-spawn-position}


#### Particle Spawn Mode {#particle-spawn-mode}


#### Simulate {#simulate}

<!--list-separator-->

-  Gravity

<!--list-separator-->

-  Viscous Drag

<!--list-separator-->

-  Wind Fields

<!--list-separator-->

-  Collision


#### Particle Type {#particle-type}

<!--list-separator-->

-  Billboard Particle

    always face the camera, appear to 3d

<!--list-separator-->

-  Mesh Particle

<!--list-separator-->

-  Ribbon Particle

    滞留感, Centripetal Catmull-Rom interpolation


#### Particle System Rendering {#particle-system-rendering}

<!--list-separator-->

-  Alpha Blending Order

<!--list-separator-->

-  Particle Sort

    <!--list-separator-->

    -  Sorting mode

        <!--list-separator-->

        -  Global

            accurate, but expensive

        <!--list-separator-->

        -  Hierarchy

            per system - per emitter - within emitter

    <!--list-separator-->

    -  Sort rules

        <!--list-separator-->

        -  Between particles: based on particle distance with camera

        <!--list-separator-->

        -  Between systems or emitters: bounding box

<!--list-separator-->

-  Full-Resolution Particles

    Costy, Worst case as particles fill the screen

    <!--list-separator-->

    -  Low-Resolution Particles

        Downsampling (scene depth) -&gt; half-res depth -&gt; particle color &amp; particle alpha -&gt; Bilateral upsampling (both scene color and scene depth) -&gt; final image


#### GPU Particles {#gpu-particles}

<!--list-separator-->

-  Initial State

    -   Particle Pool
    -   Dead List
    -   Alive List

<!--list-separator-->

-  Spawn Particles

<!--list-separator-->

-  Simulate

    view frustum culling

<!--list-separator-->

-  Sort, Render and Swap Alive Lists

    <!--list-separator-->

    -  Parallel Mergesort

<!--list-separator-->

-  Depth Buffer Collision


#### Advanced Particles {#advanced-particles}

<!--list-separator-->

-  Crowd Simulation

    <!--list-separator-->

    -  Animated Particle Mesh

    <!--list-separator-->

    -  Particle Animation Texture

    <!--list-separator-->

    -  Navigation Texture

        Signed Distance Field, Direction Texture (RG channels)

    <!--list-separator-->

    -  Runtime Behavior

<!--list-separator-->

-  Skeleton Mesh emitter

<!--list-separator-->

-  Dynamic Procedural Splines

<!--list-separator-->

-  Interacting with environment

<!--list-separator-->

-  ...


#### Design Philosophy {#design-philosophy}

<!--list-separator-->

-  Preset Stack-Style Modules

<!--list-separator-->

-  Graph-Based Design

<!--list-separator-->

-  Hybrid Design

    Unreal's Niagara System Design


### Sound System {#sound-system}


#### Audio {#audio}

<!--list-separator-->

-  Volume

    <!--list-separator-->

    -  Sound Pressure （分贝）

    <!--list-separator-->

    -  Particle Velocity

    <!--list-separator-->

    -  Sound Intensity

<!--list-separator-->

-  Pitch 音高

<!--list-separator-->

-  Timbre 音色

<!--list-separator-->

-  Phase and Noise Cancelling

<!--list-separator-->

-  Human Hearing Characteristic


#### Digital Sound {#digital-sound}

<!--list-separator-->

-  Pulse-code Modulation (PCM)

    <!--list-separator-->

    -  Sampling

        Nyquist-Shannon Sampling Theorem

    <!--list-separator-->

    -  Quantizing

        bit-depth

    <!--list-separator-->

    -  Encoding

        audio format
        游戏行业OGG格式用的多，无版权问题


#### 3D Audio Rendering {#3d-audio-rendering}

<!--list-separator-->

-  3D Sound Sources

    <!--list-separator-->

    -  Listener

        a virtual microphone

        -   position
        -   velocity
        -   orientation

    <!--list-separator-->

    -  Spatialization

        <!--list-separator-->

        -  Panning

        <!--list-separator-->

        -  Soundfield

            Full-sphere
            used in 360 videos and VR

        <!--list-separator-->

        -  Binaural Audio

    <!--list-separator-->

    -  Attenuation （衰弱）制造空间感

        <!--list-separator-->

        -  Attenuation Shape

            <!--list-separator-->

            -  Sphere

            <!--list-separator-->

            -  Capsule

            <!--list-separator-->

            -  Box

            <!--list-separator-->

            -  Cone

    <!--list-separator-->

    -  Obstruction and Occlusion

    <!--list-separator-->

    -  Reverb （混响）

        -   Direct(dry)
        -   Early reflections(echo)
        -   Late reverberations(tail)

        <!--list-separator-->

        -  Reverberation Time

        <!--list-separator-->

        -  Absorption

        <!--list-separator-->

        -  Reverb in Action - Reverb Effect Control from Acoustic Parameters

            <!--list-separator-->

            -  Pre-delay

            <!--list-separator-->

            -  HF ratio

            <!--list-separator-->

            -  Wet level

            <!--list-separator-->

            -  Dry level

    <!--list-separator-->

    -  Sound in Motion: The Doppler Effect


#### Common Middlewares {#common-middlewares}

<!--list-separator-->

-  fMod

<!--list-separator-->

-  WWise


## 引擎 (Tool Chains) {#引擎--tool-chains}


### Foundation of Tool Chains {#foundation-of-tool-chains}


#### What is Game Engine Tool Chains {#what-is-game-engine-tool-chains}

适应各工种特性、调和各工种矛盾，使之能一起工作


#### Complicated Tool GUI {#complicated-tool-gui}

-   fast iteration
-   separation of design and implementation
-   reusability
-   ...

<!--list-separator-->

-  Immediate Mode

<!--list-separator-->

-  Retained Mode\*

<!--list-separator-->

-  Design Pattern

    <!--list-separator-->

    -  MVC (model - view - controller)

    <!--list-separator-->

    -  MVP (model - view - presenter)

    <!--list-separator-->

    -  MVVM (model - view - viewmodel)


#### How to Load Asset - Desesrialization {#how-to-load-asset-desesrialization}

<!--list-separator-->

-  Serialization and Deserialization

    <!--list-separator-->

    -  Text Files 先用text，方便debug，再改成json或者别的。

    <!--list-separator-->

    -  Binary Files (eg. FBX)

<!--list-separator-->

-  Asset Reference

    <!--list-separator-->

    -  Asset Data Repeatance

    <!--list-separator-->

    -  Object Instance in Scene

        <!--list-separator-->

        -  Object Instance Variance

            <!--list-separator-->

            -  Build Variance by Copying

            <!--list-separator-->

            -  Build Variance by Data Inheritance

<!--list-separator-->

-  Parse Asset File

    <!--list-separator-->

    -  Build Key-Type-Value Pair Tree

    <!--list-separator-->

    -  Binary vs. Text

    <!--list-separator-->

    -  Endianness （字节序）

<!--list-separator-->

-  Asset Version Compatibility

    <!--list-separator-->

    -  Version Hardcode (add version code)

    <!--list-separator-->

    -  Field UID (Google Protocol Buffers)


#### How to Make a Robust Tools {#how-to-make-a-robust-tools}

<!--list-separator-->

-  !!!Undo &amp; Redo

<!--list-separator-->

-  Crash Recovery

<!--list-separator-->

-  Command

    abstract operations to atomic commands which can invoke, revoke and serialize, deserialize

    <!--list-separator-->

    -  Definition

        <!--list-separator-->

        -  uid

        <!--list-separator-->

        -  data

        <!--list-separator-->

        -  Invoke()

        <!--list-separator-->

        -  Revoke()

        <!--list-separator-->

        -  Serialize() 由data提供

        <!--list-separator-->

        -  Deserialize() 由data提供

<!--list-separator-->

-  Three Key Commands

    <!--list-separator-->

    -  Add

    <!--list-separator-->

    -  Delete

    <!--list-separator-->

    -  Update


#### How to Make Tool Chain {#how-to-make-tool-chain}

<!--list-separator-->

-  Find Common Building Blocks

<!--list-separator-->

-  Schema

    A Description Structure

    <!--list-separator-->

    -  Basic Elements

    <!--list-separator-->

    -  Inheritance

    <!--list-separator-->

    -  Data Reference

    <!--list-separator-->

    -  2 Definition Ways

        <!--list-separator-->

        -  standalone schema definition file

        <!--list-separator-->

        -  defined in code

            Reflection in Piccolo Engine

    <!--list-separator-->

    -  Three Views for Engine Data

        <!--list-separator-->

        -  Runtime View

        <!--list-separator-->

        -  Storage View

        <!--list-separator-->

        -  Tools View

            <!--list-separator-->

            -  Various Editor Modes


#### What You See is What You Get (WYSIWYG) {#what-you-see-is-what-you-get--wysiwyg}

<!--list-separator-->

-  Stand-alone Tools

<!--list-separator-->

-  \*In Game Tools

    <!--list-separator-->

    -  Editor Mode

    <!--list-separator-->

    -  Play in Editor(PIE)

        <!--list-separator-->

        -  Play in Editor World

        <!--list-separator-->

        -  Play in PIE World: duplicate editor world to create a PIE world and play in it


#### One More Thing - Plugin {#one-more-thing-plugin}

<!--list-separator-->

-  Extensibility

<!--list-separator-->

-  Framework

    <!--list-separator-->

    -  PluginManager

    <!--list-separator-->

    -  Interface

    <!--list-separator-->

    -  API

<!--list-separator-->

-  toolbar, tool menu


### Applications &amp; Advanced Topic {#applications-and-advanced-topic}


#### Glance of Game Production {#glance-of-game-production}


#### Architecture of A World Editor {#architecture-of-a-world-editor}

<!--list-separator-->

-  Editor Viewport

    warning: editor-only code must be moved out of released game

<!--list-separator-->

-  Everything is an Editable Object

<!--list-separator-->

-  Different Views of Objects

<!--list-separator-->

-  Schema-driven Object Property Editing

<!--list-separator-->

-  Content Browser

<!--list-separator-->

-  Editing Utilities in World Editor

    <!--list-separator-->

    -  Mouse Picking

        <!--list-separator-->

        -  Ray Casting

        <!--list-separator-->

        -  RTT

    <!--list-separator-->

    -  Object Transform Editing

    <!--list-separator-->

    -  Terrain

    <!--list-separator-->

    -  Height Brush

    <!--list-separator-->

    -  Instance Brush

    <!--list-separator-->

    -  Environment

        sky, light, roads, rivers ...

    <!--list-separator-->

    -  Rule System in Environment Editing


#### Plugin Architecture {#plugin-architecture}

<!--list-separator-->

-  Any system and object type could be plug-ins to Editors

<!--list-separator-->

-  Combination of Multiple Plugins

    <!--list-separator-->

    -  Covered

    <!--list-separator-->

    -  Distributed

    <!--list-separator-->

    -  Pipeline

    <!--list-separator-->

    -  Onion rings

<!--list-separator-->

-  One More Thing - Version Control


#### Design Narrative Tools {#design-narrative-tools}

<!--list-separator-->

-  Sotrytelling in Game Engine

    <!--list-separator-->

    -  Sequencer

        <!--list-separator-->

        -  Track

        <!--list-separator-->

        -  Propertry Track

        <!--list-separator-->

        -  Timeline

        <!--list-separator-->

        -  Key Frame

        <!--list-separator-->

        -  Sequence


#### Reflection and Gameplay {#reflection-and-gameplay}

<!--list-separator-->

-  Reflection is Foundation of Sequencer

<!--list-separator-->

-  Visual Scripting System

<!--list-separator-->

-  A Common Solution - Reflection

    <!--list-separator-->

    -  Reflection Build the Bridge between Code and Tools

    <!--list-separator-->

    -  How to Implement Reflection in C++

        <!--list-separator-->

        -  AST

        <!--list-separator-->

        -  Generate Schema from AST

        <!--list-separator-->

        -  Precise Control of Reflection Scope

        <!--list-separator-->

        -  Use Marco to Add Reflection Controls

    <!--list-separator-->

    -  Reflection Accessors

    <!--list-separator-->

    -  Code Rendering

        <!--list-separator-->

        -  use Mustache for code generation


#### Collaborative Editing {#collaborative-editing}

<!--list-separator-->

-  Bottlenecks in Large Projects

<!--list-separator-->

-  Merging Conflicts

    <!--list-separator-->

    -  Split Assets

        <!--list-separator-->

        -  Layering the World (cons: dependence)

        <!--list-separator-->

        -  Divide the World (cons: cross-boundary)

        <!--list-separator-->

        -  One File Per Actor

<!--list-separator-->

-  Coordinate Editing in One Space

    <!--list-separator-->

    -  Synchronize Operations with Others

        how to ensure the consistency of distributed operations?

        <!--list-separator-->

        -  eg. Instance lock

        <!--list-separator-->

        -  eg. Asset lock

    <!--list-separator-->

    -  How to Solve these problems thoroughly

        <!--list-separator-->

        -  Operation Transform (OT)

        <!--list-separator-->

        -  Conflict-free Replicated Data Type(CRDT)


### <span class="org-todo done DONE">DONE</span> homework4 toolchain {#homework4-toolchain}


## 玩法 (Gameplay) {#玩法--gameplay}


### Gameplay Complexity and Building Blocs {#gameplay-complexity-and-building-blocs}


#### Overview {#overview}

<!--list-separator-->

-  Challenges

    <!--list-separator-->

    -  Cooperation among multiple systems

    <!--list-separator-->

    -  Diversity of game play in the same game

    <!--list-separator-->

    -  Rapid iteration


#### Event/Message Mechanism {#event-message-mechanism}

<!--list-separator-->

-  Publish-subscribe Pattern

    <!--list-separator-->

    -  Event Definition

        should be editable (as extension)

        <!--list-separator-->

        -  type

        <!--list-separator-->

        -  argument

    <!--list-separator-->

    -  Callback Registration

        <!--list-separator-->

        -  Object Lifespan and Callback Safety

        <!--list-separator-->

        -  Object Strong Reference

        <!--list-separator-->

        -  Object Weak Reference

    <!--list-separator-->

    -  Event Despatching

        <!--list-separator-->

        -  Immediate

        <!--list-separator-->

        -  Event Queue

            <!--list-separator-->

            -  Event Serializing and Deserializing

            <!--list-separator-->

            -  Ring buffer

            <!--list-separator-->

            -  Batching

            <!--list-separator-->

            -  cons: Timeline not determined by publisher, one-frame delays


#### Script System {#script-system}

<!--list-separator-->

-  Problem of Compiled Languages

    <!--list-separator-->

    -  hard for hot-update

<!--list-separator-->

-  Pros of Scripting languages

    <!--list-separator-->

    -  support for rapid iteration

    <!--list-separator-->

    -  easy to learn and write

    <!--list-separator-->

    -  support for hot update

    <!--list-separator-->

    -  stable, less crash by running in a sandbox

<!--list-separator-->

-  How Script Languages Work

    <!--list-separator-->

    -  script text -&gt; bytecode -&gt; run on VM

<!--list-separator-->

-  Object Management between Scripts and Engine

    <!--list-separator-->

    -  engine manage

    <!--list-separator-->

    -  script manage

<!--list-separator-->

-  Architectures for Scripting System

    <!--list-separator-->

    -  Native language dominants the game world

    <!--list-separator-->

    -  Script language dominants the game world

<!--list-separator-->

-  Advanced Script Features

    <!--list-separator-->

    -  Hot Update

<!--list-separator-->

-  Popular Script Languages

    <!--list-separator-->

    -  Lua

        <!--list-separator-->

        -  Robust and mature

        <!--list-separator-->

        -  Excellent runtime performance

        <!--list-separator-->

        -  Light-weighted and highly extensible

    <!--list-separator-->

    -  Python

        <!--list-separator-->

        -  Reflection support

        <!--list-separator-->

        -  Built-in object-oriented support

        <!--list-separator-->

        -  Extensive standard libraries and third-party modules

    <!--list-separator-->

    -  C#

        <!--list-separator-->

        -  Low learning curve

        <!--list-separator-->

        -  Built-in object-oriented support

        <!--list-separator-->

        -  Great community


#### Visual Scripting {#visual-scripting}

<!--list-separator-->

-  Variable Visualization - Data Pin and Wire

<!--list-separator-->

-  Statement and Expression Visualization - Node

<!--list-separator-->

-  Control Flow Visualization - execution Pin and Wire

<!--list-separator-->

-  Function Visualization - Function Graph

<!--list-separator-->

-  Class Visualization - Blueprint

    <!--list-separator-->

    -  Make Graph User Friendly

        <!--list-separator-->

        -  Fuzzy finding

        <!--list-separator-->

        -  Accurate suggestions by type

<!--list-separator-->

-  Visual Script Debugger

<!--list-separator-->

-  Issues

    <!--list-separator-->

    -  Hard to merge for a team work

        <!--list-separator-->

        -  visual script is stored as a binary file

        <!--list-separator-->

        -  manually reorder script graph is inefficient and error-prone

        <!--list-separator-->

        -  graph can get pretty messy with complex logic


#### Script and Graph are Twins {#script-and-graph-are-twins}


#### Character, Control and Camera {#character-control-and-camera}

<!--list-separator-->

-  Character

    <!--list-separator-->

    -  Movement

        <!--list-separator-->

        -  Movement State Machine

    <!--list-separator-->

    -  more complex and varied states

    <!--list-separator-->

    -  cooperate with other systems

    <!--list-separator-->

    -  more realistic motion with Physics

<!--list-separator-->

-  Control

    <!--list-separator-->

    -  Different input device

    <!--list-separator-->

    -  Different game play

    <!--list-separator-->

    -  case: shoot

        <!--list-separator-->

        -  Zoom in and out

        <!--list-separator-->

        -  Aim Assist

    <!--list-separator-->

    -  Feedback

    <!--list-separator-->

    -  Context Awareness

        <!--list-separator-->

        -  Context-sensitive controls

    <!--list-separator-->

    -  Chord &amp; Key Sequences

    <!--list-separator-->

    -  Subjective Feelings

<!--list-separator-->

-  Camera

    <!--list-separator-->

    -  Basic: POV &amp; FOV

    <!--list-separator-->

    -  Camera Control

        <!--list-separator-->

        -  Spring Arm

        <!--list-separator-->

        -  Focuing FOV &amp; distance Curve

    <!--list-separator-->

    -  Camera Track

    <!--list-separator-->

    -  Camera Effects

        <!--list-separator-->

        -  Camera Shake

        <!--list-separator-->

        -  Camera Filter

    <!--list-separator-->

    -  Camera Manager

        <!--list-separator-->

        -  Camera Switch

    <!--list-separator-->

    -  Subjective Feelings

        <!--list-separator-->

        -  loose feeling

        <!--list-separator-->

        -  Cinematic


#### Everything is Gameplay ... {#everything-is-gameplay-dot-dot-dot}


### Basic Artificial Intelligence {#basic-artificial-intelligence}


#### Navigation {#navigation}

<!--list-separator-->

-  Navigation Steps

    <!--list-separator-->

    -  Map Representation

        <!--list-separator-->

        -  Walkable Area

        <!--list-separator-->

        -  Formats

            <!--list-separator-->

            -  Waypoint Network

                <!--list-separator-->

                -  先走到路网上，再沿着路网走。（像地铁网一样，先到站点，再自动寻路）

            <!--list-separator-->

            -  Grid

                <!--list-separator-->

                -  3D不太好做

            <!--list-separator-->

            -  Navigation Mesh

                <!--list-separator-->

                -  must use Convex Polygon, not concave polygon

                <!--list-separator-->

                -  Not support 3D space (直升飞机之类)

                <!--list-separator-->

                -  NavMesh Generation

                    <!--list-separator-->

                    -  Voxelization

                        <!--list-separator-->

                        -  Region Segmentation

                            <!--list-separator-->

                            -  Watershed Algorithm

                <!--list-separator-->

                -  Advanced Features

                    <!--list-separator-->

                    -  Polygon Flag

                    <!--list-separator-->

                    -  Tile

                    <!--list-separator-->

                    -  Off-mesh Link

            <!--list-separator-->

            -  Sparse Voxel Octree

                <!--list-separator-->

                -  Represents flyable 3D space

                <!--list-separator-->

                -  Similar to spatial partitioning

                <!--list-separator-->

                -  八叉树

    <!--list-separator-->

    -  Path Finding

        <!--list-separator-->

        -  can be abstracted as shortest path problem in non-directional graph

            <!--list-separator-->

            -  DFS

            <!--list-separator-->

            -  BFS

            <!--list-separator-->

            -  Dijkstra Algorithm

            <!--list-separator-->

            -  A Star

    <!--list-separator-->

    -  Path Smoothing

        <!--list-separator-->

        -  String Pulling (Funnel Algorithm)


#### Steering {#steering}

From Path to Motion

<!--list-separator-->

-  Steering Behaviors

    <!--list-separator-->

    -  Seek/Flee

        <!--list-separator-->

        -  Pusue

        <!--list-separator-->

        -  Wander

        <!--list-separator-->

        -  Path Following

        <!--list-separator-->

        -  Flow Field Following

    <!--list-separator-->

    -  Velocity Match

        <!--list-separator-->

        -  Arrive

    <!--list-separator-->

    -  Align


#### Crowd Simulation {#crowd-simulation}

<!--list-separator-->

-  Crowd Simulation Models

    <!--list-separator-->

    -  Microscopic models

        <!--list-separator-->

        -  Rule-based models

    <!--list-separator-->

    -  Macroscopic models

    <!--list-separator-->

    -  Mesoscopic models

        <!--list-separator-->

        -  RTS游戏中的小兵

<!--list-separator-->

-  Collision Avoidance

    <!--list-separator-->

    -  Force-based Models

    <!--list-separator-->

    -  Velocity-based Models

        <!--list-separator-->

        -  Velocity Obstacle (VO)

        <!--list-separator-->

        -  Reciprocal Velocity Obstacle (RVO)


#### Sensing (Perception) {#sensing--perception}

<!--list-separator-->

-  Information

    <!--list-separator-->

    -  Internal Information

    <!--list-separator-->

    -  External Information

        <!--list-separator-->

        -  Static Spatial Information

        <!--list-separator-->

        -  Dynamic Spatial Information

            <!--list-separator-->

            -  Influence Map

            <!--list-separator-->

            -  Game Objects

<!--list-separator-->

-  Sensing Simulation


#### Classic Decision Making Algorithms {#classic-decision-making-algorithms}

<!--list-separator-->

-  Finite State Machine (forward)

    <!--list-separator-->

    -  Hierarchical Finite State Machine

<!--list-separator-->

-  Behavior Tree (forward)

    <!--list-separator-->

    -  Execution node (leaf node)

        <!--list-separator-->

        -  Condition node

        <!--list-separator-->

        -  Action node

            <!--list-separator-->

            -  Success

            <!--list-separator-->

            -  Fail

            <!--list-separator-->

            -  Running

    <!--list-separator-->

    -  Control flow node (internal node)

        <!--list-separator-->

        -  Sequencer

            依次执行子节点

        <!--list-separator-->

        -  Selector

            只要完成某几个字节点就行

        <!--list-separator-->

        -  Parallel

            同时执行多个字节点

        <!--list-separator-->

        -  Decorator

            负责杂七杂八的事项

            <!--list-separator-->

            -  timer

            <!--list-separator-->

            -  looper

            <!--list-separator-->

            -  ...

    <!--list-separator-->

    -  Tick a BT

        每次都会从Root node开始tick

    <!--list-separator-->

    -  Blackboard


### Advanced AI {#advanced-ai}


#### Hierarchical Tasks Network (backward) {#hierarchical-tasks-network--backward}

Make a plan like human

<!--list-separator-->

-  HTN Framework

    <!--list-separator-->

    -  Sensors

        <!--list-separator-->

        -  Perception

    <!--list-separator-->

    -  World State

        <!--list-separator-->

        -  a subject world view in AI Brain

    <!--list-separator-->

    -  Planner

        <!--list-separator-->

        -  Make a plan (Planning)

            <!--list-separator-->

            -  start from the root task and choose the method satisfying the preconditions in order

            <!--list-separator-->

            -  decompose the method to tasks

            <!--list-separator-->

            -  (for primitive tasks) assume all actions will be succeed, update "world state" in temporary memory. world state has a duplicated copy in planning phase

            <!--list-separator-->

            -  (for primitive tasks) go back and select a new method if precondition is not satisfied

            <!--list-separator-->

            -  repeat steps until no more task needs to be done

    <!--list-separator-->

    -  Plan Runner

        <!--list-separator-->

        -  Update the world state

            <!--list-separator-->

            -  Run plan

                <!--list-separator-->

                -  execute tasks in order

                <!--list-separator-->

                -  stop until all tasks succeed, or one task failed

            <!--list-separator-->

            -  Replan

                <!--list-separator-->

                -  not have a plan

                <!--list-separator-->

                -  the current plan is finished or failed

                <!--list-separator-->

                -  the world state changes via its sensor

    <!--list-separator-->

    -  HTN Domain

        <!--list-separator-->

        -  Hierarchical Tasks

            <!--list-separator-->

            -  Primitive Task

                <!--list-separator-->

                -  Preconditions

                <!--list-separator-->

                -  Action

                <!--list-separator-->

                -  Effects

            <!--list-separator-->

            -  Compound Task

                methods have priority and precondition

                <!--list-separator-->

                -  Method

                    a chain of sub-tasks

                    <!--list-separator-->

                    -  can be a primitive task

                    <!--list-separator-->

                    -  or a compound task

<!--list-separator-->

-  conclusion

    <!--list-separator-->

    -  HTN is similar with BT and more high-level

    <!--list-separator-->

    -  faster than BT

    <!--list-separator-->

    -  !hard to design tasks


#### Goal-Oriented Action Planning (backward) {#goal-oriented-action-planning--backward}

<!--list-separator-->

-  Structure

    Select Goal -&gt; Make Plan -&gt; Execute

    <!--list-separator-->

    -  Sensors

    <!--list-separator-->

    -  World State

    <!--list-separator-->

    -  Goal Set

        <!--list-separator-->

        -  Precondition

        <!--list-separator-->

        -  Priority

        <!--list-separator-->

        -  Goal can be presented as a Collection of States

    <!--list-separator-->

    -  Action Set

        <!--list-separator-->

        -  Precondition

        <!--list-separator-->

        -  Effect

        <!--list-separator-->

        -  Cost

    <!--list-separator-->

    -  Planning

        <!--list-separator-->

        -  check goals based on priority

        <!--list-separator-->

        -  find the first goal which precondition is satisfied

        <!--list-separator-->

        -  compare the target state with world state to find unsatified goal

        <!--list-separator-->

        -  set all unsatisfied states of the goal into a stack

        <!--list-separator-->

        -  check the top unsatisfied state from the stack

        <!--list-separator-->

        -  select an action from action set which could satisfy the chosen state

        <!--list-separator-->

        -  pop the state if it is satisfied by the selected action

        <!--list-separator-->

        -  push action to plan stack

        <!--list-separator-->

        -  check precondition of corresponded action

        <!--list-separator-->

        -  if precondition is not satisfied, push state to stack of unsatisfied states

        <!--list-separator-->

        -  Build States-Action-Cost Graph

            can be turned into a path planning problem. The graph has direction. Use A\* algorithm. Heuristics can be represented with number of unsatisfied states.

            <!--list-separator-->

            -  Node: Combination of states

            <!--list-separator-->

            -  Edge: Action

            <!--list-separator-->

            -  Distance: Cost

    <!--list-separator-->

    -  Plan Runner

<!--list-separator-->

-  Conclusion

    <!--list-separator-->

    -  more dynamic

    <!--list-separator-->

    -  decouping goals and behaviors

    <!--list-separator-->

    -  !more expensive

    <!--list-separator-->

    -  !needs a well-represented world state and action effect


#### Monte Carlo Tree Search (backward) {#monte-carlo-tree-search--backward}

simulate millions possible moves and choose the best step

<!--list-separator-->

-  Monte Carlo Method

    <!--list-separator-->

    -  node: State

        A Tree Structured State Space

    <!--list-separator-->

    -  edge: Action

    <!--list-separator-->

    -  Rebuild the state space for each move

    <!--list-separator-->

    -  Simulation

        Run from the state node according to the Default Policy to produce an outcome

        <!--list-separator-->

        -  Default Policy

            A meaningful but quick rule or neural network to play the game

    <!--list-separator-->

    -  Evaluation Factors

        <!--list-separator-->

        -  Q: Accumulation of Simulation Results

        <!--list-separator-->

        -  N: Number of simulations

    <!--list-separator-->

    -  Backpropagate

        Propagate influence of child state back parent state

    <!--list-separator-->

    -  Iteration Steps

        <!--list-separator-->

        -  Selection

            Always search from the root node.

            <!--list-separator-->

            -  Expandable node

                select the most urgent "expandable" node

                <!--list-separator-->

                -  has unvisited children

                <!--list-separator-->

                -  nonterminal state

            <!--list-separator-->

            -  Exploitatoin

                select the child which has high Q/N value

            <!--list-separator-->

            -  Exploration

                select the child which has low number of visits

            <!--list-separator-->

            -  UCB (Upper Confidence Bounds)

                balance exploitation and exploration.

        <!--list-separator-->

        -  Expansion

            one or more new child nodes are added to selected node

        <!--list-separator-->

        -  Simulation

        <!--list-separator-->

        -  Backpropagate

        <!--list-separator-->

        -  The End Condition

            <!--list-separator-->

            -  Computational budget

                <!--list-separator-->

                -  Memory size

                <!--list-separator-->

                -  Computation time

            <!--list-separator-->

            -  Choose the Best Move

                <!--list-separator-->

                -  Max child

                    the highest Q-value

                <!--list-separator-->

                -  Robust child

                    the most visited

                <!--list-separator-->

                -  Max-Robust child

                    both the highest visit count and the highest reward. if none exist, continue simulation

                <!--list-separator-->

                -  Secure child

                    the child which maximises a lower confidence bound(LCB)

<!--list-separator-->

-  Conclusion

    <!--list-separator-->

    -  behaves diverse

    <!--list-separator-->

    -  agent makes the decision totally by itself

    <!--list-separator-->

    -  can solve the problem of large search space

    <!--list-separator-->

    -  !hard to design for most real-time games

    <!--list-separator-->

    -  !hard to model for most real-time games


#### Machine Learning Basic {#machine-learning-basic}

<!--list-separator-->

-  Supervised Learning

    learn from labeled data
    分类器

<!--list-separator-->

-  Unsupervised Learning

    learn from unlabeled data
    聚类

<!--list-separator-->

-  Semi-supervised Learning

    learn from a lot of unlabeled data and very scarece labeled data. 小样本学习

<!--list-separator-->

-  Reinforcement Learning

    learn from an interaction process with environment

    -   Trial-and-error search
    -   Delayed reward

    <!--list-separator-->

    -  Markov Decision Process

        <!--list-separator-->

        -  Basics

            <!--list-separator-->

            -  Agent

            <!--list-separator-->

            -  Environment

                ---

            <!--list-separator-->

            -  State

            <!--list-separator-->

            -  Action

            <!--list-separator-->

            -  Reward

        <!--list-separator-->

        -  Mathematical Model

            <!--list-separator-->

            -  Probability of transition

            <!--list-separator-->

            -  Policy

                a mapping from states to probabilities of selecting each possible action

            <!--list-separator-->

            -  Total reward


#### Build Advanced Game AI {#build-advanced-game-ai}

<!--list-separator-->

-  Machine Learning Framework in Game

    <!--list-separator-->

    -  Observation

        <!--list-separator-->

        -  The Game State that AI can observe

    <!--list-separator-->

    -  DRL Example

        星际

        <!--list-separator-->

        -  Model the Game

            <!--list-separator-->

            -  State

                state = map + game statistics + units + player data

            <!--list-separator-->

            -  Action

            <!--list-separator-->

            -  Reward

                direct reward: win +1; lose -1
                pseudo reward
                case: OpenAI Five at Dota2

            <!--list-separator-->

            -  NN design

                case: AlphaStar NN Architecture

                <!--list-separator-->

                -  Encoder

                    <!--list-separator-->

                    -  DRL example

                        <!--list-separator-->

                        -  Multi-Layer Perceptron (MLP)

                            处理定长的东西

                        <!--list-separator-->

                        -  CNN

                            处理图像

                        <!--list-separator-->

                        -  Transformer

                            处理不定长的东西

                        <!--list-separator-->

                        -  Long-Short Term Memory (LSTM)

                            汇总处理

                <!--list-separator-->

                -  Decoder

            <!--list-separator-->

            -  Training Strategy

                <!--list-separator-->

                -  Supervised learning

                    <!--list-separator-->

                    -  KL divergence

                <!--list-separator-->

                -  Reinforcement learning

                    <!--list-separator-->

                    -  Train the Agent - Self Play &amp; Adversarial

                        <!--list-separator-->

                        -  Main agents(MA)

                            -   goal: most robust and output
                            -   self-play (35%)
                            -   against past LE and ME (50%)
                            -   against past MA (15%)

                        <!--list-separator-->

                        -  League Exploiters(LE)

                            -   goal: find weakness of past all agents
                            -   against all past agents

                        <!--list-separator-->

                        -  Main Exploiters(ME)

                            -   goal: find weakness of current MA agent
                            -   against current MA agent

                <!--list-separator-->

                -  RL or SL

                    先用sl训练出个能用的，再用rl训练提高上限。
                    Dense reward, rl好，否则sl好。

<!--list-separator-->

-  Hybrid

    ML is powerful but also expensive.


## Online Gaming Architecture (网络游戏的架构) {#online-gaming-architecture--网络游戏的架构}


### Challenges {#challenges}


#### Consistency {#consistency}

Network Synchronization


#### Reliability {#reliability}

Network Latency
Drop and Reconnect


#### Security {#security}

Cheats
Accounts Hacked


#### Diversities {#diversities}

Cross-Play
Rapid iteration
Multiple Games Systems


#### Complexities {#complexities}

High Concurrency
High Availability
High Performance


### Basics {#basics}


#### Network Protocols {#network-protocols}

OSI Model

<!--list-separator-->

-  TCP (Transmission Control Protocol)

<!--list-separator-->

-  UDP (User Datagram Protocol)

<!--list-separator-->

-  Reliable UDP

    TCP is Not Time Critical
    UDP is Unreliable

    <!--list-separator-->

    -  ACK &amp; Sequence Number

        ACK, NACK, SEQ, Timeout

    <!--list-separator-->

    -  Automatic Repeaat Request (ARQ)

        Sliding Window Protocol

        <!--list-separator-->

        -  Stop-and-wait ARQ

        <!--list-separator-->

        -  Go-Back-N ARQ

        <!--list-separator-->

        -  Selective Repeat ARQ

    <!--list-separator-->

    -  Forward Error Correction (FEC)

        <!--list-separator-->

        -  XOR FEC 校验码

        <!--list-separator-->

        -  Reed-Solomon Codes


#### Clock Synchronization {#clock-synchronization}

<!--list-separator-->

-  RTT

    <!--list-separator-->

    -  Round-Trip Time

    <!--list-separator-->

    -  RTT vs. Ping

        ping: transport layer
        rtt: application layer

    <!--list-separator-->

    -  RTT vs. Latency

<!--list-separator-->

-  Network Time Protocol (NTP)

    <!--list-separator-->

    -  Reference clock

    <!--list-separator-->

    -  NTP Algorithm

    <!--list-separator-->

    -  Stream-Based Time Synchronization with Elimination of Higher Order Modes

<!--list-separator-->

-  Time Server Stratums

    <!--list-separator-->

    -  Stratum Values


#### Remote Procedure Call(RPC) {#remote-procedure-call--rpc}

<!--list-separator-->

-  Interface Definition Language

<!--list-separator-->

-  RPC Stubs (票据存根)

<!--list-separator-->

-  Stub Compiler


#### Network Topology {#network-topology}

<!--list-separator-->

-  Original Peer-to-Peer(p2p)

<!--list-separator-->

-  p2p with Host Server 自建房间的人

<!--list-separator-->

-  Dedicated Server


#### Game Synchronization {#game-synchronization}

<!--list-separator-->

-  Snapshot Sync

    client sends inputs to server,
    server simulates the game world and generates game state snapshots,
    server sends them down to clients,
    clients updates the display based on the snapshot

    <!--list-separator-->

    -  Snapshot Interpolation

    <!--list-separator-->

    -  Delta Compression

<!--list-separator-->

-  Lockstep Sync 帧同步

    Deterministic

    <!--list-separator-->

    -  Same input + same execution = same game state

        服务器收集所有人的收入，然后同时分发给所有人，每个人根据同样的输入自行模拟，应当得出同样的结果。
        game progress depends on slowest player

    <!--list-separator-->

    -  Bucket Sync

        设置一个最长时间，超过时间就不等了，直接收集信息并分发。

        <!--list-separator-->

        -  A good trade-off between Consistency and Interactivity Maintenance

            <!--list-separator-->

            -  The Threshold

    <!--list-separator-->

    -  Deterministic Difficulties

        <!--list-separator-->

        -  Floating Point Numbers

            need to follow IEEE-754 standard;
            Fixed-point math;

        <!--list-separator-->

        -  Random Number

            pseudorandom
            make sure the random number seed and the generation algorithm the same

    <!--list-separator-->

    -  Tracing and Debugging

        <!--list-separator-->

        -  checksum

        <!--list-separator-->

        -  frame log

    <!--list-separator-->

    -  Lag and Delay

        <!--list-separator-->

        -  use buffer to cache frames

        <!--list-separator-->

        -  Separating game logic from rendering

    <!--list-separator-->

    -  Reconnection Problem

        <!--list-separator-->

        -  Client Game State Snapshots

        <!--list-separator-->

        -  Quick Catch Up

        <!--list-separator-->

        -  Server State Snapshot Optimization

    <!--list-separator-->

    -  Observing

        存关键帧，电脑自行模拟

    <!--list-separator-->

    -  Replay

    <!--list-separator-->

    -  Cheating Issues

        <!--list-separator-->

        -  upload key data checksum, compare checksums among multiplayers

            server can be considered as a player, too.

        <!--list-separator-->

        -  Difficult to avoid third-party plugin which only access data.

<!--list-separator-->

-  State Sync 状态同步

    state data, events, control data

    <!--list-separator-->

    -  Server authorizes the game world

    <!--list-separator-->

    -  Clients only upload relevant data

    <!--list-separator-->

    -  Authorized and Replicated Clients

        <!--list-separator-->

        -  Authorized

            player's local game client

        <!--list-separator-->

        -  Server

            authorized server

        <!--list-separator-->

        -  Replicated

            simulated character in other player's clien t

    <!--list-separator-->

    -  Dumb Client Problem

        clients can not do anything until receive server state update

        <!--list-separator-->

        -  Client-side prediction

        <!--list-separator-->

        -  Server reconciliation

            <!--list-separator-->

            -  Ring buffer for inputs

        <!--list-separator-->

        -  Packet Loss

            server may duplicate user's last input


### Advanced Topics {#advanced-topics}


#### Character Movement Replication {#character-movement-replication}

<!--list-separator-->

-  Interpolation &amp; Extrapolation

    Smooth movement

    <!--list-separator-->

    -  Interpolation 内插值

        <!--list-separator-->

        -  Scenario

            <!--list-separator-->

            -  Characters' movement

    <!--list-separator-->

    -  Extrapolation 外插值

        Estimate Current State by Extrapolation

        <!--list-separator-->

        -  Dead Reckoning 航位推算

            estimate future state based on states that have been received

        <!--list-separator-->

        -  Projective Velocity Blending

        <!--list-separator-->

        -  Collision Issues

            <!--list-separator-->

            -  Physics Simulatioin Blending During Collision

                switch system between physics system and Extrapolation

        <!--list-separator-->

        -  Scenario

            <!--list-separator-->

            -  Player movement uses a realistic physical model

            <!--list-separator-->

            -  Gameplay suffers from latency

    <!--list-separator-->

    -  Blend Scenario of Interpolation and Extrapolation


#### Hit Registration {#hit-registration}

hit registration is making a consensus of all players that whether you've actually hit your enemy

<!--list-separator-->

-  Client-Side Hit Detection

    best shooting experience, efficient,
    unsafe for cheating

    <!--list-separator-->

    -  A Comparison of Hitscan Weapons vs Projectile Weapons

    <!--list-separator-->

    -  A very simple server verification of Hit Event

        server verification is Very Tricky and Complicated

<!--list-separator-->

-  Server-Side Hit Registration

    <!--list-separator-->

    -  Lag Compensation

        用上一帧的画面来做hit detection。需要做快照
        RewindTime = Current Server Time - Packet Latency - Client View Interpolation Offset

    <!--list-separator-->

    -  Cover Problems

        <!--list-separator-->

        -  Running into Cover

        <!--list-separator-->

        -  Coming out from Cover

        <!--list-separator-->

        -  Hack

            <!--list-separator-->

            -  Startup Frames to Ease Latency Feeling

                前摇动画

            <!--list-separator-->

            -  Local Forecast VFX Impacts

                本地先播放动画，再让服务器结算


#### MMOG Network Architecture {#mmog-network-architecture}

MMOG: Massively Multiplayer Online Game.
MMORPG, MMOFPS...

<!--list-separator-->

-  Game Sub-Systems

    <!--list-separator-->

    -  User Management

    <!--list-separator-->

    -  Matchmaking

    <!--list-separator-->

    -  Trading System

    <!--list-separator-->

    -  Social System

    <!--list-separator-->

    -  Data Storage

    <!--list-separator-->

    -  ...

<!--list-separator-->

-  MMO Architecture

    <!--list-separator-->

    -  Services of Link Layer

        <!--list-separator-->

        -  Login Server

        <!--list-separator-->

        -  Gateway

            隔绝内外网

    <!--list-separator-->

    -  Lobby

    <!--list-separator-->

    -  Character Server

        All player data is managed in one system.

    <!--list-separator-->

    -  Trading System

    <!--list-separator-->

    -  Social System

    <!--list-separator-->

    -  Matchmaking

    <!--list-separator-->

    -  Data Storage

<!--list-separator-->

-  Distributed Systems

    <!--list-separator-->

    -  Load Balancing

        <!--list-separator-->

        -  Consistent Hashing 一致性哈希

        <!--list-separator-->

        -  Virtual Server Node in Consistent Hashing

    <!--list-separator-->

    -  Servers Management

        <!--list-separator-->

        -  Service Discovery

            <!--list-separator-->

            -  Registry

            <!--list-separator-->

            -  Query and Watch

            <!--list-separator-->

            -  Health Check


#### Bandwidth Optimization {#bandwidth-optimization}

<!--list-separator-->

-  Calculate Bandwidth

<!--list-separator-->

-  Data Compression

<!--list-separator-->

-  Object Relevance

    <!--list-separator-->

    -  Static Zones

    <!--list-separator-->

    -  Area of Interest (AOI)

        <!--list-separator-->

        -  Direct Range-Query

        <!--list-separator-->

        -  Spatial-Grid

        <!--list-separator-->

        -  Orthogonal Linked-list 十字链表法

        <!--list-separator-->

        -  Potentially Visible Set (PVS)

<!--list-separator-->

-  Varying Update Frequency by Player Position


#### Anti-Cheat {#anti-cheat}

<!--list-separator-->

-  Cheating ways

    <!--list-separator-->

    -  Game code modifications

        <!--list-separator-->

        -  Obfuscating Memory

            <!--list-separator-->

            -  Executable Packers 加壳

            <!--list-separator-->

            -  内存混淆

        <!--list-separator-->

        -  Verifying Local Files by Hashing

    <!--list-separator-->

    -  System software invoke

        <!--list-separator-->

        -  Valve Anti-Cheat and Easy Anti-Cheat

        <!--list-separator-->

        -  Detecting Known Cheat Program

            对商业化外挂比较好

    <!--list-separator-->

    -  Net packet interception

        <!--list-separator-->

        -  Encrypt the Network Traffic

            use asym encryption to distribute sym key.
            transfer data using sym encryption

            <!--list-separator-->

            -  Symmetric-key algorithm

            <!--list-separator-->

            -  Asymmetric encryption

    <!--list-separator-->

    -  ...

    <!--list-separator-->

    -  AI Cheat

        <!--list-separator-->

        -  Rich AI Middlewares

            -   Real-Time Object Detection. YOLO v7...
            -   Skeleton based Action Recognition

    <!--list-separator-->

    -  Statistic-based System


#### Build a Scalable World {#build-a-scalable-world}

<!--list-separator-->

-  Scalable Game Servers

    Combination of zoning, instancing, and prelication

    <!--list-separator-->

    -  Zoning 分区

        <!--list-separator-->

        -  Seamless Zones

            <!--list-separator-->

            -  Zone Border

                Border width &gt;= max AOI radius

                <!--list-separator-->

                -  Active Entity

                <!--list-separator-->

                -  Ghost Entity

            <!--list-separator-->

            -  Cross Border

    <!--list-separator-->

    -  Instancing

    <!--list-separator-->

    -  Replication


## Advanced Topics {#advanced-topics}


### Data-Oriented Programming and Job System {#data-oriented-programming-and-job-system}


#### Basics of Parallel Programming {#basics-of-parallel-programming}

<!--list-separator-->

-  Types of Multitasking

    <!--list-separator-->

    -  Preemptive Multitasking 抢占式多任务

    <!--list-separator-->

    -  Non-preemptive Multitasking

<!--list-separator-->

-  Thread Context Switch

<!--list-separator-->

-  Parallel Problems in Parallel Computing

    <!--list-separator-->

    -  Embarrassingly Parallel Problem (or Perfectly Parallel)

        Little or no dependency or need for communication between parallel tasks

    <!--list-separator-->

    -  Non-embarrassingly Parallel Problem

        Communication is needed between parallel tasks

<!--list-separator-->

-  Data Race in Parallel Programming

    <!--list-separator-->

    -  Blocking Algorithm

        lock may cause dead lock

        <!--list-separator-->

        -  Locking Primitives

            make a critical section for shared resource access

        <!--list-separator-->

        -  Atomic Operation

            <!--list-separator-->

            -  Lock-free Programming

        <!--list-separator-->

        -  Lock Free vs Wait Free

            lock free will waste cpu usage

<!--list-separator-->

-  Problem of Memory Reordering

    <!--list-separator-->

    -  Compiler Reordering Optimizations

        compilers and CPUs often modify the execution order of instructions to optimize performance.
        execution order of instructions is very critical.


#### Parallel Framework of Game Engine {#parallel-framework-of-game-engine}

<!--list-separator-->

-  Fixed Multi-thread

    -   Render Thread
    -   Simulation Thread
    -   Logic Thread
    -   Network Thread

    cons: not balanced workload; unscalable

<!--list-separator-->

-  Thread Fork-Join

    use a thread pool to prevent frequent thread creation/destruction
    cons: still not very balanced

<!--list-separator-->

-  Task Graph

    a directed acyclic graph

    -   node: Task
    -   edge: Dependency

    building task graph by links
    too static, not dynamic


#### Job System {#job-system}

<!--list-separator-->

-  Coroutine

    <!--list-separator-->

    -  Coroutine vs. Thread

        <!--list-separator-->

        -  Coroutine

            -   scheduled by programmers
            -   to be executed within a thread
            -   context switch is faster without kernel switch

        <!--list-separator-->

        -  Thread

            -   scheduled by os
            -   resides in a process
            -   context switch is costly with kernel switch

    <!--list-separator-->

    -  Stackful Coroutine

    <!--list-separator-->

    -  Stackless Coroutine

<!--list-separator-->

-  Fiber-based Job Systesm

    fiber is like coroutine except that fiber is scheduled by a scheduler

    <!--list-separator-->

    -  One work thread for one core

    <!--list-separator-->

    -  Job Scheduler

        <!--list-separator-->

        -  Global Job

            outside threads add globaljobs to global job queue;
            work thread gets global job from global queue;

        <!--list-separator-->

        -  LIFO and FIFO Mode

            一般会用LIFO Mode，类似dependency关系

        <!--list-separator-->

        -  Job Dependency

            if job yields, add it to waiting job and add necessory jobs to work thread. after finishing necessory jobs, move the job which yields back to work thread

        <!--list-separator-->

        -  Job Stealing

            if some work threads have no job, they will steal jobs from busy threads


#### Data-Oriented Programming(DOP) {#data-oriented-programming--dop}

<!--list-separator-->

-  Principle of Locality

    <!--list-separator-->

    -  Spatial Locality

    <!--list-separator-->

    -  Single Instruction Multiple Data (SIMD)

    <!--list-separator-->

    -  LRU(Least Recently Used)

<!--list-separator-->

-  Cache Line

    <!--list-separator-->

    -  Cache Miss

        <!--list-separator-->

        -  Row-major order

        <!--list-separator-->

        -  Column-major order

<!--list-separator-->

-  Keep Code and Data Tight in Memory

<!--list-separator-->

-  Performance-Sensitive Programming

    <!--list-separator-->

    -  Reducing Order Dependency

        <!--list-separator-->

        -  False Sharing in Cache Line

    <!--list-separator-->

    -  Rranch Prediction

    <!--list-separator-->

    -  Existential Processing

<!--list-separator-->

-  Performance-Sensitive Data Arrangement

    <!--list-separator-->

    -  Array of Structure vs. Structure of Array

        if we want to read the position of all particles, SOA has better performance

<!--list-separator-->

-  Entity Component System (ECS)

    <!--list-separator-->

    -  Entity

        an ID refer to a set of components

    <!--list-separator-->

    -  Componet

        the data to be processed by systems, no logic at all

    <!--list-separator-->

    -  System

        where the logic happens, read/write component data

    <!--list-separator-->

    -  Unity Data-Oriented Tech Stack (DOTS)

        <!--list-separator-->

        -  ECS

            <!--list-separator-->

            -  Archetype

                Type of Game Objects
                Entities are grouped into archetypes

            <!--list-separator-->

            -  Data Layout in Archetype

                same components in an archetype are packed tightly into chunks for cache friendliness

        <!--list-separator-->

        -  C# Job System

        <!--list-separator-->

        -  Burst Compiler

    <!--list-separator-->

    -  Unreal Mass Framework

        <!--list-separator-->

        -  Entity

        <!--list-separator-->

        -  Component

            Fragments and tags

        <!--list-separator-->

        -  Systems

            Processors

            -   ConfigureQuery
            -   Execute

    <!--list-separator-->

    -  ![](/ox-hugo/_20221102_103042cpu_operations_cost.png)

        :ATTACH:


### Dynamic Global Illumination and Lumen {#dynamic-global-illumination-and-lumen}


#### Global Illumination {#global-illumination}

<!--list-separator-->

-  Monte Carlo Integration

    <!--list-separator-->

    -  Sampling is the Key

        <!--list-separator-->

        -  Uniform Sampling 均一采样

            Probability Distribution Function (PDF)
            Importance Sampling

<!--list-separator-->

-  Reflective Shadow Maps (RSM)

    <!--list-separator-->

    -  Cone Tracing with RSM

    <!--list-separator-->

    -  Acceleration with Low-Res Indirect Illumination

<!--list-separator-->

-  Light Propagation Volumes (LPV)

    <!--list-separator-->

    -  Radiance Propagation

<!--list-separator-->

-  Sparse Voxel Octree for Real-time GLobal Illumination (SVOGI)

    <!--list-separator-->

    -  Collect Surface Voxel

    <!--list-separator-->

    -  Shading with Cone Tracing in Voxel Tree

<!--list-separator-->

-  Voxelization Based Global Illumination (VXGI)

    Store the voxel data in clipmaps
    采用LOD的方法，离自己50米内用高精度存个图，离自己50到100米内用中精度存，100米外用低精度存，最后三个图打包起来用

    <!--list-separator-->

    -  Voxel Update and Toroidal Addressing

    <!--list-separator-->

    -  Voxelization for Opacity

    <!--list-separator-->

    -  Accumulate Voxel Radiance and Opacity along the Path

    <!--list-separator-->

    -  Problems

        <!--list-separator-->

        -  Incorrect Occlusion (opacity)

        <!--list-separator-->

        -  Light Leaking

            when occlusion wall is much smaller than voxel size

<!--list-separator-->

-  Screen Space Global Illumination (SSGI)

    <!--list-separator-->

    -  Radiance Sampling in Screen Space

    <!--list-separator-->

    -  Linear Raymarching

    <!--list-separator-->

    -  Hierarchical Tracing

        用hierarchical z的方法，如果光线不被上层阻挡，就直接射向下一层，直至被某层阻挡后，回到上一层去一个个判断

    <!--list-separator-->

    -  Ray Reuse among Neighbor Pixels

    <!--list-separator-->

    -  Cone Tracing with Mipmap Filtering


#### Lumen {#lumen}

-   Ray trace is slow
-   Sampling is hard
-   Low-res filtered scene space probes lit full pixels

<!--list-separator-->

-  Fast Ray Trace in Any Hardware

    <!--list-separator-->

    -  Signed Distance Field (SDF)

        <!--list-separator-->

        -  Per-Mesh SDF

        <!--list-separator-->

        -  SDF for Thin Meshes

        <!--list-separator-->

        -  Ray Tracing with SDF

        <!--list-separator-->

        -  Cone Tracing with SDF

        <!--list-separator-->

        -  Sparse Mesh SDF

        <!--list-separator-->

        -  Mesh SDF LoD

    <!--list-separator-->

    -  Ray Tracing with Global SDF

        <!--list-separator-->

        -  Global SDF

            inaccurate near the start of the zone, but fast

        <!--list-separator-->

        -  Cache Global SDF around Camera

<!--list-separator-->

-  Radiance Injection and Caching

    <!--list-separator-->

    -  Mesh card - orthogonal camera on 6-Axis Aligned directions

        以相机的视界为基准

    <!--list-separator-->

    -  Generate Surface Cache

    <!--list-separator-->

    -  Lighing Cache Pipeline

        用时序上上一帧的直接光照计算当前帧的间接光照

    <!--list-separator-->

    -  Voxel Clipmap for RAdiance Caching of the Whole Scene

    <!--list-separator-->

    -  Build Voxel Faces by Short Ray Cast

    <!--list-separator-->

    -  Inject Light into clipmap

    <!--list-separator-->

    -  Indirect Lighting

    <!--list-separator-->

    -  Combine Lighting

        finalLighting = (directLighting + indirectLighting) \* Diffuse_Lambert(Albedo) +Emissive;

<!--list-separator-->

-  Build a lot of Probes with DIfferent Kinds

    <!--list-separator-->

    -  Screen Space Probe

        -   Octahedron mapping

        <!--list-separator-->

        -  Screen Probe Placement

            16\*16采样无效的话，就细化Tile8\*8，4\*4

        <!--list-separator-->

        -  Screen Probe Atlas

        <!--list-separator-->

        -  Screen Probe Jitter

    <!--list-separator-->

    -  Importance Sampling

        <!--list-separator-->

        -  Accumulate Normal Distribution Nearby

        <!--list-separator-->

        -  Structured Importance Sampling

        <!--list-separator-->

        -  Fix Budget Importance Sampling based on Lighting and BRDF

    <!--list-separator-->

    -  Denoising and Spatial Probe Filtering

        <!--list-separator-->

        -  Denoise

            <!--list-separator-->

            -  Gather Radiance from neighbors

            <!--list-separator-->

            -  Clamp Distance Mismatching

    <!--list-separator-->

    -  World Space Probes and Ray Connecting

        <!--list-separator-->

        -  World Space Radiance Cache

        <!--list-separator-->

        -  Connecting rays

            connect screen probe ray and world probe ray

        <!--list-separator-->

        -  Placement and caching

<!--list-separator-->

-  Shading Full Pixels with Screen Space Probes

    <!--list-separator-->

    -  Convert Probe Radiance to 3rd order Spherical Harmonic

<!--list-separator-->

-  Overall, Performance and Result <span class="tag"><span class="ATTACH">ATTACH</span></span>

    {{< figure src="/ox-hugo/_20221102_155403lumen_tracing_steps.png" >}}


### Nanite {#nanite}


#### GPU Driven Pipeline in Assassins Creed {#gpu-driven-pipeline-in-assassins-creed}

<!--list-separator-->

-  Mesh Cluster Rendering


#### GPU Instance Culling {#gpu-instance-culling}


#### GPU Cluster Culling {#gpu-cluster-culling}

<!--list-separator-->

-  Index Buffer Compaction

<!--list-separator-->

-  Codec Triangle Visibility in Cube: Backface Culling


#### Occlusion Culling for Camera and Shadow {#occlusion-culling-for-camera-and-shadow}


#### Occlusion Depth Generation {#occlusion-depth-generation}


#### Two-Phase Occlusion Culling {#two-phase-occlusion-culling}

<!--list-separator-->

-  1st phase

    cull objects&amp; clusters using last frame's depth pyramid.
    render visible objects

<!--list-separator-->

-  2nd phase

    refresh depth pyramid. Test culled objects&amp; clusters. Render false negatives


#### Fast Occlusion for Shadow {#fast-occlusion-for-shadow}


#### Camera Depth Reprojection for Shadow Culling {#camera-depth-reprojection-for-shadow-culling}


#### Visiblity Buffer {#visiblity-buffer}

a cache-friendly approach to deferred shading

<!--list-separator-->

-  Filling

    for each pixel in screen:

    -   pack (alpha masked bit, drawID, primitiveID) into one 32-bit UINT
    -   write that into a screen-sized buffer

<!--list-separator-->

-  Shading

    for each pixel in screen:

    -   get drawID/triangleID at pixel pos
    -   load data for the 3 vertices from the VB
    -   compute triangle gradients
    -   interpolate vertex attributes at pixel pos using gradients

<!--list-separator-->

-  Visibility Buffer + Deferred Shading

    <!--list-separator-->

    -  Correct Texture Mipmap with Gradient


#### Virtual Geometry - Nanite {#virtual-geometry-nanite}

适合静态事物，不适合动态人物、物品

<!--list-separator-->

-  Overview

    <!--list-separator-->

    -  Virtual Texture

    <!--list-separator-->

    -  Voxels? no

    <!--list-separator-->

    -  Subdivision Surface? no

    <!--list-separator-->

    -  Maps-based Method? no

    <!--list-separator-->

    -  Point Cloud? no

    <!--list-separator-->

    -  Foundation of Computer Graphics - Triangles! yes

<!--list-separator-->

-  Geometry Represent

    <!--list-separator-->

    -  Cluster-based LoD

        <!--list-separator-->

        -  View Dependent LoD Transitions

            近处三角形多，远处三角形少

        <!--list-separator-->

        -  Naive Solution

            <!--list-separator-->

            -  CLuster LoD Hierarchy

            <!--list-separator-->

            -  Simple Streaming Idea

        <!--list-separator-->

        -  LoD Cracks

            the cross region of different LoD

            <!--list-separator-->

            -  Locked Boundaries (bad)

            <!--list-separator-->

            -  Nanite Solution - CLuster Group

                聚合之前的三角形成group，在group里重新划分出较大的三角形。三角形的边界和之前的边界可能会不同

            <!--list-separator-->

            -  DAG for Cluster Groups (not tree)

            <!--list-separator-->

            -  detail of simplification - QEM

        <!--list-separator-->

        -  Runtime LoD Selection

            <!--list-separator-->

            -  LoD Selection in Parallel

                <!--list-separator-->

                -  Core Equation of Parallel LoD Selection for Cluster Groups

                    when can we LoD cull a cluster?

                    -   Render: ParentError &gt; threshold &amp;&amp; ClusterError &lt;= threshold
                    -   Cull: ParentError &lt;= threhold || ClusterError &gt; threshold

                    if parent is already precise enough, no need to check child.

                    -   ParentError &lt;= threshold
                    -   tree based on ParentError, not ClusterError

                <!--list-separator-->

                -  Isolated LoD Selection for Each Cluster Group

    <!--list-separator-->

    -  BVH and runtime LoD

        <!--list-separator-->

        -  Build BVH for Acceleratio of LoD Selection

            balance BVH for 4 nodes, every node is a LoD cluster group

<!--list-separator-->

-  Rendering

    <!--list-separator-->

    -  Software and Hardware Rasterization

        <!--list-separator-->

        -  Scanline Software Rasterizer

        <!--list-separator-->

        -  Depth Test

        <!--list-separator-->

        -  Visibility Buffer

        <!--list-separator-->

        -  Imposters for Tiny Instances

    <!--list-separator-->

    -  Deferred Materials

        <!--list-separator-->

        -  Shading Efficiency

            hardware depth test;
            convert material id to depth value

        <!--list-separator-->

        -  Material Sorting with Tile-Bsed Rendering

            <!--list-separator-->

            -  Material Classify

    <!--list-separator-->

    -  Tile-based Acceleration

<!--list-separator-->

-  Virtual Shadow Map

    A Cached Shadow System

    <!--list-separator-->

    -  Sample Distribution Shadow Maps

    <!--list-separator-->

    -  Virtual Shadow Mapping

        16k \* 16k virtual shadow map for each light;
        相机和光源不动就不用更新，动了才更新

        <!--list-separator-->

        -  Shadow Page Allocation

        <!--list-separator-->

        -  Shadow Page Cache Invalidation

<!--list-separator-->

-  Streaming and Compression

    <!--list-separator-->

    -  Memory representation

        vertex quantization and encoding

    <!--list-separator-->

    -  Disk representation

        -   hardware LZ decompression
        -   for better compression
        -   transcode on the GPU

