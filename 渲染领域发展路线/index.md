# 渲染领域发展路线


## 总览 {#总览}


### 工作职责 {#工作职责}

|     | 初级     | 中级          | 高级     |
|-----|--------|-------------|--------|
| 程序向 | 工具编写 | DCC工具开发   | 自动化工具系统 |
|     | 渲染知识、效果 |               | 深度优化渲染管线 |
|     | 美术流程 | 效率优化      |          |
|     | 环节规范 |               |          |
| 美术向 | 建立美术动作规范 | 深度制定流程，确定效果标杆 | 制定效果标杆 |


#### 技术美术（制作内容） {#技术美术-制作内容}

<!--list-separator-->

-  Shaders

<!--list-separator-->

-  Effects

<!--list-separator-->

-  Workflows R&amp;D

<!--list-separator-->

-  Optimization, debugging

<!--list-separator-->

-  Education

<!--list-separator-->

-  Connecting artists with programmers

<!--list-separator-->

-  Pipeline

<!--list-separator-->

-  Tools

<!--list-separator-->

-  Animation and rigging tools


#### 图形程序（制作工具） <span class="tag"><span class="ATTACH">ATTACH</span></span> {#图形程序-制作工具}

{{< figure src="/ox-hugo/_20230710_005348TA_functions.jpg" >}}

{{< figure src="/ox-hugo/_20230710_005401TA_functions_cn.jpg" >}}


## 技术方向 {#技术方向}


### DCC工具的基本使用 {#dcc工具的基本使用}

模型、贴图、动画制作

-   3D Max，Maya, Blender 建模、动画
-   ZBrush 雕刻模型、低模高模、贴图制作
-   Substance 贴图绘制
-   Houdini 程序化生成


### 引擎 {#引擎}

Unity、UE

-   渲染管线
-   粒子特效
-   光照灯光
-   材质着色器
-   动画


### NPR {#npr}

-   以美学经验为理论基础。高度依赖于创作者的经验和审美能力。
    &gt; 既然这方面并不需要所谓真实的理论基础，那是否可以借助当前的AI技术，通过数据形成渲染参数模型。AI generated stylization
    &gt; 绘画相比摄像的优势在于提取主观而关键的信息来表达，NPR也是如此。
-   引擎材质与美术模型的配合很重要
    -   模型脸部法线对于NPR的阴影效果影响很重要。


#### 底层基础 {#底层基础}

<!--list-separator-->

-  光照模型

    缺乏系统的光照模型

    -   基于引擎，或是自研

<!--list-separator-->

-  相机模型


### PBR {#pbr}

当今的GPU图形管线和3D美术流程，是为了PBR量身打造的，而NPR需要使用各种Trick。

-   以物理光学和微分几何为理论基础。


#### 光照模型 {#光照模型}


#### 高质量美术模型、细节及贴图 {#高质量美术模型-细节及贴图}


#### 基于物理和摄影的场景灯光 {#基于物理和摄影的场景灯光}


#### PBR材质 {#pbr材质}

<!--list-separator-->

-  皮肤

<!--list-separator-->

-  眼睛

<!--list-separator-->

-  毛发

<!--list-separator-->

-  衣服


### 效果 {#效果}


#### 描边 - Pencil4+的效果、参考动漫行业标准 {#描边-pencil4-plus-的效果-参考动漫行业标准}


#### Toon-shading - 迪士尼的头发效果 {#toon-shading-迪士尼的头发效果}


#### Fog Everything {#fog-everything}


#### Ambient Occlusion {#ambient-occlusion}


#### Anti-aliasing {#anti-aliasing}


### 后处理 {#后处理}


#### Paraffin 赛璐璐动画中彩色石蜡纸通过叠加在赛璐璐胶片上，光与暗的反射产生的特殊效果。 {#paraffin-赛璐璐动画中彩色石蜡纸通过叠加在赛璐璐胶片上-光与暗的反射产生的特殊效果}


#### Diffusioin 模糊 {#diffusioin-模糊}


#### Kuwahara 平滑滤波，油画效果 {#kuwahara-平滑滤波-油画效果}


#### Cinematic Dof 景深 {#cinematic-dof-景深}


#### Lens Flares 圆形镜头光晕 {#lens-flares-圆形镜头光晕}


#### Bloom · Exposure 曝光 {#bloom-exposure-曝光}


#### Chromatic Aberration 色差 {#chromatic-aberration-色差}


#### Vignette Intensity {#vignette-intensity}


#### Color Grading LUT {#color-grading-lut}


#### Motion Blur {#motion-blur}


#### ToneMapping {#tonemapping}


### 场景渲染 {#场景渲染}

大气、云、植被、地形


### 渲染管线 {#渲染管线}


#### Forward Rendering {#forward-rendering}


#### Deferred Rendering {#deferred-rendering}


#### Tile-based Rendering {#tile-based-rendering}


#### etc.. {#etc-dot-dot}

[GAMES104笔记]({{< relref "games104笔记.md" >}})


### 物理模拟 {#物理模拟}


#### 布料模拟 {#布料模拟}

-   基于刚体
-   基于网格


### 平台针对性 {#平台针对性}


#### 移动端效果提升及性能优化 {#移动端效果提升及性能优化}


### 硬件底层 {#硬件底层}


#### 不同图形API {#不同图形api}


#### AMD FSR（FidelityFX Super Resolution) {#amd-fsr-fidelityfx-super-resolution}

空间像素倍增技术，用于提升帧率


#### Nvidia DLSS {#nvidia-dlss}

通过AI提升分辨率


#### 虚拟纹理 {#虚拟纹理}

屏蔽了中间层的像素-&gt;渲染接口


### 格式规范 {#格式规范}


#### 贴图压缩标准 {#贴图压缩标准}


### LookDev {#lookdev}

交互式查看接近最终品质的渲染结果，立即看到材质、灯光、渲染参数修改带来的效果反馈。
[lookdev](https://zhuanlan.zhihu.com/p/397143080)


### 新技术 {#新技术}


#### 神经风格迁移 {#神经风格迁移}

Unity Barracuda库有支持，可以即用


#### 神经辐射场建模 NeRF三维建模 {#神经辐射场建模-nerf三维建模}

仅用2d的posed images作为监督，即可表示复杂的三维场景。(适合用于做跟客户交互的设计原型Demo，场景建模)


## 发展目标 {#发展目标}


### 模型渲染 {#模型渲染}


#### NPR {#npr}

<!--list-separator-->

-  小k

<!--list-separator-->

-  抖音直播伴侣


#### PBR {#pbr}

<!--list-separator-->

-  Unity HDRP Enemies

    <!--list-separator-->

    -  HairFX制作

    <!--list-separator-->

    -  SSS制作

<!--list-separator-->

-  UE MetaHuman

    [UE Metahuman剖析](https://zhuanlan.zhihu.com/p/565831151)

<!--list-separator-->

-  Maya X-gen for Hair


#### BlendShape制作 or 生成 {#blendshape制作-or-生成}


### 性能优化 {#性能优化}


#### PC {#pc}


#### 移动端 {#移动端}

