# Solà 两篇论文精读

精读对象是已下载的原文，不是转述网页。

| 文件 | 文献 | 下载版本 |
| --- | --- | --- |
| `1711.02508.pdf` | Joan Solà, Quaternion kinematics for the error-state Kalman filter | arXiv:1711.02508v1，2017-11-08，正文约 95 页 |
| `1812.01537.pdf` | Joan Solà, Jeremie Deray, Dinesh Atchuthan, A micro Lie theory for state estimation in robotics | arXiv:1812.01537v9，2021-12-08，正文 17 页 |

第一篇把四元数、旋转和 IMU 误差状态卡尔曼滤波写成一套能直接编码的公式。第二篇把同一套“流形上的小扰动”收成李群语言，并给出 ESKF 与图优化共用的雅可比规则。代码库 [manif](https://github.com/artivis/manif) 实现的是第二篇，不是第一篇的 IMU 机械编排。

当前环境里的技能只有需求文档管理和交互面板，没有论文精读流程。下面的笔记按这两份 PDF 的章节写，公式编号沿用原文。

## 建议怎么读

先读第一篇第 3、5、6 节，再读第二篇第 II.E、II.G、II.H 和 V.A。第一篇第 1、2 节是四元数和 $SO(3)$ 的工具箱，遇到符号再回查。第二篇明确不讲李括号，不要拿它当完整李群教材。

两篇的默认约定都是局部、右扰动。第一篇正文用

$$
\mathbf{q}_t = \mathbf{q} \otimes \delta\mathbf{q}
$$

第二篇默认

$$
\mathcal{Y} = \mathcal{X} \oplus {}^{\mathcal{X}}\!\boldsymbol{\tau} \triangleq \mathcal{X} \circ \mathrm{Exp}({}^{\mathcal{X}}\!\boldsymbol{\tau})
$$

右侧的指数属于 $\mathcal{X}$ 处的切空间，作者把它称为局部坐标系中的扰动。全局误差是左乘，两篇都单独讨论，都不是默认实现。

## 第一篇在解决什么

IMU 积分会漂移。作者把真值拆成可非线性积分的名义状态，以及适合线性高斯滤波的小误差。名义状态吃高频 IMU，误差状态只在 GNSS、视觉等低频量测到来时更新。更新后把误差均值注入名义状态，再把误差均值清零，并用重置雅可比修正协方差。

第 5.2 节列出 ESKF 相对直接 EKF 的四条理由，这些理由决定了后面的公式形态：

- 姿态误差用三维旋转向量，与自由度相同，协方差不必再约束四元数范数。
- 误差始终在原点附近，线性化点不会跑到万向节锁或四元数冗余带来的奇异点。
- 二阶小量可以丢掉，雅可比往往只含名义姿态和 IMU 读数。
- 大信号已经放进名义积分，误差动态慢，校正频率可以低于 IMU。

## 四元数约定：先选定再写代码

第 3 节把四元数差异收成四个二元选择：分量顺序、乘法手性、主动或被动、局部到全局还是反向。作者在式 (2) 起就选定 Hamilton：

- 分量顺序是 $(q_w, q_x, q_y, q_z)$，实部在前。
- 代数是 $ij = k$，右手系。
- 旋转解释为被动。
- 与 Eigen、ROS、Ceres 以及大量 IMU 姿态滤波文献一致。

JPL 约定实部在后，$ji = k$，左手系。作者写明它与 Trawny、Roumeliotis 2005 年的技术报告以及 Li、Mourikis 的视觉惯性论文一致。式 (146) 给出二者关系：左手四元数是右手四元数的共轭。混用时，向量旋转和误差乘法都会反号，不能只改存储顺序。

本资料库里的 `04_imu_gps_localization` 和 `06_eskf_gnss_imu_localization` 按这篇 Hamilton 笔记实现。KF-GINS 用导航系 $\phi$ 角，不使用这篇的四元数右误差块，但“名义状态积分、误差状态滤波、再反馈”的结构相同。

## 真值、名义值和误差

第 5.3 节表 3 是全文的状态定义。真值、名义值、误差的复合是：

$$
\begin{aligned}
\mathbf{p}_t &= \mathbf{p} + \delta\mathbf{p} \\
\mathbf{v}_t &= \mathbf{v} + \delta\mathbf{v} \\
\mathbf{q}_t &= \mathbf{q} \otimes \delta\mathbf{q} \\
\mathbf{a}_{bt} &= \mathbf{a}_b + \delta\mathbf{a}_b \\
\boldsymbol{\omega}_{bt} &= \boldsymbol{\omega}_b + \delta\boldsymbol{\omega}_b \\
\mathbf{g}_t &= \mathbf{g} + \delta\mathbf{g}
\end{aligned}
$$

姿态误差只保留三维，$\delta\mathbf{q} = \exp(\delta\boldsymbol{\theta}/2)$。误差状态因此是 18 维：

$$
\delta\mathbf{x} =
\begin{bmatrix}
\delta\mathbf{p} \\
\delta\mathbf{v} \\
\delta\boldsymbol{\theta} \\
\delta\mathbf{a}_b \\
\delta\boldsymbol{\omega}_b \\
\delta\mathbf{g}
\end{bmatrix}
$$

重力放进状态，是因为初始姿态取单位四元数，水平面不确定被转移到初始重力，而不是去估计相对水平面的姿态。式 (235f) 里重力真值的导数为零。地球自转在式 (232) 的脚注中被省略；作者写明，只有零偏和噪声小到能直接感知约 $15^\circ/\mathrm{h}$ 的高端 IMU，才需要把地球自转加回去。这解释了为什么 Solà 路线的短时机器人代码与 KF-GINS 的地球模型不是同一套方程。

IMU 模型写成机体读数：

$$
\begin{aligned}
\mathbf{a}_m &= \mathbf{R}_t^{\mathrm{T}}(\mathbf{a}_t - \mathbf{g}_t) + \mathbf{a}_{bt} + \mathbf{a}_n \\
\boldsymbol{\omega}_m &= \boldsymbol{\omega}_t + \boldsymbol{\omega}_{bt} + \boldsymbol{\omega}_n
\end{aligned}
$$

反解后代入运动学，得到真值系统 $\dot{\mathbf{x}}_t = f_t(\mathbf{x}_t, \mathbf{u}, \mathbf{w})$。名义系统去掉噪声，并用当前零偏估计补偿 $\mathbf{a}_m$ 和 $\boldsymbol{\omega}_m$。误差系统是时变线性系统，系统矩阵由名义状态算出。

角速度和默认的角误差都定义在名义姿态的局部系，因此陀螺读数可以直接使用。第 7 节改成全局角误差 $\mathbf{q}_t = \delta\mathbf{q} \otimes \mathbf{q}$，角速度积分仍是右乘，因为陀螺永远在机体系。Li 与 Mourikis 2012 的证据被作者用来说明全局误差性质更好，但算法主体仍是局部误差。

## 离散预测：均值可以不写，协方差不能省

误差均值初始化为零，线性预测 $\hat{\delta\mathbf{x}} \leftarrow \mathbf{F}_x \hat{\delta\mathbf{x}}$ 永远给出零。作者在式 (268) 后明确要求代码里跳过这一行，但协方差预测必须保留：

$$
\mathbf{P} \leftarrow \mathbf{F}_x \mathbf{P} \mathbf{F}_x^{\mathrm{T}} + \mathbf{F}_i \mathbf{Q}_i \mathbf{F}_i^{\mathrm{T}}
$$

式 (270) 给出最粗的 Euler 转移矩阵。速度误差受姿态误差、加计零偏和重力误差驱动；姿态误差受陀螺零偏驱动。$\mathbf{F}_x$ 里出现名义旋转 $\mathbf{R}$、补偿后的比力 $[\mathbf{a}_m - \mathbf{a}_b]_{\times}$ 和角速度。附录 B 到 D 给出更高阶的离散化，正文这一版只适合先把数据流跑通。

噪声向量 $\mathbf{i}$ 含速度、角度、加计和陀螺四块脉冲，对应协方差 $\mathbf{V}_i$、$\boldsymbol{\Theta}_i$、$\mathbf{A}_i$、$\boldsymbol{\Omega}_i$。零偏随机游走在连续模型里是 $\mathbf{a}_w$ 和 $\boldsymbol{\omega}_w$，不要和量测白噪声 $\mathbf{a}_n$、$\boldsymbol{\omega}_n$ 合成一个标量。

## 校正、注入和重置

非 IMU 量测写成 $y = h(\mathbf{x}_t) + v$。因为滤波的是误差，雅可比必须对 $\delta\mathbf{x}$ 求导，并在名义状态处取值。链式法则是：

$$
\mathbf{H} = \mathbf{H}_x \mathbf{X}_{\delta x}
$$

$\mathbf{H}_x$ 是普通 EKF 对真值状态的雅可比，随传感器改变。$\mathbf{X}_{\delta x}$ 只取决于状态复合方式，位置和速度块是单位阵，姿态块来自四元数右乘。

校正之后的三步在第 6 节开头写死：

1. 用卡尔曼增益观察误差。
2. 把误差均值注入名义状态。位置、速度、零偏和重力是相加；姿态是 $\mathbf{q}^+ = \mathbf{q} \otimes \hat{\delta\mathbf{q}}$。
3. 重置误差。

重置函数是 $\delta\mathbf{x} \leftarrow \delta\mathbf{x} \ominus \hat{\delta\mathbf{x}}$。均值被置零。协方差乘重置雅可比 $\mathbf{G}$。除姿态外 $\mathbf{G}$ 是单位阵。姿态块是：

$$
\frac{\partial \delta\boldsymbol{\theta}^+}{\partial \delta\boldsymbol{\theta}} = \mathbf{I} - \left[\frac{1}{2}\hat{\delta\boldsymbol{\theta}}\right]_{\times}
$$

推导用了“真值姿态在重置时不变”，于是新误差四元数等于旧误差四元数左乘已注入误差的共轭。作者承认多数实现直接取 $\mathbf{G} = \mathbf{I}_{18}$。完整的半角叉乘项用于减小长期里程计漂移，不是数值装饰。第二篇的右雅可比在小角度下展开，第一项就是这个 $\mathbf{I} - \frac{1}{2}[\hat{\delta\boldsymbol{\theta}}]_{\times}$。

## 第二篇删掉了什么，留下了什么

第二篇把李群定义为同时满足群公理的光滑流形。估计时真正使用的是切空间 $\mathbb{R}^m$，而不是李代数上的括号。作者引用 Howe 关于李括号几乎决定李群的论述，然后明确把括号留在外面：机器人状态估计需要的是不确定性、导数和积分，这些都可以放在与李代数同构的向量空间里，精度没有损失。因此这篇不能用来推导新的李代数结构常数，但足够写 EKF 和图优化。

群、切空间和指数的图像是：恒等元处的切平面经 $\exp$ 铺到流形上的测地线；反过来每个群元素有一个 $\log$。球面只是示意图，三维球面不是他们用来计算的群，$S^3$ 上的单位四元数才是。

## 右加、右减和伴随

第 II.E 节定义增量。右加、右减是：

$$
\begin{aligned}
\mathcal{Y} &= \mathcal{X} \oplus {}^{\mathcal{X}}\!\boldsymbol{\tau} \triangleq \mathcal{X} \circ \mathrm{Exp}({}^{\mathcal{X}}\!\boldsymbol{\tau}) \\
{}^{\mathcal{X}}\!\boldsymbol{\tau} &= \mathcal{Y} \ominus \mathcal{X} \triangleq \mathrm{Log}(\mathcal{X}^{-1} \circ \mathcal{Y})
\end{aligned}
$$

左加是 $\mathrm{Exp}({}^{E}\!\boldsymbol{\tau}) \circ \mathcal{X}$，扰动在恒等元切空间，也就是全局坐标。减号本身不分左右，作者规定默认用右减。这个默认与第一篇局部角误差一致：第一篇的 $\mathbf{q} \otimes \delta\mathbf{q}$ 就是 $SO(3)$ 或单位四元数上的右加。

伴随矩阵把局部切向量变到全局切向量，${}^{E}\!\boldsymbol{\tau} = \mathrm{Ad}_{\mathcal{X}}\, {}^{\mathcal{X}}\!\boldsymbol{\tau}$。$SE(3)$ 的例子是：

$$
\mathrm{Ad}_{\mathbf{M}} =
\begin{bmatrix}
\mathbf{R} & [\mathbf{t}]_{\times}\mathbf{R} \\
\mathbf{0} & \mathbf{R}
\end{bmatrix}
$$

左雅可比和右雅可比通过伴随互相转换。预测协方差时，运动增量的右雅可比 $\mathbf{J}_r(\mathbf{u})$ 和 $\mathrm{Ad}_{\mathrm{Exp}(\mathbf{u})}^{-1}$ 就是从这里来的，不必再从四元数乘法重新求导。

## 协方差必须写在切空间

第 II.H 节把流形上的高斯写成切空间扰动的协方差：

$$
\boldsymbol{\Sigma}_{\mathcal{X}} \triangleq \mathrm{E}\!\left[(\mathcal{X} \ominus \bar{\mathcal{X}})(\mathcal{X} \ominus \bar{\mathcal{X}})^{\mathrm{T}}\right]
$$

维度等于自由度。若对过参数的四元数或旋转矩阵直接做 $\mathrm{E}[(\mathbf{X}-\bar{\mathbf{X}})(\mathbf{X}-\bar{\mathbf{X}})^{\mathrm{T}}]$，协方差是奇异的。这把第一篇“姿态误差必须三维”从 $SO(3)$ 推广到任意李群。

不确定性传播仍是 $\boldsymbol{\Sigma} \leftarrow \mathbf{J}\boldsymbol{\Sigma}\mathbf{J}^{\mathrm{T}}$，只是 $\mathbf{J}$ 换成右雅可比。链式法则在第 III.A 节证明：复合函数的右雅可比等于各块右雅可比相乘。因此新传感器只要写出对位姿的作用，再乘附录里的逆、复合、指数和动作雅可比。

## 流形上的 ESKF 只改了加号

第 V.A 节用已知信标的 $SE(2)$ 定位把滤波写成可执行步骤。误差和协方差定义在估计位姿的切空间：

$$
\delta\mathbf{x} \triangleq \mathcal{X} \ominus \hat{\mathcal{X}}, \quad
\mathbf{P} \triangleq \mathrm{E}[(\mathcal{X} \ominus \hat{\mathcal{X}})(\mathcal{X} \ominus \hat{\mathcal{X}})^{\mathrm{T}}]
$$

预测是：

$$
\begin{aligned}
\hat{\mathcal{X}}_j &= \hat{\mathcal{X}}_i \oplus \mathbf{u}_j \\
\mathbf{P}_j &= \mathbf{F}\mathbf{P}_i\mathbf{F}^{\mathrm{T}} + \mathbf{G}\mathbf{W}_j\mathbf{G}^{\mathrm{T}}
\end{aligned}
$$

其中 $\mathbf{F} = \mathrm{Ad}_{\mathrm{Exp}(\mathbf{u}_j)}^{-1}$，$\mathbf{G} = \mathbf{J}_r(\mathbf{u}_j)$。校正的新息、增益和协方差与普通卡尔曼滤波相同。唯一的状态更新是 $\hat{\mathcal{X}} \leftarrow \hat{\mathcal{X}} \oplus \delta\mathbf{x}$。作者写明：相对普通 EKF，改变的是式 (99) 和式 (101) 里的加法；雅可比按李群计算，增益公式不动。

这一节没有 IMU 零偏，也没有第一篇的名义状态与误差状态双轨积分。它说明的是：一旦状态本身放在流形上，注入误差就是右加，不必再维护一个单独的误差四元数均值。FAST-LIO 的 IKFoM 属于这一支。KF-GINS 的 $\phi$ 角模型仍是第一篇式的双轨误差状态，姿态用旋转向量，但机械编排在导航系，并含地球模型。

第 V.B 节把同一套右减用于平滑与建图。运动残差是 $\mathbf{u}_{ij} - (\hat{\mathcal{X}}_j \ominus \hat{\mathcal{X}}_i)$，路标残差是量测减去位姿逆作用在路标上。信息矩阵开方后加权，正规方程的雅可比仍按块右雅可比组装。因此 ESKF 和图优化可以共用附录中的雅可比，差别只在求解器一次更新还是滑窗迭代。

## 两篇公式的对应

| 操作 | 第一篇局部 ESKF | 第二篇默认右扰动 |
| --- | --- | --- |
| 姿态复合 | $\mathbf{q}_t = \mathbf{q} \otimes \delta\mathbf{q}$ | $\mathcal{X} \oplus \boldsymbol{\tau} = \mathcal{X}\circ\mathrm{Exp}(\boldsymbol{\tau})$ |
| 全局误差 | 第 7 节 $\mathbf{q}_t = \delta\mathbf{q} \otimes \mathbf{q}$ | 左加 $\mathrm{Exp}(\boldsymbol{\tau})\circ\mathcal{X}$ |
| 协方差所在空间 | 18 维误差，姿态 3 维 | 切空间，维度等于自由度 |
| 预测均值 | 名义状态由 IMU 积分；误差均值保持为零 | 位姿本身做 $\oplus\mathbf{u}$ |
| 协方差预测 | $\mathbf{F}_x \mathbf{P} \mathbf{F}_x^{\mathrm{T}} + \mathbf{F}_i\mathbf{Q}_i\mathbf{F}_i^{\mathrm{T}}$ | $\mathbf{F}\mathbf{P}\mathbf{F}^{\mathrm{T}} + \mathbf{G}\mathbf{W}\mathbf{G}^{\mathrm{T}}$，$\mathbf{G}=\mathbf{J}_r$ |
| 注入 | 分块相加或四元数右乘 | 统一为 $\oplus$ |
| 重置雅可比 | $\mathbf{I}-\frac{1}{2}[\hat{\delta\boldsymbol{\theta}}]_{\times}$ | 小角度右雅可比的一阶项 |

## 读的时候容易写错的地方

- 第一篇式 (268) 的误差均值预测在零均值初始化后恒为零。漏掉的是式 (269) 的协方差，不是均值。
- 重置雅可比作用在更新后的协方差上。把它乘进卡尔曼增益，或在注入之前清零误差，都会把姿态不确定性转错。
- Hamilton 与 JPL 不能只交换 $w$ 的位置。手性相反时，叉乘和旋转作用一起变号。
- 第一篇默认局部角误差。抄第 7 节的全局误差运动学时，速度方程里 $[\delta\boldsymbol{\theta}]_{\times}$ 的位置不同，不能与局部 $\mathbf{F}_x$ 混用。
- 第二篇的减号默认是右减。若协方差用左减定义，伴随必须一起改，否则 $\mathbf{F}$ 和 $\mathbf{H}$ 会差一个 $\mathrm{Ad}$。
- 地球自转、子午圈曲率和里程计非完整约束不在这两篇的主公式里。车载 RTK 与底盘仍以 KF-GINS 的 $\phi$ 角模型为准，这两篇负责姿态块和流形雅可比，不负责导航系机械编排。

## 和本目录代码的对应

| 论文中的对象 | 代码 |
| --- | --- |
| 第一篇 Hamilton 局部 ESKF，IMU 加位置量测 | `02_codes/04_imu_gps_localization`，`02_codes/06_eskf_gnss_imu_localization` |
| 同一预测上比较 ESKF、IEKF、UKF | `02_codes/05_imu_x_fusion` |
| 名义 IMU 积分加误差状态，但是导航系 $\phi$ 角 | `02_codes/01_KF-GINS`，`02_codes/02_KF-GINS-Matlab` |
| 第二篇的右扰动、右雅可比和流形协方差 | FAST-LIO 的 `include/IKFoM_toolkit`，以及 manif |
| 第二篇第 V.B 节的图优化 | 本目录没有下载。OB_GINS、LIO-SAM 属于这一类，但不是这两篇的参考实现 |
