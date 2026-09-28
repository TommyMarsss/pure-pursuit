package controller

import "math"

// Stanley 控制器参数。
//
// 经典 Stanley 前轮转角律（前轮为控制点、自行车模型）：
//
//	delta = psi + atan(k * e / (v + kSoft))
//
// 其中 psi 为航向误差，e 为前轮处横向误差。本实现的控制点是
// 单轮模型的车体原点，并明确采用如下符号约定：
//
//	横向误差 e = cross(tangent, robot - nearest)
//	  e > 0 机器人位于有向路径左侧，需要向右打舵修正，
//	  e < 0 位于右侧，需要向左打舵。
//
// 因此横向项在代入经典公式时取 -e：
//
//	delta = psi - atan(k * e / (|v| + kSoft))
//
// （等价于经典公式中“路径在车体左方时 e 取正”的约定。）
type Stanley struct {
	// K 横向误差增益（1/速度量纲的软兜底由 KSoft 处理），越大回到路径越快。
	K float64
	// KPsi 航向误差增益，经典 Stanley 取 1。
	KPsi float64
	// KSoft 速度软化项：分母 |v|+KSoft，避免 v→0 时横向项奇异。
	KSoft float64
	// Wheelbase 虚拟轮距，用于角速度与等效前轮角互换。
	Wheelbase float64
	// SearchWindow 最近点搜索的弧长窗口，含义同 PurePursuit.SearchWindow。
	SearchWindow float64
	// MaxSteer 等效前轮转角限幅（弧度），<=0 不限幅。
	MaxSteer float64

	lastS float64
	init_ bool
}

// Reset 清除内部状态。
func (st *Stanley) Reset() {
	st.lastS = 0
	st.init_ = false
}

// StanleyResult 输出。
type StanleyResult struct {
	// OmegaCmd 角速度指令 rad/s（单轮模型）。
	OmegaCmd float64
	// Delta 等效前轮转角 rad。
	Delta float64
	// Proj 最近点投影（含带符号横向误差 CrossTrack）。
	Proj Projection
	// HeadingErr 归一化后的航向误差 psi = wrap(pathHeading - theta)。
	HeadingErr float64
	// CrossTerm 横向误差贡献的转角项 -atan(k*e/(|v|+kSoft))。
	CrossTerm float64
}

// Compute 计算一帧控制量。
func (st *Stanley) Compute(path *Path, pose Pose, v float64) StanleyResult {
	hint := -1.0
	if st.init_ {
		hint = st.lastS
	}
	proj := path.ClosestFrom(pose.X, pose.Y, hint, st.SearchWindow)

	// 航向误差：唯一合法写法是先求差再 WrapPi。
	// psi 在路径航向或机器人航向扫过 ±pi 时都保持连续，
	// 不会在边界处产生 2*pi 的控制量跳变。
	psi := AngleDiff(proj.Heading, pose.Theta)

	// 横向误差项。e 由叉积给出（见 Path.ClosestFrom），机器人越过
	// 切点（最近点沿路径移动）时 e 的符号只取决于它在路径切线的
	// 哪一侧，不会因为 atan2 的包裹而翻转，转向方向始终物理一致。
	kpsi := st.KPsi
	if kpsi == 0 {
		kpsi = 1
	}
	ksoft := st.KSoft
	if ksoft <= 0 {
		ksoft = 0.1
	}
	crossTerm := -math.Atan(st.K * proj.CrossTrack / (math.Abs(v) + ksoft))

	delta := kpsi*psi + crossTerm
	if st.MaxSteer > 0 {
		delta = clamp(delta, -st.MaxSteer, st.MaxSteer)
	}
	L := st.Wheelbase
	if L <= 0 {
		L = 1
	}
	// 自行车模型 omega = v/L * tan(delta)。
	omega := v / L * math.Tan(delta)

	st.lastS = proj.S
	st.init_ = true

	return StanleyResult{
		OmegaCmd:   omega,
		Delta:      delta,
		Proj:       proj,
		HeadingErr: psi,
		CrossTerm:  crossTerm,
	}
}
