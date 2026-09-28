package controller

import "math"

// PurePursuit 参数。
type PurePursuit struct {
	// Lookahead 基础前视距离（米）。
	Lookahead float64
	// K 速度自适应增益：实际前视距离 = Lookahead + K*v。
	// 速度越高看得越远，是抑制高速切弯（corner cutting）失稳的常用手段，0 表示固定前视。
	K float64
	// Wheelbase 虚拟轮距，用于把角速度换算成等效前轮转角 delta。
	Wheelbase float64
	// SearchWindow 最近点/前视点搜索的弧长窗口（相对上一帧投影弧长，前后各 Window）。
	// <= 0 表示全路径搜索（首帧或无回环的简单路径可用）。
	SearchWindow float64
	// MaxSteer 等效前轮转角限幅（弧度，绝对值），<=0 表示不限幅。
	MaxSteer float64

	// 内部状态：上一帧最近点弧长，用于搜索窗口与防跳变。
	lastS float64
	// 内部状态：上一帧前视点弧长，保证前视点沿路径单调，不允许跳回。
	lastLookS float64
	init_     bool
}

// Reset 清除内部状态（更换路径或重新开始仿真时调用）。
func (pp *PurePursuit) Reset() {
	pp.lastS = 0
	pp.lastLookS = 0
	pp.init_ = false
}

func (pp *PurePursuit) lookahead(v float64) float64 {
	l := pp.Lookahead
	if pp.K > 0 {
		l += pp.K * math.Abs(v)
	}
	if l < 1e-9 {
		l = 1e-9
	}
	return l
}

// PPCResult 输出。
type PPCResult struct {
	// OmegaCmd 角速度指令（rad/s），单轮模型 theta' = omega。
	OmegaCmd float64
	// Delta 等效自行车模型前轮转角（rad），delta = atan(L*kappa)。
	Delta float64
	// Target 前视点。
	Target Point
	// TargetS 前视点弧长。
	TargetS float64
	// Lookahead 本帧实际使用的前视距离。
	Lookahead float64
	// Proj 当前最近点投影信息。
	Proj Projection
	// Kappa 前视弧对应的等效曲率 2*sin(alpha)/Ld。
	Kappa float64
}

// Compute 计算一帧控制量。
//
// 几何推导：设机器人系下前视点坐标 (gx, gy)，前视距离 Ld，
// 则圆弧跟踪曲率 kappa = 2*gy / Ld^2 = 2*sin(alpha)/Ld，
// 其中 alpha 为机器人航向到前视点视线的夹角。
func (pp *PurePursuit) Compute(path *Path, pose Pose, v float64) PPCResult {
	ld := pp.lookahead(v)

	hint := -1.0
	if pp.init_ {
		hint = pp.lastS
	}
	proj := path.ClosestFrom(pose.X, pose.Y, hint, pp.SearchWindow)

	// 期望前视弧长 = 最近点弧长 + Ld。超出路径终点时，At() 会把
	// 目标夹在终点上——这是“前视距离超出剩余路径”的正确处理：
	// 目标即终点，视线距离变为真实剩余距离，曲率公式仍有效，
	// 机器人会沿最小转弯圆弧驶向终点而非发出突变指令。
	targetS := proj.S + ld
	if targetS > path.Length() {
		targetS = path.Length()
	}

	// 防跳变：前视点弧长只允许沿路径前进方向单调增长
	// （搜索窗口已经隔离了自交叉的另一段路径，这里再加一道保险，
	// 允许 1e-9 的数值回退）。
	if pp.init_ && targetS+1e-9 < pp.lastLookS {
		targetS = pp.lastLookS
	}
	target := path.At(targetS)

	alpha := AngleDiff(angleTo(Point{X: pose.X, Y: pose.Y}, target), pose.Theta)

	// 实际视线距离可能小于名义 Ld（终点截断情形），曲率按真实
	// 视线长度计算，保证接近终点时指令随距离平滑衰减。
	eye := math.Hypot(target.X-pose.X, target.Y-pose.Y)
	if eye < 1e-9 {
		// 已到达前视点（即终点）：沿路径切向收尾。
		alpha = AngleDiff(proj.Heading, pose.Theta)
		eye = ld
	}
	kappa := 2 * math.Sin(alpha) / eye
	delta := math.Atan(pp.wheelbase() * kappa)
	if pp.MaxSteer > 0 {
		delta = clamp(delta, -pp.MaxSteer, pp.MaxSteer)
		kappa = math.Tan(delta) / pp.wheelbase()
	}

	pp.lastS = proj.S
	pp.lastLookS = targetS
	pp.init_ = true

	return PPCResult{
		OmegaCmd:  kappa * v,
		Delta:     delta,
		Target:    target,
		TargetS:   targetS,
		Lookahead: ld,
		Proj:      proj,
		Kappa:     kappa,
	}
}

func (pp *PurePursuit) wheelbase() float64 {
	if pp.Wheelbase > 0 {
		return pp.Wheelbase
	}
	return 1
}
