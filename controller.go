package main

import "math"

// LookaheadPolicy 决定 Pure Pursuit 的前视距离 Ld 随速度变化：
//
//	Ld = clamp(Kf * v, LdMin, LdMax)
//
// 低速时由 LdMin 兜底，避免 Ld→0 造成 ω=2v sinα/Ld 增益无限大（这是 Pure Pursuit
// 最典型的失稳来源）；高速时由 LdMax 限制，保证弯道上不会"看穿"到很远的路径段。
type LookaheadPolicy struct {
	Kf    float64 // 速度前馈系数（s）
	LdMin float64 // 最小前视距离（m）
	LdMax float64 // 最大前视距离（m）
}

func (p LookaheadPolicy) Ld(v float64) float64 {
	ld := p.Kf * v
	return clamp(ld, p.LdMin, p.LdMax)
}

// projBacktrack 是 Stanley 最近投影窗口相对上一帧投影位置向后保留的弧长，
// 允许最近点在离散采样下短暂回退，又不至于在自交叉路径上抓到另一枝。
const projBacktrack = 0.5

// PurePursuit 是几何 Pure Pursuit 控制器。
// 输出为角速度指令 ω = 2 v sin(α) / Ld，α 为机器人航向到前视点的方位角。
// 等价于前轮转角 δ=atan(2L sinα/Ld)（L 为轴距）经运动学映射 ω=v tanδ/L。
type PurePursuit struct {
	Look LookaheadPolicy
	// Backtrack 是前视锚点允许回扫的弧长（m）：机器人一个时间步内可能
	// 越过旧锚点（机器人位于前视圆内），交点会短暂落在旧锚点后方，故
	// 每帧从 sGoal-Backtrack 起重新找第一个圆交点。取值须大于单步位移
	// v·dt（否则漏点），又要明显小于 Ld（0 时自动取 0.4·Ld），
	// 这样临近自交叉点时来路尾段上的交点在回扫窗口之外，不会被重新捕获。
	Backtrack float64
	sGoal     float64 // 上一帧前视点弧长（状态量，控制器有记忆）
	armed     bool
}

// NewPurePursuit 创建控制器。
func NewPurePursuit(look LookaheadPolicy) *PurePursuit {
	return &PurePursuit{Look: look}
}

// Reset 清除前视点记忆（开始一段新的仿真前调用）。
func (c *PurePursuit) Reset() { c.armed = false; c.sGoal = 0 }

// targetArc 沿路径行进方向返回第一个落在半径 ld 圆上、弧长位于 [sFrom,sHi]
// 的交点。必须取"第一个交点"而不是全路径上半径误差最小的点：在自交叉路径
// 上，第二枝往往也与圆相交，全局优选会跨枝取点，锚点一旦跳过去就可能直接
// 锁到终点，表现为前视点跳变与转向指令突变。
// 关键几何细节：求交只接受段内参数 t∈[0,1] 的根。机器人临近交叉点时，圆
// 与"来路"尾段的交点落在该段延长线上（t>1），于是扫描自然越过交叉点、
// 落到"去路"首段——不需要任何针对交叉点的特判。
func targetArc(p *Path, x Vec, sFrom, sHi, ld float64) (float64, Vec, bool) {
	sFrom = clamp(sFrom, 0, p.Total())
	sHi = clamp(sHi, 0, p.Total())
	n := len(p.Pts)
	// 找到第一个与 sFrom 有交叠的段，之后按段顺序扫描。
	i0 := 0
	for i0+1 < n-1 && p.S[i0+1] < sFrom {
		i0++
	}
	for i := i0; i+1 < n; i++ {
		if p.S[i] > sHi {
			break // 段已按弧长有序，超出上界后不必再看
		}
		a, b := p.Pts[i], p.Pts[i+1]
		d := b.Sub(a)
		segLen := p.S[i+1] - p.S[i]
		if segLen <= 0 {
			continue
		}
		// 段内参数 t 满足 |x - (a+t d)| = ld。
		af := x.Sub(a)
		b2 := d.Dot(d)
		c0 := af.Dot(af) - ld*ld
		b1 := -2 * af.Dot(d)
		disc := b1*b1 - 4*b2*c0
		if disc < 0 {
			continue // 该段与半径 ld 的圆不相交
		}
		sq := math.Sqrt(disc)
		t1 := (-b1 - sq) / (2 * b2) // 进入圆的根（较小 t）
		t2 := (-b1 + sq) / (2 * b2) // 离开圆的根（较大 t）
		for _, t := range []float64{t1, t2} {
			if t < 0 || t > 1 {
				continue
			}
			s := p.S[i] + t*segLen
			if s < sFrom-1e-9 {
				continue // 单调性约束（容差处理折点重复计数）
			}
			return s, a.Add(d.Scale(t)), true
		}
	}
	return 0, Vec{}, false
}

// Compute 返回 ω（rad/s）。target 为实际采用的前视点（供可视化），
// proj 为最近投影（横向误差与失稳判据共用同一定义）。
func (c *PurePursuit) Compute(pose Pose, v float64, p *Path) (omega float64, target Vec, proj Projection) {
	x := pose.Pos()
	fwd := Vec{math.Cos(pose.Theta), math.Sin(pose.Theta)}
	if !c.armed {
		// 首帧从路径起点锚定。自交叉点两侧的枝在该点等距，最近投影
		// 无法区分"去路"与"另一圈的末端"；路径跟踪的进度语义必须是
		// 从 s=0 单调向前，故初始锚点不取最近点。
		c.sGoal = 0
		c.armed = true
	}
	ld := c.Look.Ld(v)
	bt := c.Backtrack
	if bt <= 0 {
		bt = 0.4 * ld
	}
	sFrom := c.sGoal - bt
	if sFrom < 0 {
		sFrom = 0
	}

	// 横向误差投影的窗口要容得下真正的最近点（正常跟踪时它落后前视点
	// 至多一个 Ld），故下界取 sGoal-Ld-余量；自交叉点附近两枝共点共
	// 切向，平局由 projectWindow 的航向一致性/弧长向前裁决处理。
	proj = p.projectWindow(x, fwd, sFrom-ld-0.3, c.sGoal+ld+1)

	var g Vec
	endPt := p.Pts[len(p.Pts)-1]
	// 只有沿弧确实临近终点（剩余弧长 ≤ Ld）且终点已进入前视圆时才锁定
	// 终点。两个条件缺一不可：
	//  - 只看空间距离会在自交叉路径的中间交叉点提前锁终点（那里空间上
	//    距终点很近，但沿弧还有一整圈）；
	//  - 锁定瞬间圆与路径的最后一个交点正与终点重合，目标点不跳变；
	//  - 机器人越过终点后也不会把圆在来路上的进入点误当新目标反向打舵。
	// 绝不外推终点之后的虚拟路径。
	if p.Total()-c.sGoal <= ld && x.Distance(endPt) <= ld {
		c.sGoal = p.Total()
		g = endPt
	} else {
		// 其余情况取 [sGoal-Backtrack, 终点] 上第一个圆交点，目标沿弧
		// 平滑前行。回退窗口小于 Ld，自交叉点处来路尾段上的旧交点距新
		// 锚点超过 Backtrack，扫描自然落到去路首段。
		s, q, ok := targetArc(p, x, sFrom, p.Total(), ld)
		if !ok {
			// 剩余路径整体在圆外（如起点处路径比 Ld 短且终点偏远，或
			// 初始横向偏差过大）：直接瞄准终点/前推锚点，待圆重新与
			// 路径相交后恢复正常取点。
			if p.Total()-c.sGoal <= ld {
				c.sGoal = p.Total()
				g = endPt
			} else {
				c.sGoal = math.Min(c.sGoal+ld, p.Total())
				g = p.At(c.sGoal)
			}
		} else {
			c.sGoal, g = s, q
		}
	}
	alpha := wrapPi(g.Sub(x).Angle() - pose.Theta)
	omega = 2 * v * math.Sin(alpha) / ld
	return omega, g, proj
}

// StanleyParams 为 Stanley 控制器参数。
//
//	δ = θe + atan(k E / (v + ks))
//	ω = v/L · tan(δ)
type StanleyParams struct {
	K  float64 // 横向误差增益（1/s）
	Ks float64 // 软ening 速度，低速时抑制 atan 项饱和（m/s）
	L  float64 // 等效轴距（m），把前轮转角映射为角速度
}

// Stanley 是 Stanford 路径跟踪控制器（角速率形式）。
type Stanley struct {
	P     StanleyParams
	sPrev float64
	armed bool
}

func NewStanley(par StanleyParams) *Stanley { return &Stanley{P: par} }

func (c *Stanley) Reset() { c.armed = false; c.sPrev = 0 }

// Compute 返回 ω。target 为最近切点（供可视化，含义与 Pure Pursuit 的前视点不同）。
func (c *Stanley) Compute(pose Pose, v float64, p *Path) (omega float64, target Vec, proj Projection) {
	x := pose.Pos()
	fwd := Vec{math.Cos(pose.Theta), math.Sin(pose.Theta)}
	lo := 0.0
	if c.armed {
		lo = c.sPrev - projBacktrack - 0.2
	}
	// 投影窗口前后各留约 2 m：正常跟踪横向误差远小于此，自交叉路径上
	// 另一枝距当前弧长位置一整圈，不会进入窗口；交叉点平局由航向一致性
	// 与弧长向前裁决（见 projectWindow）。
	proj = p.projectWindow(x, fwd, lo, c.sPrev+2)
	c.sPrev = proj.S
	c.armed = true

	// 航向误差：路径切向角与机器人航向之差，必须经 wrapPi 归一化。
	// θ 与路径切向同时越过 ±π 时，直接相减会得到约 ±2π 的假误差，
	// 输出 δ 随之跳变；wrapPi 保证误差始终落在 (-π,π]，控制量连续。
	thetaE := wrapPi(proj.Tangent.Angle() - pose.Theta)

	// 横向误差项 atan(k E /(v+ks))：E 的定义是"路径相对机器人的有符号偏移"
	// （t×(q−x)，路径在左为正）。机器人从路径右侧横移越过切点到左侧时，
	// E 从正连续变负、不经任何角度计算，atan 项随之平滑反向，转向方向
	// 始终把机器人拉回路径，不会出现越过切点后符号翻转导致反向推离。
	cross := math.Atan2(c.P.K*proj.E, v+c.P.Ks)
	delta := thetaE + cross

	// δ→±π/2 时 tan 数值发散。把等效转角钳到 ±deltaSat 再做映射，
	// 钳制在边界处 C0 连续（两侧表达式在同一点取值相同），符号也始终
	// 与 δ 一致；不会出现饱和区控制量突变。
	const deltaSat = math.Pi/2 - 0.05
	delta = clamp(delta, -deltaSat, deltaSat)
	omega = v / c.P.L * math.Tan(delta)
	return omega, proj.Point, proj
}
