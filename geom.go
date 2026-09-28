package main

import "math"

// Vec 是平面二维点/向量。
type Vec struct {
	X, Y float64
}

func (a Vec) Add(b Vec) Vec       { return Vec{a.X + b.X, a.Y + b.Y} }
func (a Vec) Sub(b Vec) Vec       { return Vec{a.X - b.X, a.Y - b.Y} }
func (a Vec) Scale(s float64) Vec { return Vec{a.X * s, a.Y * s} }
func (a Vec) Dot(b Vec) float64   { return a.X*b.X + a.Y*b.Y }

// Cross 是二维标量叉积 a×b。取正值表示 b 位于 a 的左侧（逆时针方向）。
func (a Vec) Cross(b Vec) float64 { return a.X*b.Y - a.Y*b.X }

func (a Vec) Len() float64 { return math.Hypot(a.X, a.Y) }

func (a Vec) Distance(b Vec) float64 { return math.Hypot(a.X-b.X, a.Y-b.Y) }

func (a Vec) Angle() float64 { return math.Atan2(a.Y, a.X) }

// wrapPi 将任意角度归一化到 (-π, π]，这是正确处理角度环绕的唯一入口，
// 航向误差一律经过它计算，禁止直接对两个 atan2 的结果做差。
func wrapPi(a float64) float64 {
	a = math.Mod(a, 2*math.Pi)
	if a > math.Pi {
		a -= 2 * math.Pi
	} else if a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// clamp 将 v 限制在 [lo, hi]。
func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// Path 是分段线性参考路径，Pts 按行进方向排列，S[i] 为 Pts[i] 处的累计弧长。
// 所有路径均按较密间距（约 0.05~0.1 m）采样，折点处切向跳变足够小，
// 最近点/切点在相邻段之间切换时横向误差连续。
type Path struct {
	Pts []Vec
	S   []float64
}

// NewPath 由有序点列构造路径并计算累计弧长。
func NewPath(pts []Vec) *Path {
	s := make([]float64, len(pts))
	for i := 1; i < len(pts); i++ {
		s[i] = s[i-1] + pts[i].Distance(pts[i-1])
	}
	return &Path{Pts: pts, S: s}
}

func (p *Path) Total() float64 {
	if len(p.S) == 0 {
		return 0
	}
	return p.S[len(p.S)-1]
}

// segAt 返回弧长 s 所在段的下标与段内比例 t∈[0,1]。
func (p *Path) segAt(s float64) (int, float64) {
	n := len(p.Pts)
	if s <= 0 {
		return 0, 0
	}
	if s >= p.Total() {
		return n - 2, 1
	}
	// 二分查找最后一个 S[i] <= s。
	lo, hi := 0, n-1
	for lo+1 < hi {
		mid := (lo + hi) / 2
		if p.S[mid] <= s {
			lo = mid
		} else {
			hi = mid
		}
	}
	segLen := p.S[lo+1] - p.S[lo]
	if segLen == 0 {
		return lo, 0
	}
	return lo, (s - p.S[lo]) / segLen
}

// At 返回弧长 s 处的插值点（s 自动截断到 [0, Total]）。
func (p *Path) At(s float64) Vec {
	i, t := p.segAt(s)
	a, b := p.Pts[i], p.Pts[i+1]
	return a.Add(b.Sub(a).Scale(t))
}

// TangentAt 返回弧长 s 处的路径前进方向单位切向量。
func (p *Path) TangentAt(s float64) Vec {
	i, _ := p.segAt(s)
	d := p.Pts[i+1].Sub(p.Pts[i])
	return d.Scale(1 / d.Len())
}

// Projection 是机器人在路径上的最近投影结果。
type Projection struct {
	S       float64 // 投影点（切点）的弧长
	Point   Vec     // 投影点坐标
	Tangent Vec     // 该处路径切向
	E       float64 // 有符号横向误差：路径相对机器人的横向偏移，
	// 路径在机器人左侧为正、右侧为负。机器人在路径左侧时 E<0，
	// 据此 atan(kE/v) 会给出右转（负 ω）指令，物理方向与
	// Stanley 前轮转角公式 δ=θe+atan(kE/v) 完全一致。
}

// projectWindow 只在弧长窗口 [sLo, sHi] 覆盖的段内搜索最近投影。
// 用弧长窗口而非全局搜索，是路径自交叉时不被另一枝"抢走"最近点的关键；
// 取窗口上界足够大、下界略向后留余量，正常跟随不会漏点。
// fwd 为机器人前进方向单位向量（航向），仅用于自交叉点处的平局裁决。
func (p *Path) projectWindow(x, fwd Vec, sLo, sHi float64) Projection {
	sLo = clamp(sLo, 0, p.Total())
	sHi = clamp(sHi, 0, p.Total())
	if sHi < sLo {
		sHi = sLo
	}
	best := Projection{S: sLo, Point: p.At(sLo)}
	bestD2 := math.Inf(1)
	// 自交叉点处多段到机器人的距离可能完全相等（8 字路径中原点在
	// s=0、s=L/2、s=L 三处重合）。平局裁决：先比段切向与机器人航向的
	// 一致性——机器人正向驶过的枝 cos>0，路径终点处的枝切向与航向相反
	// 会被排除；航向也一致时再按弧长向前裁决，使投影从"来路"自然切到
	// "去路"。仅在距离差小于 projTie 时触发，真正更近的段不受影响。
	const projTie2 = 0.05 * 0.05
	for i := 0; i+1 < len(p.Pts); i++ {
		// 与窗口无重叠的段直接跳过。
		if p.S[i+1] < sLo || p.S[i] > sHi {
			continue
		}
		a, b := p.Pts[i], p.Pts[i+1]
		d := b.Sub(a)
		l2 := d.Dot(d)
		t := 0.0
		if l2 > 0 {
			t = clamp(x.Sub(a).Dot(d)/l2, 0, 1)
		}
		q := a.Add(d.Scale(t))
		d2 := x.Distance(q)
		s := p.S[i]
		tangent := Vec{}
		if l2 > 0 {
			s += t * math.Sqrt(l2)
			tangent = d.Scale(1 / math.Sqrt(l2))
		}
		pick := false
		switch {
		case d2 < bestD2-projTie2:
			pick = true // 明显更近
		case math.Abs(d2-bestD2) <= projTie2 && l2 > 0:
			// 几何平局：航向一致性优先，其次弧长向前。
			newAlign := tangent.Dot(fwd)
			oldAlign := 0.0
			if best.Tangent != (Vec{}) {
				oldAlign = best.Tangent.Dot(fwd)
			}
			pick = newAlign > oldAlign+0.05 ||
				(math.Abs(newAlign-oldAlign) <= 0.05 && s > best.S)
		}
		if pick {
			bestD2 = d2
			// 叉积约定：E = t × (q - x)。投影点 q 在 x 左侧时为正。
			// 段切换发生在折点处：两段都给出同一个 q，符号定义不依赖
			// 机器人航向也不使用 atan2，因此机器人越过切点前后符号连续。
			e := 0.0
			if l2 > 0 {
				e = tangent.Cross(q.Sub(x))
			}
			best = Projection{S: s, Point: q, Tangent: tangent, E: e}
		}
	}
	return best
}
