package controller

import "math"

// Path 是分段线性参考路径，带累积弧长参数化。
//
// 所有查询（最近点、弧长 s 处的插值点、切向）都基于弧长 s，
// 这样在路径自交叉时只要沿用上一帧的最近弧长 s*，搜索窗口就不会
// 因为另一段交叉路径“更近”而把跟踪点拽走。
type Path struct {
	Pts []Point

	// cum[i] = Pts[0..i] 的累积长度；cum[0] = 0
	cum []float64
	// total 路径总长
	total float64
}

// NewPath 根据折线顶点构造路径，至少需要两个点。
func NewPath(pts []Point) *Path {
	if len(pts) < 2 {
		panic("controller: path needs at least 2 points")
	}
	cum := make([]float64, len(pts))
	for i := 1; i < len(pts); i++ {
		cum[i] = cum[i-1] + dist(pts[i-1], pts[i])
	}
	return &Path{Pts: pts, cum: cum, total: cum[len(cum)-1]}
}

// Length 返回路径总弧长。
func (p *Path) Length() float64 { return p.total }

// At 返回弧长 s 处的插值点，超出终点时截断到终点。
func (p *Path) At(s float64) Point {
	s = clamp(s, 0, p.total)
	i := p.segmentIndex(s)
	segLen := p.cum[i+1] - p.cum[i]
	var t float64
	if segLen > 0 {
		t = (s - p.cum[i]) / segLen
	}
	return Point{
		X: p.Pts[i].X + t*(p.Pts[i+1].X-p.Pts[i].X),
		Y: p.Pts[i].Y + t*(p.Pts[i+1].Y-p.Pts[i].Y),
	}
}

// tangentAt 返回弧长 s 处的切向单位向量（指向弧长增加方向）。
func (p *Path) tangentAt(s float64) Point {
	s = clamp(s, 0, p.total)
	i := p.segmentIndex(s)
	d := sub(p.Pts[i+1], p.Pts[i])
	l := norm(d)
	if l == 0 {
		return Point{X: 1}
	}
	return scale(d, 1/l)
}

// headingAt 返回弧长 s 处的路径航向角。
func (p *Path) headingAt(s float64) float64 {
	t := p.tangentAt(s)
	return math.Atan2(t.Y, t.X)
}

// segmentIndex 返回满足 cum[i] <= s <= cum[i+1] 的段下标 i。
func (p *Path) segmentIndex(s float64) int {
	// 折线段数通常不多，且调用方（Closest）已用窗口限制范围，
	// 线性扫描即可；对密集路径可用二分替换而不改变接口。
	lo, hi := 0, len(p.cum)-2
	for lo < hi {
		mid := (lo + hi) / 2
		if p.cum[mid+1] < s {
			lo = mid + 1
		} else {
			hi = mid
		}
	}
	return lo
}

// Projection 描述机器人在路径上的最近点（垂足/切点）信息。
type Projection struct {
	// S 最近点的弧长坐标（被夹在 [0, Length]）。
	S float64
	// Point 最近点位置。
	Point Point
	// Heading 最近点处路径切向航向。
	Heading float64
	// CrossTrack 横向误差（带符号）：机器人位置相对路径切向的
	// 左手法向偏移，正=位于路径左侧，负=右侧。
	// 定义为 cross(tangent, robotPoint - nearestPoint)，
	// 越过切点前后符号只取决于机器人在切向的哪一侧，物理意义连续。
	CrossTrack float64
	// Segment 最近点所在折线段的起始顶点下标。
	Segment int
	// PastEnd 最近点是否为路径终点（弧长已到尽头）。
	PastEnd bool
}

// ClosestFrom 从弧长提示 hintS 出发，在其前后 window 弧长窗口内
// （窗口内的折线段，且至少检查一个段）寻找 pose 的最近投影点。
//
// hintS 一般传入上一帧的 Projection.S：在自交叉或大曲率回环路径上，
// 这保证跟踪点沿路径单调推进，不会跳到几何上更近的另一截路径。
// 首帧可传 -1 表示全路径搜索。
func (p *Path) ClosestFrom(px, py, hintS float64, window float64) Projection {
	robot := Point{X: px, Y: py}

	i0, i1 := 0, len(p.Pts)-2
	if hintS >= 0 && window >= 0 {
		lo := clamp(hintS-window, 0, p.total)
		hi := clamp(hintS+window, 0, p.total)
		i0 = p.segmentIndex(lo)
		i1 = p.segmentIndex(hi)
	}

	bestSeg := i0
	bestS := 0.0
	bestD2 := math.Inf(1)
	for i := i0; i <= i1; i++ {
		a, b := p.Pts[i], p.Pts[i+1]
		d := sub(b, a)
		l2 := dot(d, d)
		t := 0.0
		if l2 > 0 {
			t = clamp(dot(sub(robot, a), d)/l2, 0, 1)
		}
		q := add(a, scale(d, t))
		d2 := dist2(robot, q)
		if d2 < bestD2 {
			bestD2 = d2
			bestSeg = i
			bestS = p.cum[i] + t*(p.cum[i+1]-p.cum[i])
		}
	}

	// 单调推进：投影弧长不允许回退（机器人物理上不可能沿参考路径
	// 倒退），这与窗口共同杜绝自交叉处的跟踪点跳变。
	if hintS > bestS {
		bestS = hintS
		bestSeg = p.segmentIndex(clamp(bestS, 0, p.total))
	}

	pt := p.At(bestS)
	tan := p.tangentAt(bestS)
	ct := cross2(tan, sub(robot, pt))
	return Projection{
		S:          bestS,
		Point:      pt,
		Heading:    math.Atan2(tan.Y, tan.X),
		CrossTrack: ct,
		Segment:    bestSeg,
		PastEnd:    bestS >= p.total-1e-12,
	}
}

func dist2(a, b Point) float64 {
	dx, dy := a.X-b.X, a.Y-b.Y
	return dx*dx + dy*dy
}
