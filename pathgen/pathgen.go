// Package pathgen 提供测试与演示用的参考路径构造器，全部为密集折线采样。
package pathgen

import (
	"math"

	"github.com/TommyMarsss/pure-pursuit/controller"
)

// Sample 按弧长参数 s∈[0,total] 均匀采样 n 个点。
func Sample(total float64, n int, fn func(s float64) controller.Point) []controller.Point {
	if n < 2 {
		n = 2
	}
	out := make([]controller.Point, n)
	for i := 0; i < n; i++ {
		s := total * float64(i) / float64(n-1)
		out[i] = fn(s)
	}
	return out
}

// Concat 顺序拼接多段采样点（相邻段的衔接点不重复）。
func Concat(parts ...[]controller.Point) []controller.Point {
	var out []controller.Point
	for _, p := range parts {
		if len(p) == 0 {
			continue
		}
		if len(out) > 0 {
			p = p[1:]
		}
		out = append(out, p...)
	}
	return out
}

// Straight 长度 L、航向 heading 的直线（起点为原点）。
func Straight(L, heading float64, n int) []controller.Point {
	c, s := math.Cos(heading), math.Sin(heading)
	return Sample(L, n, func(t float64) controller.Point {
		return controller.Point{X: t * c, Y: t * s}
	})
}

// CircleArc 圆心 (cx,cy)、半径 R、角度从 a0 扫到 a1（弧度，逆时针为正）的圆弧。
func CircleArc(cx, cy, R, a0, a1 float64, n int) []controller.Point {
	L := R * math.Abs(a1-a0)
	return Sample(L, n, func(s float64) controller.Point {
		a := a0 + (a1-a0)*(s/L)
		return controller.Point{X: cx + R*math.Cos(a), Y: cy + R*math.Sin(a)}
	})
}

// FigureEight 自交叉“8 字”路径：x=A sin(t), y=A sin(t) cos(t)，t∈[0,2π]，
// 在原点自交叉一次，用于检验前视点在自交叉处不跳变。
func FigureEight(A float64, n int) []controller.Point {
	const pmax = 2 * math.Pi
	return Sample(pmax, n, func(s float64) controller.Point {
		// 参数采样（非严格弧长，8 字对称，误差可忽略）。
		t := s
		return controller.Point{X: A * math.Sin(t), Y: A * math.Sin(t) * math.Cos(t)}
	})
}

// UTurn 进直线 + 急半圆掉头 + 出直线。R 为转弯半径（越小曲率越大）。
// 进段沿 +x 进入，转弯中心在 (entry, R)，出段沿 -x 折回。
func UTurn(entry, exit, R float64, nPerPart int) []controller.Point {
	in := Straight(entry, 0, nPerPart)
	// 半圆：从角度 -pi/2（入口处 (entry,0)）扫到 +pi/2（出口 (entry,2R)）。
	arc := CircleArc(entry, R, R, -math.Pi/2, math.Pi/2, nPerPart)
	// 出段沿 180°（-x），平移到出口点。
	out0 := Straight(exit, math.Pi, nPerPart)
	out := make([]controller.Point, len(out0))
	exitPt := arc[len(arc)-1]
	for i, p := range out0 {
		out[i] = controller.Point{X: p.X + exitPt.X, Y: p.Y + exitPt.Y}
	}
	return Concat(in, arc, out)
}

// Sinusoidal 正弦蜿蜒路径 y=amp*sin(k*x)，曲率随 amp*k^2 增大。
func Sinusoidal(length, amp, wavelength float64, n int) []controller.Point {
	k := 2 * math.Pi / wavelength
	return Sample(length, n, func(s float64) controller.Point {
		return controller.Point{X: s, Y: amp * math.Sin(k*s)}
	})
}
