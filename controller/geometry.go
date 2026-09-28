// Package controller 实现差速驱动机器人的路径跟踪控制器，
// 目前包含 Pure Pursuit 与 Stanley 两种算法。
//
// 差速底盘的运动学为单轮模型（unicycle）：
//
//	x' = v*cos(theta), y' = v*sin(theta), theta' = omega
//
// 因此控制器的最终输出是角速度指令 OmegaCmd；同时给出等效自行车
// 模型前轮转角 Delta（按虚拟轮距 Wheelbase 换算），便于与经典公式对照。
package controller

import "math"

// Point 平面点。
type Point struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Pose 机器人位姿：位置与航向角（弧度）。
type Pose struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Theta float64 `json:"theta"`
}

func sub(a, b Point) Point { return Point{a.X - b.X, a.Y - b.Y} }
func add(a, b Point) Point { return Point{a.X + b.X, a.Y + b.Y} }
func scale(a Point, k float64) Point {
	return Point{a.X * k, a.Y * k}
}
func dot(a, b Point) float64    { return a.X*b.X + a.Y*b.Y }
func cross2(a, b Point) float64 { return a.X*b.Y - a.Y*b.X }
func norm(a Point) float64      { return math.Hypot(a.X, a.Y) }
func dist(a, b Point) float64   { return math.Hypot(a.X-b.X, a.Y-b.Y) }

// angleTo 返回从点 a 看向点 b 的方位角。
func angleTo(a, b Point) float64 {
	return math.Atan2(b.Y-a.Y, b.X-a.X)
}

// WrapPi 将角度归一化到 (-pi, pi]。
//
// 注意 -pi 会被映射到 +pi：二者表示同一物理方向，且在“差值”语义下
// 配合 AngleDiff 使用时边界两侧保持连续（见 TestAngleWrapContinuity）。
func WrapPi(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// AngleDiff 返回归一化后的有向角差 a-b，结果落在 (-pi, pi]。
// 这是处理 -pi/pi 环绕的唯一正确入口，禁止直接对两个角度做减法。
func AngleDiff(a, b float64) float64 {
	return WrapPi(a - b)
}

func clamp(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
