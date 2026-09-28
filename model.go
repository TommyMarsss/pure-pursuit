package main

import "math"

// Pose 为差速驱动机器人（unicycle 运动学）位姿：车体参考点坐标与航向角（rad）。
type Pose struct {
	X, Y, Theta float64
}

func (p Pose) Pos() Vec { return Vec{p.X, p.Y} }

// Step 按单轮（unicycle）运动学积分一个时间步：
//
//	ẋ = v cosθ,  ẏ = v sinθ,  θ̇ = ω
//
// v 为前进线速度（仿真中取恒定正值），ω 为控制器输出的角速度（转向控制量）。
// dt 较小（0.05~0.1 s），欧拉积分精度足够。
func Step(p Pose, v, omega, dt float64) Pose {
	return Pose{
		X:     p.X + v*math.Cos(p.Theta)*dt,
		Y:     p.Y + v*math.Sin(p.Theta)*dt,
		Theta: p.Theta + omega*dt,
	}
}
