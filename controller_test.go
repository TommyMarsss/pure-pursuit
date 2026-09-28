package main

import (
	"math"
	"testing"
)

const (
	testV  = 1.2
	testDt = 0.05
)

// runClosedLoop 以给定控制器在路径上执行闭环仿真，返回横向误差序列与帧序列。
func runClosedLoop(t *testing.T, p *Path, c Controller, start Pose, maxSteps int) ([]float64, []Frame) {
	t.Helper()
	c.Reset()
	pose := start
	es := make([]float64, 0, maxSteps)
	frames := make([]Frame, 0, maxSteps)
	end := p.Pts[len(p.Pts)-1]
	for k := 0; k < maxSteps; k++ {
		w, tgt, proj := c.Compute(pose, testV, p)
		frames = append(frames, Frame{
			T: float64(k) * testDt, X: pose.X, Y: pose.Y, Th: pose.Theta,
			W: w, Tx: tgt.X, Ty: tgt.Y, E: proj.E,
		})
		es = append(es, proj.E)
		if pose.Pos().Distance(end) <= testV*testDt+0.12 && proj.S >= p.Total()-0.3 {
			break
		}
		pose = Step(pose, testV, w, testDt)
	}
	return es, frames
}

func maxAbs(es []float64) float64 {
	m := 0.0
	for _, e := range es {
		if a := math.Abs(e); a > m {
			m = a
		}
	}
	return m
}

func tailMax(es []float64, sec float64) float64 {
	n := int(sec / testDt)
	if n > len(es) {
		n = len(es)
	}
	return maxAbs(es[len(es)-n:])
}

// ---------------------------------------------------------------- 直线收敛

func TestStraightLineConvergence(t *testing.T) {
	p := LinePath(12, 0.1)
	start := Pose{0, 0.6, 0.1}
	controllers := map[string]Controller{
		"pure-pursuit": NewPurePursuit(LookaheadPolicy{LdMin: 0.8, LdMax: 0.8}),
		"stanley":      NewStanley(StanleyParams{K: 1.2, Ks: 0.1, L: 0.4}),
	}
	for name, c := range controllers {
		t.Run(name, func(t *testing.T) {
			es, frames := runClosedLoop(t, p, c, start, 600)
			if len(frames) >= 600 {
				t.Fatal("未在限定步数内到达终点")
			}
			if got := tailMax(es, 1.0); got > 0.02 {
				t.Errorf("末段 |e| 峰值 = %.4f，应收敛到 0.02 以内", got)
			}
			if got := maxAbs(es); got > 0.8 {
				t.Errorf("全程 |e| 峰值 = %.4f，超调超过 0.8", got)
			}
			if DetectInstability(es, testDt, 3).Divergent {
				t.Error("直线跟踪被误判为发散")
			}
		})
	}
}

// ---------------------------------------------------------------- 圆弧收敛

func TestCircularArcConvergence(t *testing.T) {
	p := QuarterCirclePath(5, 0.1)
	start := Pose{0, -0.4, 0.25}
	cases := map[string]Controller{
		"pure-pursuit": NewPurePursuit(LookaheadPolicy{LdMin: 0.9, LdMax: 0.9}),
		"stanley":      NewStanley(StanleyParams{K: 1.0, Ks: 0.1, L: 0.4}),
	}
	for name, c := range cases {
		t.Run(name, func(t *testing.T) {
			es, frames := runClosedLoop(t, p, c, start, 600)
			if len(frames) >= 600 {
				t.Fatal("未在限定步数内到达圆弧终点")
			}
			// 恒定曲率下允许保持小幅稳态横向误差（弯道几何特性）。
			if got := tailMax(es, 1.0); got > 0.12 {
				t.Errorf("末段 |e| 峰值 = %.4f，应在 0.12 以内", got)
			}
			if DetectInstability(es, testDt, 3).Divergent {
				t.Error("圆弧跟踪被误判为发散")
			}
		})
	}
}

// ------------------------------------------------ 大曲率/自交叉下前视点不跳变

// TestLookaheadNoJumps 校验前视点在发卡弯与 8 字路径上沿弧长单调推进，
// 且帧间目标位置变化平滑（曲率突变与自交叉点都不允许跳变）。
func TestLookaheadNoJumps(t *testing.T) {
	cases := []struct {
		name  string
		path  *Path
		start Pose
	}{
		{"发卡弯", HairpinPath(0.1), Pose{0, -0.5, 0}},
		{"自交叉8字", FigureEightPath(0.1), Pose{0, -0.4, 0}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			c := NewPurePursuit(LookaheadPolicy{LdMin: 1.0, LdMax: 1.0})
			c.Reset()
			pose := tc.start
			var prevGoal Vec
			havePrev := false
			maxStep := 0.0
			end := tc.path.Pts[len(tc.path.Pts)-1]
			reached := false
			for k := 0; k < 1000; k++ {
				w, g, _ := c.Compute(pose, testV, tc.path)
				if havePrev {
					step := g.Distance(prevGoal)
					if step > maxStep {
						maxStep = step
					}
					// 正常帧间前视点位移约 v·dt=0.06 m；允许控制瞬态到
					// 0.35 m，跨枝跳变通常是数米量级。
					if step > 0.35 {
						t.Errorf("k=%d 前视点跳变 %.3f m：%v → %v（弧长 sGoal=%.3f）",
							k, step, prevGoal, g, c.sGoal)
					}
				}
				prevGoal, havePrev = g, true
				if pose.Pos().Distance(end) <= testV*testDt+0.12 {
					reached = true
					break
				}
				pose = Step(pose, testV, w, testDt)
			}
			if !reached {
				t.Fatal("未到达终点")
			}
			if maxStep > 0.35 {
				t.Errorf("最大帧间前视点位移 %.3f m", maxStep)
			}
		})
	}
}

// TestLookaheadJunctionArc 定点校验：机器人在第一圈尾段临近交叉点时，
// 前视点必须选在"去路"（第二圈 x>0 的右转弯枝），不能选回来路或终点。
func TestLookaheadJunctionArc(t *testing.T) {
	p := FigureEightPath(0.1)
	c := NewPurePursuit(LookaheadPolicy{LdMin: 1.2, LdMax: 1.2})
	c.Reset()
	pose := Pose{0, -0.4, 0}
	for k := 0; k < 215; k++ {
		w, _, _ := c.Compute(pose, testV, p)
		pose = Step(pose, testV, w, testDt)
	}
	// 此刻机器人在交叉点南侧重返原点途中；再跑若干帧，前视点应进入第二圈。
	sawSecondLoop := false
	for k := 0; k < 40; k++ {
		w, g, proj := c.Compute(pose, testV, p)
		// 第二圈首段的点 x>0、y≥0；来路（第一圈尾段）点 x<0。
		if proj.S > 12.57 && g.X > 0.05 {
			sawSecondLoop = true
		}
		pose = Step(pose, testV, w, testDt)
	}
	if !sawSecondLoop {
		t.Error("通过自交叉点后前视点未进入第二圈（发生了跨枝/终点跳变）")
	}
}

// ------------------------------------------------------------- 人为失稳检测

func TestUnstableParametersDetected(t *testing.T) {
	p := LinePath(40, 0.1)

	t.Run("pure-pursuit Ld过小", func(t *testing.T) {
		c := NewPurePursuit(LookaheadPolicy{LdMin: 0.06, LdMax: 0.06})
		c.Reset()
		pose := Pose{0, 0.3, 0.3}
		var es []float64
		for k := 0; k < 300; k++ {
			w, _, proj := c.Compute(pose, 2.5, p)
			es = append(es, proj.E)
			pose = Step(pose, 2.5, w, 0.15)
		}
		r := DetectInstability(es, 0.15, 3)
		if !r.Divergent {
			t.Fatalf("Ld=0.06 的失稳配置未被检出（峰值 %.3f）", r.MaxAbs)
		}
		if r.StartIndex <= 0 || r.StartTime <= 0 {
			t.Error("发散起始时间步未定位")
		}
		// 定位的起始点之后误差包络必须真的在扩张。
		later := maxAbs(es[len(es)-10:])
		if later < math.Abs(es[r.StartIndex]) {
			t.Error("定位的发散起点之后误差并未增长")
		}
	})

	t.Run("stanley L过小k过大", func(t *testing.T) {
		// v/L 增益过大 + 粗采样导致离散闭环失稳（与饱和数值问题无关）。
		c := NewStanley(StanleyParams{K: 3, Ks: 0.1, L: 0.03})
		c.Reset()
		pose := Pose{0, 0.2, 0.2}
		var es []float64
		for k := 0; k < 200; k++ {
			w, _, proj := c.Compute(pose, 2.0, p)
			es = append(es, proj.E)
			pose = Step(pose, 2.0, w, 0.25)
		}
		r := DetectInstability(es, 0.25, 3)
		if !r.Divergent {
			t.Fatalf("L=0.03,k=3 的失稳配置未被检出（峰值 %.3f）", r.MaxAbs)
		}
		if r.StartIndex < 0 {
			t.Error("发散起始时间步未定位")
		}
	})
}

// ------------------------------------------------- 角度环绕边界控制量连续

// TestWrapPi 直接校验角度归一化。
func TestWrapPi(t *testing.T) {
	cases := []struct{ in, want float64 }{
		{3 * math.Pi / 2, -math.Pi / 2},
		{-3 * math.Pi / 2, math.Pi / 2},
		{math.Pi, math.Pi},
		{-math.Pi, math.Pi},
		{2*math.Pi + 0.1, 0.1},
		{-2*math.Pi - 0.1, -0.1},
	}
	for _, tc := range cases {
		if got := wrapPi(tc.in); math.Abs(got-tc.want) > 1e-12 {
			t.Errorf("wrapPi(%v) = %v，期望 %v", tc.in, got, tc.want)
		}
	}
}

// TestHeadingWrapControlContinuous：Stanley 在整圆跟踪中航向与路径切向
// 同时穿越 ±π，控制量 ω 必须连续（直接做角度差会出现约 ±2π 的跳变）。
func TestHeadingWrapControlContinuous(t *testing.T) {
	p := FullCirclePath(3, 0.05)
	c := NewStanley(StanleyParams{K: 1.0, Ks: 0.1, L: 0.4})
	c.Reset()
	// 从路径起点 (0,-R) 出发，初始切向 +x。
	pose := Pose{0, -3, 0}
	var omegas []float64
	maxJump := 0.0
	for k := 0; k < 500; k++ {
		w, _, proj := c.Compute(pose, testV, p)
		if len(omegas) > 0 {
			if j := math.Abs(w - omegas[len(omegas)-1]); j > maxJump {
				maxJump = j
			}
		}
		omegas = append(omegas, w)
		if proj.S >= p.Total()-0.2 {
			break
		}
		pose = Step(pose, testV, w, testDt)
	}
	// 正常帧间 ω 变化在 0.1 rad/s 量级；角度环绕错误会造成 v/L*tan(±2π)
	// 附近以外的突变，这里把阈值放到 0.8 仍能抓到环绕跳变。
	if maxJump > 0.8 {
		t.Errorf("航向穿越 ±π 时 ω 跳变 %.3f rad/s，控制量不连续", maxJump)
	}
	// 圆周跟踪稳态 ω 应接近 v/R = 0.4 rad/s（左转为正）。
	n := 20
	if n > len(omegas) {
		n = len(omegas)
	}
	sum := 0.0
	for _, w := range omegas[len(omegas)-n:] {
		sum += w
	}
	if mean := sum / float64(n); math.Abs(mean-testV/3) > 0.1 {
		t.Errorf("圆周稳态 ω=%.3f，期望约 %.3f", mean, testV/3)
	}
}

// TestCrossTrackSignAcrossTangent：机器人从路径一侧匀速横移越过切点，
// 有符号横向误差必须平滑变号、atan 横向项随之平滑反向，不允许在切点
// 附近因角度计算出现符号翻转。
func TestCrossTrackSignAcrossTangent(t *testing.T) {
	// 水平直线路径 y=0，机器人沿 y 从 -0.5 走到 +0.5。
	pts := []Vec{{0, 0}, {10, 0}}
	p := NewPath(pts)
	c := NewStanley(StanleyParams{K: 1.5, Ks: 0.1, L: 0.5})
	c.Reset()
	prevW := 0.0
	for i := 0; i <= 100; i++ {
		y := -0.5 + float64(i)/100.0
		// 航向沿 +x，与路径切向一致，消除航向项、只看横向项。
		pose := Pose{1.0, y, 0}
		w, _, proj := c.Compute(pose, testV, p)
		switch {
		case y < -1e-3:
			// 机器人在路径南侧（右侧）：路径在机器人左侧，E>0，
			// 横向项应给左转（ω>0）把机器人拉回去。
			if proj.E <= 0 || w <= 0 {
				t.Errorf("y=%.3f: E=%.4f、w=%.4f，符号应为正", y, proj.E, w)
			}
		case y > 1e-3:
			if proj.E >= 0 || w >= 0 {
				t.Errorf("y=%.3f: E=%.4f、w=%.4f，符号应为负", y, proj.E, w)
			}
		}
		if i > 0 && math.Abs(w-prevW) > 0.2 {
			t.Errorf("y=%.3f 附近 ω 不连续：%.4f → %.4f", y, prevW, w)
		}
		prevW = w
	}
}

// TestStanleySaturationContinuous：横向误差从 0 连续增大到使 δ 进入饱和，
// ω 必须有限、符号一致且在饱和边界处不发生跳变（回归 tan/sin 两支不连续
// 的历史缺陷）。
func TestStanleySaturationContinuous(t *testing.T) {
	pts := []Vec{{0, 0}, {20, 0}}
	p := NewPath(pts)
	c := NewStanley(StanleyParams{K: 5, Ks: 0.05, L: 0.1})
	c.Reset()
	var prevW float64
	for i := 1; i <= 400; i++ {
		y := float64(i) * 0.02 // 机器人在路径北侧（左侧）越偏越远
		pose := Pose{2.0, y, 0}
		w, _, _ := c.Compute(pose, 0.5, p)
		if math.IsNaN(w) || math.IsInf(w, 0) {
			t.Fatalf("y=%.2f 时 ω 非有限值", y)
		}
		if w > 0 {
			t.Fatalf("y=%.2f：路径在机器人右侧，ω 应为负，得到 %.3f", y, w)
		}
		if i > 1 && prevW-w < -0.5 {
			t.Errorf("y=%.2f 进入饱和时 ω 反向跳变：%.4f → %.4f", y, prevW, w)
		}
		prevW = w
	}
}

// ------------------------------------------------- 失稳判据的合成序列覆盖

func TestEnvelopeDivergenceDetection(t *testing.T) {
	dt := 0.1
	// 慢指数发散：前 10 s 在 ±0.12 内有界振荡，之后包络按约 1.23 倍/2s
	// 扩张，全程峰值约 2.7 m（不触发 3 m 硬限）——专门覆盖"包络连续
	// 增长"判据及其起始定位。
	var es []float64
	const k0 = 100
	perBlock := 1.23
	perSample := math.Pow(perBlock, 1.0/20.0)
	// 长度取 burn 之后的整块数，保证末端分块完整（截断块的包络偏低会
	// 打断末端扩张链）：10 + 19×20 = 390 个样本。
	const n = 390
	for k := 0; k < n; k++ {
		tm := float64(k) * dt
		amp := 0.12
		if k > k0 {
			amp = 0.12 * math.Pow(perSample, float64(k-k0))
		}
		es = append(es, amp*math.Sin(2*math.Pi*tm/1.2))
	}
	if m := maxAbs(es); m > 3 {
		t.Fatalf("测试序列峰值 %.2f 应小于硬限，改用更小增长率", m)
	}
	r := DetectInstability(es, dt, 3)
	if !r.Divergent {
		t.Fatalf("慢发散序列未被检出（峰值 %.3f）", maxAbs(es))
	}
	if r.Reason != "横向误差包络连续增长" {
		t.Fatalf("应由包络增长判据检出，实际：%q", r.Reason)
	}
	// 起点应落在扩张段（t>10s），且早于序列末端。
	if r.StartTime < 10 || r.StartTime > 38 {
		t.Errorf("发散起点 t=%.2f 不在合理区间", r.StartTime)
	}
}

func TestBoundedOscillationNotDivergent(t *testing.T) {
	dt := 0.1
	var es []float64
	for k := 0; k < 400; k++ {
		tm := float64(k) * dt
		// 包络先增大后回落、最终保持有界，不构成末端持续扩张。
		amp := 0.2 + 0.5*math.Exp(-(tm-8)*(tm-8)/30)
		es = append(es, amp*math.Sin(2*math.Pi*tm/1.5))
	}
	if r := DetectInstability(es, dt, 3); r.Divergent {
		t.Fatalf("有界振荡被误判为发散（起点 %d, %s）", r.StartIndex, r.Reason)
	}
}

func TestConvergingErrorNotDivergent(t *testing.T) {
	dt := 0.1
	var es []float64
	for k := 0; k < 300; k++ {
		tm := float64(k) * dt
		es = append(es, 0.6*math.Exp(-0.25*tm)*math.Sin(2*tm))
	}
	if r := DetectInstability(es, dt, 3); r.Divergent {
		t.Fatal("收敛误差被误判为发散")
	}
}

// ------------------------------------------------- 终点附近前视处理

func TestLookaheadNearEnd(t *testing.T) {
	// 短直线 + 大前视距离：机器人从一开始就以前视圆锁定终点，
	// 校验 (a) 进入前视圆后目标始终是终点（越过终点附近也不会在来路上
	// 重新找交点反向打舵），(b) 机器人最终到达终点。
	p := LinePath(2, 0.1)
	ld := 2.5
	c := NewPurePursuit(LookaheadPolicy{LdMin: ld, LdMax: ld})
	c.Reset()
	pose := Pose{0, 0.3, 0}
	end := p.Pts[len(p.Pts)-1]
	reached := false
	for k := 0; k < 400; k++ {
		w, g, _ := c.Compute(pose, testV, p)
		if pose.Pos().Distance(end) <= ld && g != end {
			t.Fatalf("k=%d 终点已在前视圆内，目标应为终点，得到 %v（位姿 %v）", k, g, pose)
		}
		if pose.Pos().Distance(end) <= testV*testDt+0.12 {
			reached = true
			break
		}
		pose = Step(pose, testV, w, testDt)
	}
	if !reached {
		t.Fatal("未能到达终点")
	}

	// 正常长度路径：终点进入前视圆前目标沿弧平滑推进，进入瞬间锁定终点，
	// 锁定前后目标点位移不发生跳变。
	p2 := LinePath(6, 0.1)
	c2 := NewPurePursuit(LookaheadPolicy{LdMin: 1.0, LdMax: 1.0})
	c2.Reset()
	pose = Pose{0, 0.2, 0}
	end2 := p2.Pts[len(p2.Pts)-1]
	var prevG Vec
	maxLockJump := 0.0
	reached = false
	for k := 0; k < 600; k++ {
		w, g, _ := c2.Compute(pose, testV, p2)
		if k > 0 && g.Distance(prevG) > maxLockJump {
			maxLockJump = g.Distance(prevG)
		}
		prevG = g
		if pose.Pos().Distance(end2) <= testV*testDt+0.12 {
			reached = true
			break
		}
		pose = Step(pose, testV, w, testDt)
	}
	if !reached {
		t.Fatal("正常长度路径未能到达终点")
	}
	if maxLockJump > 0.2 {
		t.Errorf("终点锁定瞬间前视点跳变 %.3f m", maxLockJump)
	}
}
