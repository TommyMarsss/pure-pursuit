package sim

import (
	"math"
	"testing"

	"github.com/TommyMarsss/pure-pursuit/controller"
	"github.com/TommyMarsss/pure-pursuit/pathgen"
)

// settleCTE 返回仿真后半段的最大 |e|，用于判定“收敛并保持有界”。
func settleCTE(r *Result) float64 {
	start := len(r.Frames) / 2
	m := 0.0
	for _, f := range r.Frames[start:] {
		if a := math.Abs(f.CTE); a > m {
			m = a
		}
	}
	return m
}

func TestConvergenceLineAndCircle(t *testing.T) {
	dt := 0.02
	const pi = math.Pi

	line := controller.NewPath(pathgen.Straight(14, 0, 500))
	arc := controller.NewPath(pathgen.CircleArc(0, 4, 4, -pi/2, -pi/2+200*pi/180, 700))

	cases := []struct {
		name string
		path *controller.Path
		ctl  Controller
	}{
		{"line/pure-pursuit", line, NewPurePursuit(&controller.PurePursuit{
			Lookahead: 0.6, Wheelbase: 0.5, SearchWindow: 2})},
		{"line/stanley", line, NewStanley(&controller.Stanley{
			K: 1.0, KSoft: 0.1, Wheelbase: 0.5, SearchWindow: 2})},
		{"arc/pure-pursuit", arc, NewPurePursuit(&controller.PurePursuit{
			Lookahead: 0.7, K: 0.15, Wheelbase: 0.5, SearchWindow: 2})},
		{"arc/stanley", arc, NewStanley(&controller.Stanley{
			K: 2.5, KSoft: 0.1, Wheelbase: 0.5, SearchWindow: 2})},
	}

	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			cfg := Config{
				DT: dt, TMax: 16, Speed: 1.3, SteerLagTau: 0.05, MaxOmega: 3,
				Start: controller.Pose{X: 0.2, Y: 0.5, Theta: 0.35},
			}
			r := Run("test", c.path, c.ctl, cfg)
			if r.Divergent {
				t.Fatalf("被误判为发散，起始步 %d (%.2fs)", r.DivergeStep, r.DivergeTime)
			}
			if !r.ReachedGoal {
				t.Fatalf("未在时限内到达终点: %s，末态 |e|=%.3f", r.Reason, math.Abs(r.FinalCTE))
			}
			if m := settleCTE(r); m > 0.12 {
				t.Fatalf("收敛后稳态横向误差过大: %.3f m", m)
			}
		})
	}
}

// 失稳配置必须被检测为发散，且定位到的发散起始步位于真正的逃逸段：
// onset 之前 |e| 应较小，onset 之后应出现显著放大。
func TestDivergenceDetected(t *testing.T) {
	// Pure Pursuit：短前视 + 高速 + 大滞后。
	ppRun := Run("pure_pursuit",
		controller.NewPath(pathgen.Straight(60, 0, 600)),
		NewPurePursuit(&controller.PurePursuit{
			Lookahead: 0.15, Wheelbase: 0.5, SearchWindow: 3}),
		Config{
			DT: 0.02, TMax: 8, Speed: 3.5, SteerLagTau: 0.5,
			Start: controller.Pose{X: 0, Y: 0.3, Theta: 0},
		})
	if !ppRun.Divergent {
		t.Fatalf("PP 失稳配置未被检测到，max|e|=%.3f", ppRun.MaxAbsCTE)
	}
	checkOnset(t, "PP", ppRun)

	// Stanley：横向增益过大 + 滞后。
	stRun := Run("stanley",
		controller.NewPath(pathgen.Straight(60, 0, 600)),
		NewStanley(&controller.Stanley{
			K: 9, KSoft: 0.1, Wheelbase: 0.5, SearchWindow: 3}),
		Config{
			DT: 0.02, TMax: 8, Speed: 2.2, SteerLagTau: 0.25,
			Start: controller.Pose{X: 0, Y: 0.3, Theta: 0},
		})
	if !stRun.Divergent {
		t.Fatalf("Stanley 失稳配置未被检测到，max|e|=%.3f", stRun.MaxAbsCTE)
	}
	checkOnset(t, "Stanley", stRun)
}

func checkOnset(t *testing.T, name string, r *Result) {
	t.Helper()
	if r.DivergeStep < 0 || r.DivergeStep >= len(r.Frames) {
		t.Fatalf("%s 发散起始步非法: %d", name, r.DivergeStep)
	}
	if r.ConfirmStep < r.DivergeStep {
		t.Fatalf("%s 确认步 %d 早于起始步 %d", name, r.ConfirmStep, r.DivergeStep)
	}
	// 起始步处 |e| 应明显小于全程最大（定位到的是逃逸起点而非峰值）。
	eAt := math.Abs(r.Frames[r.DivergeStep].CTE)
	if eAt > 0.7*r.MaxAbsCTE {
		t.Fatalf("%s 起始步误差 %.3f 已接近峰值 %.3f，定位过晚",
			name, eAt, r.MaxAbsCTE)
	}
	// 起始步之后 1.5s 内必须能看到明显增长，否则“起点”定在收敛段里。
	end := r.DivergeStep + int(1.5/r.DT)
	if end >= len(r.Frames) {
		end = len(r.Frames) - 1
	}
	maxAfter := 0.0
	for _, f := range r.Frames[r.DivergeStep:end] {
		if a := math.Abs(f.CTE); a > maxAfter {
			maxAfter = a
		}
	}
	if maxAfter < 1.6*math.Max(eAt, 0.15) {
		t.Fatalf("%s 起始步后未见明显放大: at=%.3f after=%.3f", name, eAt, maxAfter)
	}
}

// 有界振荡（极限环）不得误报为发散：构造一组保持小幅振荡但不放大的参数。
func TestBoundedOscillationNotFlagged(t *testing.T) {
	// 中等前视 + 适度滞后会产生轻微振荡但摆幅有界。
	r := Run("pure_pursuit",
		controller.NewPath(pathgen.Straight(80, 0, 800)),
		NewPurePursuit(&controller.PurePursuit{
			Lookahead: 0.45, Wheelbase: 0.5, SearchWindow: 3}),
		Config{
			DT: 0.02, TMax: 12, Speed: 2.2, SteerLagTau: 0.18,
			Start: controller.Pose{X: 0, Y: 0.25, Theta: 0},
		})
	if r.Divergent {
		t.Fatalf("有界振荡被误判为发散，onset=%d (%.2fs)", r.DivergeStep, r.DivergeTime)
	}
	// 确认它确实不是单调收敛（确为振荡），并确有摆幅，避免测试空洞。
	pos, neg := 0.0, 0.0
	for _, f := range r.Frames {
		if f.CTE > pos {
			pos = f.CTE
		}
		if f.CTE < neg {
			neg = f.CTE
		}
	}
	if pos < 0.04 || neg > -0.04 {
		t.Fatalf("该参数未产生预期的两侧振荡: max=%.3f min=%.3f", pos, neg)
	}
}

// 急弯掉头（U 形路径）闭环跟踪：前视点选取在大曲率处不得跳变，
// 表现为相邻帧角速度指令无突变、轨迹保持在路径附近且能到达终点。
func TestSharpTurnNoCommandJump(t *testing.T) {
	path := controller.NewPath(pathgen.UTurn(4, 4, 1.2, 300))
	ctl := NewPurePursuit(&controller.PurePursuit{
		Lookahead: 0.55, K: 0.1, Wheelbase: 0.5,
		SearchWindow: 1.2, MaxSteer: 0.9,
	})
	r := Run("pure_pursuit", path, ctl, Config{
		DT: 0.02, TMax: 16, Speed: 1.0, SteerLagTau: 0.06, MaxOmega: 2.2,
		Start: controller.Pose{X: 0, Y: 0.25, Theta: 0},
	})
	if r.Divergent {
		t.Fatalf("急弯跟踪被误判发散，onset=%d", r.DivergeStep)
	}
	if !r.ReachedGoal {
		t.Fatalf("急弯路径未到达终点: %s", r.Reason)
	}
	if r.MaxAbsCTE > 0.4 {
		t.Fatalf("急弯处横向误差过大: %.3f m", r.MaxAbsCTE)
	}
	// 相邻帧指令跳变上限：v*kappa 在 20ms 内变化不应超过约 1.2 rad/s，
	// 前视点若在折回直道之间跳变，会产生远超该值的突跳。
	maxJump := 0.0
	for i := 1; i < len(r.Frames); i++ {
		if j := math.Abs(r.Frames[i].OmegaCmd - r.Frames[i-1].OmegaCmd); j > maxJump {
			maxJump = j
		}
	}
	if maxJump > 1.2 {
		t.Fatalf("急弯处角速度指令最大帧间跳变 %.3f rad/s，疑似前视点跳变", maxJump)
	}
}

func TestDivergenceDetectorOnSyntheticSeries(t *testing.T) {
	d := newDivergenceDetector(0.02, DivergenceConfig{})
	confirmed := false
	onset := -1
	confirmK := -1
	const onsetTrue = 200
	for k := 0; k < 400; k++ {
		var e float64
		if k < onsetTrue {
			e = 0.1 * math.Abs(math.Sin(float64(k)*0.3)) // 有界小振荡
		} else {
			e = 0.08 * math.Exp(0.025*float64(k-onsetTrue)) // 指数逃逸
		}
		if c, o, ck := d.update(k, e); c {
			confirmed, onset, confirmK = true, o, ck
			break
		}
	}
	if !confirmed {
		t.Fatal("指数逃逸未被检测到")
	}
	if onset < onsetTrue-25 || onset > onsetTrue+25 {
		t.Fatalf("起始步定位 %d 偏离真实拐点 %d 超过 ±25 步", onset, onsetTrue)
	}
	if confirmK < onset {
		t.Fatalf("确认步 %d 早于起始步 %d", confirmK, onset)
	}
}

// 全程有界的合成序列不得触发。
func TestDivergenceDetectorBoundedSynthetic(t *testing.T) {
	d := newDivergenceDetector(0.02, DivergenceConfig{})
	for k := 0; k < 500; k++ {
		e := 0.3 + 0.15*math.Sin(float64(k)*0.2) // 恒定有界振荡
		if c, _, _ := d.update(k, e); c {
			t.Fatalf("恒定有界振荡在第 %d 步被误判", k)
		}
	}
}
