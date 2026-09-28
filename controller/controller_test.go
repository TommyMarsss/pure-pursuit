package controller

import (
	"math"
	"testing"
)

const eps = 1e-9

func approxEq(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s = %v, want %v (tol %v)", name, got, want, tol)
	}
}

// ---------------------------------------------------------------------------
// 角度环绕
// ---------------------------------------------------------------------------

func TestWrapPiBoundaries(t *testing.T) {
	cases := []struct{ in, want float64 }{
		{0, 0},
		{2.5, 2.5},
		{-2.5, -2.5},
		{3.2, 3.2 - 2*math.Pi},
		{-3.2, -3.2 + 2*math.Pi},
		{math.Pi, math.Pi},
		{-math.Pi, math.Pi}, // -pi 与 +pi 同一方向，归一到 (+pi] 侧
		{3 * math.Pi, math.Pi},
		{-3 * math.Pi, math.Pi},
	}
	for _, c := range cases {
		got := WrapPi(c.in)
		if math.Abs(got-c.want) > 1e-12 || got <= -math.Pi || got > math.Pi+1e-12 {
			t.Errorf("WrapPi(%v) = %v, want %v in (-pi,pi]", c.in, got, c.want)
		}
	}
}

// 同一物理航向用 +pi 与 -pi 两种表示时，航向误差必须几乎相同，
// 不允许出现约 2*pi 的控制量跳变（需求二/测试：角度环绕边界处连续）。
func TestHeadingErrorWrapContinuity(t *testing.T) {
	path := NewPath([]Point{{0, 0}, {10, 0}})
	st := &Stanley{K: 1.0, KSoft: 0.1, Wheelbase: 0.5, SearchWindow: 2}

	// 路径切向为 0；机器人真实朝向略偏 -x 一侧（约 178.9°）。
	// 同一物理方向的两种数值表示：pi-0.02 与 -pi-0.02（后者 wrap 后相同），
	// 航向误差与转向指令必须一致，不能在 ±pi 两侧差出 2*pi。
	r1 := st.Compute(path, Pose{X: 1, Y: 0.1, Theta: math.Pi - 0.02}, 1.0)
	r2 := st.Compute(path, Pose{X: 1, Y: 0.1, Theta: -math.Pi - 0.02}, 1.0)
	if math.Abs(r1.HeadingErr-r2.HeadingErr) > 0.05 {
		t.Fatalf("跨越 ±pi 边界航向误差不连续: %v vs %v（差 %v）",
			r1.HeadingErr, r2.HeadingErr, r1.HeadingErr-r2.HeadingErr)
	}
	if math.Abs(r1.Delta-r2.Delta) > 0.05 {
		t.Fatalf("跨越 ±pi 边界转向指令突变: %v vs %v", r1.Delta, r2.Delta)
	}

	// 连续扫描：路径切向与机器人航向同步从 pi-0.15 扫到 pi+0.15，
	// 二者保持恒定小误差 psi=0.1（机器人整体扫过 ±pi 表示接缝）。
	// 正确包裹时 delta 在接缝两侧几乎不变；若直接相减，
	// delta 会在接缝处出现约 2*pi 的突跳。
	var prev float64
	for i := 0; i <= 30; i++ {
		h := math.Pi - 0.15 + 0.3*float64(i)/30 // 路径切向（不取包裹）
		t0 := Point{}
		t1 := Point{X: math.Cos(h), Y: math.Sin(h)}
		seg := NewPath([]Point{t0, t1})
		normal := Point{X: -math.Sin(h), Y: math.Cos(h)} // 左法向
		robot := Point{X: 0.5*t1.X + 0.1*normal.X, Y: 0.5*t1.Y + 0.1*normal.Y}
		theta := h - 0.1 // psi 恒为 0.1
		r := st.Compute(seg, Pose{X: robot.X, Y: robot.Y, Theta: theta}, 1.0)
		if math.Abs(r.HeadingErr-0.1) > 1e-9 {
			t.Fatalf("扫过 ±pi 接缝时 psi 跳变: h=%.3f psi=%.4f", h, r.HeadingErr)
		}
		if i > 0 && math.Abs(r.Delta-prev) > 1e-6 {
			t.Fatalf("航向表示扫过 ±pi 接缝时 delta 突变: h=%.3f delta %.6f -> %.6f",
				h, prev, r.Delta)
		}
		prev = r.Delta
	}

	// 让路径航向本身扫过 ±pi：切向角取 +pi 与 -pi 是同一条直线。
	pathNegX := NewPath([]Point{{10, 0}, {0, 0}}) // 切向 = pi
	for _, psi := range []float64{3.13, 3.14, -3.14, -3.13} {
		r := st.Compute(pathNegX, Pose{X: 5, Y: 0, Theta: psi}, 1.0)
		want := AngleDiff(math.Pi, psi)
		approxEq(t, "heading err near wrap", r.HeadingErr, want, 1e-9)
	}
}

// ---------------------------------------------------------------------------
// Stanley 横向误差符号与转向方向
// ---------------------------------------------------------------------------

func TestStanleyCrossTrackSign(t *testing.T) {
	path := NewPath([]Point{{0, 0}, {10, 0}})
	st := &Stanley{K: 2.0, KSoft: 0.1, Wheelbase: 0.5, SearchWindow: 2}

	// 机器人在路径左侧（y>0）：e>0，必须向右打舵（delta<0）。
	left := st.Compute(path, Pose{X: 2, Y: 0.5, Theta: 0}, 1.0)
	if left.Proj.CrossTrack <= 0 || left.CrossTerm >= 0 || left.Delta >= 0 {
		t.Fatalf("左侧误差符号/方向错误: e=%v cross=%v delta=%v",
			left.Proj.CrossTrack, left.CrossTerm, left.Delta)
	}

	// 右侧（y<0）：e<0，向左打舵（delta>0）。
	right := st.Compute(path, Pose{X: 2, Y: -0.5, Theta: 0}, 1.0)
	if right.Proj.CrossTrack >= 0 || right.CrossTerm <= 0 || right.Delta <= 0 {
		t.Fatalf("右侧误差符号/方向错误: e=%v cross=%v delta=%v",
			right.Proj.CrossTrack, right.CrossTerm, right.Delta)
	}

	// 越过切点前后：最近点沿 x 方向移动，y 不变，符号与转向方向必须一致。
	l1 := st.Compute(path, Pose{X: 1.999, Y: 0.5, Theta: 0}, 1.0)
	l2 := st.Compute(path, Pose{X: 2.001, Y: 0.5, Theta: 0}, 1.0)
	if l1.Delta*l2.Delta <= 0 || math.Abs(l1.Delta-l2.Delta) > 0.02 {
		t.Fatalf("越过切点前后转向方向翻转/突变: %v -> %v", l1.Delta, l2.Delta)
	}
}

// ---------------------------------------------------------------------------
// Pure Pursuit 前视点：终点截断
// ---------------------------------------------------------------------------

func TestPurePursuitLookaheadBeyondEnd(t *testing.T) {
	path := NewPath([]Point{{0, 0}, {5, 0}})
	pp := &PurePursuit{Lookahead: 1.5, Wheelbase: 0.5, SearchWindow: 2}

	// 剩余路径 0.3m < 前视 1.5m：前视点必须被夹到终点而非外推。
	r := pp.Compute(path, Pose{X: 4.7, Y: 0.0, Theta: 0}, 1.0)
	if math.Abs(r.TargetS-path.Length()) > 1e-9 {
		t.Fatalf("终点附近前视弧长 = %v, want %v", r.TargetS, path.Length())
	}
	if r.Target != (Point{5, 0}) {
		t.Fatalf("前视点应为终点 (5,0)，得到 %+v", r.Target)
	}
	if math.IsNaN(r.Kappa) || math.IsInf(r.Kappa, 0) {
		t.Fatalf("终点截断后曲率非有限值: %v", r.Kappa)
	}

	// 已到达终点：曲率仍应有限，指令为沿切向收尾而非崩溃。
	r2 := pp.Compute(path, Pose{X: 5, Y: 0, Theta: 0.05}, 1.0)
	if math.IsNaN(r2.OmegaCmd) || math.IsInf(r2.OmegaCmd, 0) {
		t.Fatalf("终点处 omega 非有限值: %v", r2.OmegaCmd)
	}
}

// ---------------------------------------------------------------------------
// Pure Pursuit 前视点：自交叉处不跳变
// ---------------------------------------------------------------------------

// figureEight 生成 x=A sin(u), y=A sin(u)cos(u), u∈[0,2pi] 的自交叉 8 字，
// 返回点与对应参数 u（采样时 s≈u，非严格弧长，仅用于本测试构造）。
func figureEight(A float64, n int) []Point {
	pts := make([]Point, n)
	for i := 0; i < n; i++ {
		u := 2 * math.Pi * float64(i) / float64(n-1)
		pts[i] = Point{X: A * math.Sin(u), Y: A * math.Sin(u) * math.Cos(u)}
	}
	return pts
}

func TestPurePursuitTargetMonotonicAtSelfCrossing(t *testing.T) {
	pts := figureEight(3.0, 1400)
	path := NewPath(pts)
	pp := &PurePursuit{Lookahead: 0.6, Wheelbase: 0.5, SearchWindow: 1.5}

	// 让机器人位姿严格沿参考路径运动（开环位姿序列），控制器输出
	// 前视点弧长。8 字在 u=pi 处与起点自交叉；全局最近点在该处会
	// 二义，带弧长窗口与单调约束的选取必须始终留在当前环上。
	var prevLookS float64
	maxJumpBack := 0.0
	maxJumpFwd := 0.0
	crossingS := -1.0
	for i := 0; i < len(pts)-2; i += 2 {
		p := pts[i]
		q := pts[i+1]
		th := math.Atan2(q.Y-p.Y, q.X-p.X)
		r := pp.Compute(path, Pose{X: p.X, Y: p.Y, Theta: th}, 1.0)
		if i > 0 {
			if d := prevLookS - r.TargetS; d > maxJumpBack {
				maxJumpBack = d
			}
			// 正常每帧前视点只随机器人前进约一个采样间隔（~0.02m），
			// 若向前瞬移超过 0.5m，说明窗口失效、前视点搭上了交叉路径
			// 的未来段（单调约束挡不住“向前跳”）。
			if d := r.TargetS - prevLookS; d > maxJumpFwd {
				maxJumpFwd = d
			}
		}
		// 自交叉点（原点，u≈pi，即序列中点附近）：离散采样不精确落在
		// 原点，在中点附近窗口内取距原点最近的一点记录其投影弧长。
		if i > len(pts)*4/10 && i < len(pts)*6/10 {
			if d := math.Hypot(p.X, p.Y); d < 0.05 {
				crossingS = r.Proj.S
			}
		}
		prevLookS = r.TargetS
	}
	if maxJumpBack > 1e-6 {
		t.Fatalf("前视点弧长出现回跳，最大回跳 %.4f m（自交叉处跳到另一截路径）", maxJumpBack)
	}
	if maxJumpFwd > 0.5 {
		t.Fatalf("前视点弧长向前瞬移 %.3f m，大曲率/自交叉处发生跳变", maxJumpFwd)
	}
	if crossingS < 0 {
		t.Fatal("测试路径未经过自交叉点，用例本身失效")
	}
	// 交叉点位于路径后半程（弧长约一半以上），证明跟踪点没有退回到起点环。
	if crossingS < path.Length()/2-0.5 {
		t.Fatalf("经过自交叉点时投影弧长 %.3f 退回前半程，发生跳变", crossingS)
	}
}
