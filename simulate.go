package main

import "math"

// Controller 是两种路径跟踪控制器的统一接口。
// Compute 给定当前位姿、前进速度与参考路径，返回角速度指令 ω、
// 几何参考点（PP 为前视点，Stanley 为切点）以及最近投影。
type Controller interface {
	Compute(pose Pose, v float64, p *Path) (omega float64, target Vec, proj Projection)
	Reset()
}

// Frame 是一次回放帧：位姿、控制量、几何参考点与有符号横向误差。
type Frame struct {
	T  float64 `json:"t"`
	X  float64 `json:"x"`
	Y  float64 `json:"y"`
	Th float64 `json:"th"`
	W  float64 `json:"w"`
	Tx float64 `json:"tx"`
	Ty float64 `json:"ty"`
	E  float64 `json:"e"`
}

// Scenario 是一次完整跟踪仿真的输入与结果。
type Scenario struct {
	Name      string             `json:"name"`
	Algo      string             `json:"algo"`
	Params    map[string]float64 `json:"params"`
	V         float64            `json:"v"`
	Dt        float64            `json:"dt"`
	Path      [][2]float64       `json:"path"`
	Frames    []Frame            `json:"frames"`
	Stability Stability          `json:"stability"`
	Reached   bool               `json:"reached"`
	Bounds    [4]float64         `json:"bounds"` // minX, minY, maxX, maxY

	path *Path
}

// RunConfig 控制仿真终止条件。
type RunConfig struct {
	V, Dt     float64
	MaxTime   float64 // 仿真时长上限（s）
	HardLimit float64 // 失稳硬限（m），<=0 用默认值
	Start     Pose
}

// Run 执行一次闭环仿真：每帧先用控制器算出 ω 并记录，再按 unicycle 运动学推进。
// 机器人距路径终点一个步程以内、且最近投影已接近终点弧长时判定为到达；
// 失稳场景通常到 MaxTime 才停，交给 DetectInstability 判定。
func Run(name, algo string, params map[string]float64, path *Path, ctrl Controller, cfg RunConfig) *Scenario {
	ctrl.Reset()
	sc := &Scenario{
		Name: name, Algo: algo, Params: params, V: cfg.V, Dt: cfg.Dt,
		path: path,
	}
	sc.Path = make([][2]float64, len(path.Pts))
	for i, q := range path.Pts {
		sc.Path[i] = [2]float64{q.X, q.Y}
	}

	end := path.Pts[len(path.Pts)-1]
	pose := cfg.Start
	hard := cfg.HardLimit
	if hard <= 0 {
		hard = defaultHardLimit
	}
	// 越过硬限后再放任运动只会得到无意义的狂奔；误差达到 abortFactor 倍
	// 硬限即中止本帧之后的仿真（仍保留硬限→中止之间的失控段供可视化）。
	const abortFactor = 2.0
	es := []float64{}
	for k := 0; ; k++ {
		t := float64(k) * cfg.Dt
		omega, tgt, proj := ctrl.Compute(pose, cfg.V, path)
		sc.Frames = append(sc.Frames, Frame{
			T: t, X: pose.X, Y: pose.Y, Th: pose.Theta,
			W: omega, Tx: tgt.X, Ty: tgt.Y, E: proj.E,
		})
		es = append(es, proj.E)

		// 到达判据：投影已到末段，且机器人位置进入终点的到达半径。
		// 半径除一个步程外再留 reachTol 余量：Stanley 在恒定曲率路径上
		// 存在小幅弯道稳态横向误差（几何控制器的已知特性），不应视为未到达。
		const reachTol = 0.12
		dEnd := pose.Pos().Distance(end)
		if dEnd <= cfg.V*cfg.Dt+reachTol && proj.S >= path.Total()-0.3 {
			sc.Reached = true
			break
		}
		// 失控中止：|e| 已达硬限的 abortFactor 倍。
		if math.Abs(proj.E) >= abortFactor*hard {
			break
		}
		if t >= cfg.MaxTime-cfg.Dt/2 {
			break
		}
		pose = Step(pose, cfg.V, omega, cfg.Dt)
	}

	sc.Stability = DetectInstability(es, cfg.Dt, cfg.HardLimit)
	sc.Bounds = computeBounds(path, sc.Frames)
	return sc
}

func computeBounds(p *Path, fs []Frame) [4]float64 {
	minX, minY := math.Inf(1), math.Inf(1)
	maxX, maxY := math.Inf(-1), math.Inf(-1)
	upd := func(x, y float64) {
		minX = math.Min(minX, x)
		maxX = math.Max(maxX, x)
		minY = math.Min(minY, y)
		maxY = math.Max(maxY, y)
	}
	for _, q := range p.Pts {
		upd(q.X, q.Y)
	}
	for _, f := range fs {
		upd(f.X, f.Y)
	}
	pad := 0.5
	return [4]float64{minX - pad, minY - pad, maxX + pad, maxY + pad}
}

// ---------------------------------------------------------------------------
// 参考路径构造
// ---------------------------------------------------------------------------

// resample 将稠密折线按固定弧长间距 ds 重采样，首尾点保留。
func resample(raw []Vec, ds float64) []Vec {
	if len(raw) < 2 {
		return raw
	}
	cum := make([]float64, len(raw))
	for i := 1; i < len(raw); i++ {
		cum[i] = cum[i-1] + raw[i].Distance(raw[i-1])
	}
	total := cum[len(cum)-1]
	out := []Vec{raw[0]}
	next := ds
	i := 1
	for next < total {
		for cum[i] < next {
			i++
		}
		lo := cum[i-1]
		seg := cum[i] - lo
		t := 0.0
		if seg > 0 {
			t = (next - lo) / seg
		}
		out = append(out, raw[i-1].Add(raw[i].Sub(raw[i-1]).Scale(t)))
		next += ds
	}
	out = append(out, raw[len(raw)-1])
	return out
}

// LinePath 构造沿 +x 方向的直线路径。
func LinePath(length, ds float64) *Path {
	n := int(math.Round(length/ds)) + 1
	pts := make([]Vec, n)
	for i := 0; i < n; i++ {
		pts[i] = Vec{float64(i) * ds, 0}
	}
	return NewPath(pts)
}

// QuarterCirclePath 构造圆心 (0,R)、半径 R 的四分之一圆弧：
// (0,0) 出发沿 +x，左转弯到 (R,R)，弧长 R·π/2。
func QuarterCirclePath(R, ds float64) *Path {
	n := int(math.Round(R*math.Pi/2/ds)) + 1
	pts := make([]Vec, n)
	for i := 0; i < n; i++ {
		a := float64(i) / float64(n-1) * math.Pi / 2
		pts[i] = Vec{R * math.Sin(a), R * (1 - math.Cos(a))}
	}
	return NewPath(pts)
}

// FullCirclePath 构造以原点为圆心、半径 R 的完整圆（逆时针）：
// 起点 (0,-R)、初始切向 +x，绕行一周回到起点。路径切向在最左点附近
// 从 +π 跳变到 -π，是航向误差角度环绕测试的标准场景。
func FullCirclePath(R, ds float64) *Path {
	n := int(math.Round(2*math.Pi*R/ds)) + 1
	pts := make([]Vec, n)
	for i := 0; i < n; i++ {
		a := float64(i) / float64(n-1) * 2 * math.Pi
		pts[i] = Vec{R * math.Sin(a), -R * math.Cos(a)}
	}
	return NewPath(pts)
}

// HairpinPath 构造发卡弯（U 形掉头）：4 m 直线 → R=1 m 左后转 180° → 4.9 m 反向直线。
// 总长约 12 m，入弯曲率突变（曲率 0→1），用于检验前视点不跳变。
func HairpinPath(ds float64) *Path {
	var raw []Vec
	push := func(q Vec) { raw = append(raw, q) }
	// 入弯直线 (0,0)→(4,0)
	for d := 0.0; d <= 4.0; d += 0.02 {
		push(Vec{d, 0})
	}
	// 半圆：圆心 (4,1)，φ 从 -π/2 到 π/2，入弯切向 +x、出弯切向 -x。
	for f := 0.0; f <= math.Pi; f += 0.02 {
		phi := -math.Pi/2 + f
		push(Vec{4 + math.Cos(phi), 1 + math.Sin(phi)})
	}
	// 出弯直线 (4,2)→(-0.9,2)
	for d := 0.0; d <= 4.9; d += 0.02 {
		push(Vec{4 - d, 2})
	}
	return NewPath(resample(raw, ds))
}

// FigureEightPath 构造自交叉 8 字路径：两个半径 R=2 的圆在原点相切拼接。
// 先绕圆心 (-2,0) 逆时针一周回到原点（切向 +y），再绕圆心 (2,0) 顺时针一周，
// 路径在原点处自交叉，是前视点跨枝跳变的典型考验。
func FigureEightPath(ds float64) *Path {
	var raw []Vec
	push := func(q Vec) { raw = append(raw, q) }
	R := 2.0
	// 第一圈：(-2+2cos t, 2 sin t)，t:0→2π，t=0 位于原点，切向 +y。
	for i := 0; i <= 400; i++ {
		t := float64(i) / 400 * 2 * math.Pi
		push(Vec{-2 + R*math.Cos(t), R * math.Sin(t)})
	}
	// 第二圈：(2-2cos u, 2 sin u)，u:0→2π，u=0 位于原点，切向 +y，向右弯。
	for i := 1; i <= 400; i++ {
		u := float64(i) / 400 * 2 * math.Pi
		push(Vec{2 - R*math.Cos(u), R * math.Sin(u)})
	}
	return NewPath(resample(raw, ds))
}
