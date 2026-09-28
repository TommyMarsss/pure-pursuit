// Package sim 在差速驱动运动学模型上运行路径跟踪闭环仿真，
// 并提供横向误差发散（失稳）检测。
package sim

import (
	"math"
	"time"

	"github.com/TommyMarsss/pure-pursuit/controller"
)

// Controller 是仿真器使用的控制器接口，由 PurePursuit / Stanley 的
// 适配器实现（见 NewPurePursuit / NewStanley）。
type Controller interface {
	Reset()
	Step(path *controller.Path, pose controller.Pose, v float64) Output
}

// Output 是单步控制输出。TargetX/Y 对 Pure Pursuit 为前视点，
// 对 Stanley 为路径最近点（用于可视化）。
type Output struct {
	OmegaCmd   float64
	Delta      float64
	S          float64
	CrossTrack float64
	HeadingErr float64
	PastEnd    bool
	TargetX    float64
	TargetY    float64
}

// PurePursuitAdapter 包装 controller.PurePursuit。
type PurePursuitAdapter struct{ C *controller.PurePursuit }

func (a *PurePursuitAdapter) Reset() { a.C.Reset() }
func (a *PurePursuitAdapter) Step(p *controller.Path, pose controller.Pose, v float64) Output {
	r := a.C.Compute(p, pose, v)
	return Output{
		OmegaCmd:   r.OmegaCmd,
		Delta:      r.Delta,
		S:          r.Proj.S,
		CrossTrack: r.Proj.CrossTrack,
		HeadingErr: controller.AngleDiff(r.Proj.Heading, pose.Theta),
		PastEnd:    r.Proj.PastEnd,
		TargetX:    r.Target.X,
		TargetY:    r.Target.Y,
	}
}

// StanleyAdapter 包装 controller.Stanley。
type StanleyAdapter struct{ C *controller.Stanley }

func (a *StanleyAdapter) Reset() { a.C.Reset() }
func (a *StanleyAdapter) Step(p *controller.Path, pose controller.Pose, v float64) Output {
	r := a.C.Compute(p, pose, v)
	return Output{
		OmegaCmd:   r.OmegaCmd,
		Delta:      r.Delta,
		S:          r.Proj.S,
		CrossTrack: r.Proj.CrossTrack,
		HeadingErr: r.HeadingErr,
		PastEnd:    r.Proj.PastEnd,
		TargetX:    r.Proj.Point.X,
		TargetY:    r.Proj.Point.Y,
	}
}

// NewPurePursuit 构造 Pure Pursuit 仿真适配器。
func NewPurePursuit(c *controller.PurePursuit) *PurePursuitAdapter {
	return &PurePursuitAdapter{C: c}
}

// NewStanley 构造 Stanley 仿真适配器。
func NewStanley(c *controller.Stanley) *StanleyAdapter {
	return &StanleyAdapter{C: c}
}

// Config 闭环仿真配置。
type Config struct {
	DT          float64 // 仿真步长（秒），默认 0.02
	TMax        float64 // 最大仿真时长（秒），默认 30
	Speed       float64 // 恒定前进速度 v（米/秒）
	SteerLagTau float64 // 转向（角速度）一阶滞后时间常数，0 表示无滞后
	MaxOmega    float64 // 角速度指令绝对值限幅，<=0 不限幅
	Start       controller.Pose
	GoalTol     float64 // 距终点多少米视为到达，默认 0.15
	Detect      DivergenceConfig
}

// DivergenceConfig 失稳（横向误差发散）判据参数。
type DivergenceConfig struct {
	// AbsThreshold 平滑后的 |e| 必须超过该绝对量级（米）才可能判为发散，
	// 用于把“小幅有界振荡”与“逃逸”区分开。默认 0.8。
	AbsThreshold float64
	// HardEscape 平滑 |e| 超过该值（米）无条件判为发散。默认 3。
	HardEscape float64
	// GrowthFactor 确认窗口末端相对始端的放大倍数。默认 2.5。
	GrowthFactor float64
	// ConfirmWindow 持续增长的确认时长，默认 0.8s。
	ConfirmWindow time.Duration
	// SmoothTau |e| 包络平滑时间常数，默认 0.12s。
	SmoothTau float64
	// LookbackWindow 定位发散起点时向前回溯的时长，默认 2.4s。
	LookbackWindow time.Duration
}

func (c *Config) withDefaults() {
	if c.DT <= 0 {
		c.DT = 0.02
	}
	if c.TMax <= 0 {
		c.TMax = 30
	}
	if c.GoalTol <= 0 {
		c.GoalTol = 0.15
	}
	c.Detect.setDefaults()
}

// Frame 一帧仿真记录。
type Frame struct {
	T          float64 `json:"t"`
	X          float64 `json:"x"`
	Y          float64 `json:"y"`
	Theta      float64 `json:"th"`
	OmegaCmd   float64 `json:"wc"`
	OmegaAct   float64 `json:"wa"`
	Delta      float64 `json:"d"`
	CTE        float64 `json:"e"`
	TargetX    float64 `json:"tx"`
	TargetY    float64 `json:"ty"`
	HeadingErr float64 `json:"psi"`
}

// Result 一次完整仿真的结果。
type Result struct {
	Algorithm string             `json:"algorithm"`
	Params    map[string]float64 `json:"params"`
	Speed     float64            `json:"speed"`
	DT        float64            `json:"dt"`
	Path      []controller.Point `json:"path"`
	Frames    []Frame            `json:"frames"`

	ReachedGoal bool `json:"reachedGoal"`
	// Divergent 横向误差是否被判定为发散。
	Divergent bool `json:"divergent"`
	// DivergeStep 发散开始的帧下标（逃逸段的包络最低点）。
	DivergeStep int `json:"divergeStep"`
	// DivergeTime 发散开始的仿真时间（秒）。
	DivergeTime float64 `json:"divergeTime"`
	// ConfirmStep 判据确认发散的帧下标（用于解释检测延迟）。
	ConfirmStep int `json:"confirmStep"`
	// MaxAbsCTE 全程最大绝对横向误差。
	MaxAbsCTE float64 `json:"maxAbsCTE"`
	// FinalCTE 最后一帧横向误差。
	FinalCTE float64 `json:"finalCTE"`
	// Reason 结论的文字说明。
	Reason string `json:"reason"`
}

// Run 执行一次闭环仿真。algorithm 仅用于结果标注（"pure_pursuit" / "stanley"）。
func Run(algorithm string, path *controller.Path, ctl Controller, cfg Config) *Result {
	cfg.withDefaults()
	det := newDivergenceDetector(cfg.DT, cfg.Detect)

	res := &Result{
		Algorithm:   algorithm,
		Speed:       cfg.Speed,
		DT:          cfg.DT,
		Path:        path.Pts,
		Params:      map[string]float64{"speed": cfg.Speed, "steerLagTau": cfg.SteerLagTau, "maxOmega": cfg.MaxOmega, "dt": cfg.DT},
		DivergeStep: -1,
		ConfirmStep: -1,
	}
	ctl.Reset()

	pose := cfg.Start
	omegaAct := 0.0
	end := path.At(path.Length())

	record := func(t, omegaCmd float64, out Output) {
		e := out.CrossTrack
		res.Frames = append(res.Frames, Frame{
			T: t, X: pose.X, Y: pose.Y, Theta: pose.Theta,
			OmegaCmd: omegaCmd, OmegaAct: omegaAct, Delta: out.Delta,
			CTE: e, TargetX: out.TargetX, TargetY: out.TargetY,
			HeadingErr: out.HeadingErr,
		})
		if ae := math.Abs(e); ae > res.MaxAbsCTE {
			res.MaxAbsCTE = ae
		}
	}

	stopAfter := -1.0 // 发散确认后再多录一小段，让曲线展示逃逸过程
	for k := 0; ; k++ {
		t := float64(k) * cfg.DT
		if t > cfg.TMax+cfg.DT/2 {
			break
		}
		out := ctl.Step(path, pose, cfg.Speed)
		omegaCmd := out.OmegaCmd
		if cfg.MaxOmega > 0 {
			omegaCmd = clampF(omegaCmd, -cfg.MaxOmega, cfg.MaxOmega)
		}
		record(t, omegaCmd, out)

		// 失稳检测（在记录之后、积分之前）。
		if !res.Divergent {
			if confirmed, onset, confirm := det.update(k, math.Abs(out.CrossTrack)); confirmed {
				res.Divergent = true
				res.DivergeStep = onset
				res.ConfirmStep = confirm
				res.DivergeTime = float64(onset) * cfg.DT
				stopAfter = t + 0.8
				res.Reason = "横向误差包络在确认窗口内持续放大并超过阈值"
			}
		}

		// 到达终点。
		if !res.Divergent && out.PastEnd && math.Hypot(pose.X-end.X, pose.Y-end.Y) < cfg.GoalTol {
			res.ReachedGoal = true
			res.Reason = "到达路径终点，横向误差有界"
			break
		}
		if res.Divergent && stopAfter >= 0 && t >= stopAfter {
			break
		}

		// 一阶转向滞后：omega_act 以时间常数 SteerLagTau 跟踪指令。
		if cfg.SteerLagTau > 0 {
			a := 1 - math.Exp(-cfg.DT/cfg.SteerLagTau)
			omegaAct += a * (omegaCmd - omegaAct)
		} else {
			omegaAct = omegaCmd
		}
		if cfg.MaxOmega > 0 {
			omegaAct = clampF(omegaAct, -cfg.MaxOmega, cfg.MaxOmega)
		}

		// 单轮模型积分（角度用半隐式，位置用平均航向，减小欧拉误差）。
		thNew := controller.WrapPi(pose.Theta + omegaAct*cfg.DT)
		pose.X += cfg.Speed * math.Cos((pose.Theta+thNew)/2) * cfg.DT
		pose.Y += cfg.Speed * math.Sin((pose.Theta+thNew)/2) * cfg.DT
		pose.Theta = thNew
	}

	if n := len(res.Frames); n > 0 {
		res.FinalCTE = res.Frames[n-1].CTE
	}
	if !res.Divergent && !res.ReachedGoal && res.Reason == "" {
		res.Reason = "仿真到时：横向误差未发散，但未在时限内到达终点"
	}
	return res
}

func clampF(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
