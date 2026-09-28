package sim

import (
	"math"
	"time"
)

// divergenceDetector 基于 |e| 的平滑包络判定横向误差是否发散。
//
// 判据（幅值条件满足时，以下任一条成立即确认发散）：
//  1. 硬逃逸：平滑包络 env > HardEscape；
//  2. 持续放大：在 ConfirmWindow 时长内包络单调走高（允许 2% 回撤噪声），
//     且窗口末端相对窗口内最低点放大 GrowthFactor 倍以上，
//     同时末端超过 AbsThreshold（排除小振幅有界振荡）。
//
// “发散开始时间步”定位为逃逸段内包络的最低点（trough）——
// 即有界/收敛状态与逃逸段的分界；持续放大情形取确认窗口内的
// 最低点，硬逃逸情形在更长的 LookbackWindow 内回溯。
type divergenceDetector struct {
	dt      float64
	cfg     DivergenceConfig
	env     []float64 // 每个采样步的平滑包络
	alphaUp float64   // 上行平滑系数
	alphaDn float64   // 下行平滑系数（更慢，使包络贴合峰值）
	wSteps  int
}

// setDefaults 为零值判据参数填入默认值。
func (cfg *DivergenceConfig) setDefaults() {
	if cfg.AbsThreshold <= 0 {
		cfg.AbsThreshold = 0.8
	}
	if cfg.HardEscape <= 0 {
		cfg.HardEscape = 3.0
	}
	if cfg.GrowthFactor <= 0 {
		cfg.GrowthFactor = 2.5
	}
	if cfg.ConfirmWindow <= 0 {
		cfg.ConfirmWindow = 800 * time.Millisecond
	}
	if cfg.SmoothTau <= 0 {
		cfg.SmoothTau = 0.12
	}
	if cfg.LookbackWindow <= 0 {
		cfg.LookbackWindow = 2400 * time.Millisecond
	}
}

func newDivergenceDetector(dt float64, cfg DivergenceConfig) *divergenceDetector {
	// 即便调用方传入零值配置（如单元测试直接构造），也兜底默认值，
	// 保证判据阈值不会退化为“任何非零误差都算发散”。
	cfg.setDefaults()
	a := 1 - math.Exp(-dt/cfg.SmoothTau)
	return &divergenceDetector{
		dt:      dt,
		cfg:     cfg,
		wSteps:  maxInt(2, int(math.Round(cfg.ConfirmWindow.Seconds()/dt))),
		alphaUp: a,
		alphaDn: a * 0.35,
	}
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

// update 喂入第 k 步的 |e|，返回是否确认发散、发散起始步与确认步。
func (d *divergenceDetector) update(k int, absE float64) (confirmed bool, onset int, confirm int) {
	// 快上慢下的单向平滑，使包络贴合峰值而不被振荡谷值打断。
	var e float64
	if len(d.env) == 0 {
		e = absE
	} else {
		prev := d.env[len(d.env)-1]
		a := d.alphaUp
		if absE < prev {
			a = d.alphaDn
		}
		e = prev + a*(absE-prev)
	}
	d.env = append(d.env, e)

	// 硬逃逸：包络大幅超阈直接确认，在回溯窗口内定位起点。
	if e > d.cfg.HardEscape {
		return true, d.locateTrough(k), k
	}

	// 持续放大：检查最近一个确认窗口。
	if len(d.env) >= d.wSteps+1 {
		win := d.env[len(d.env)-d.wSteps-1:]
		head := win[len(win)-1]

		mono := true
		runMax := win[0]
		for i := 1; i < len(win); i++ {
			if win[i] > runMax {
				runMax = win[i]
			} else if runMax-win[i] > 0.02*math.Max(runMax, 1e-9) {
				mono = false
				break
			}
		}

		minV := win[0]
		for i := 1; i < len(win); i++ {
			if win[i] < minV {
				minV = win[i]
			}
		}

		if mono && head > d.cfg.AbsThreshold && head >= d.cfg.GrowthFactor*minV {
			// 确认后在更长的回溯窗口内定位真正的逃逸拐点：
			// 仅取确认窗口最低点会把起点锁定在阈值达成前 0.8s，
			// 而指数从低幅值长到阈值可能远早于该窗口。
			return true, d.locateTrough(k), k
		}
	}
	return false, 0, 0
}

// locateTrough 在最近 LookbackWindow 步内找包络最低点，作为发散起点。
func (d *divergenceDetector) locateTrough(k int) int {
	n := int(math.Round(d.cfg.LookbackWindow.Seconds() / d.dt))
	if n < d.wSteps {
		n = d.wSteps
	}
	if n > k {
		n = k
	}
	start := k - n
	if start < 0 {
		start = 0
	}
	best := k
	bestV := math.Inf(1)
	for i := k; i >= start; i-- {
		if d.env[i] < bestV {
			bestV = d.env[i]
			best = i
		}
	}
	return best
}
