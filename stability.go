package main

import "math"

// Stability 是对一条横向误差时间序列的失稳判定结果。
type Stability struct {
	Divergent  bool    `json:"divergent"`
	StartIndex int     `json:"startIndex"` // 发散开始的帧号（-1 表示未发散）
	StartTime  float64 `json:"startTime"`  // 对应时间（s）
	MaxAbs     float64 `json:"maxAbs"`     // 全程 |e| 峰值
	FinalAbs   float64 `json:"finalAbs"`   // 末帧 |e|
	Reason     string  `json:"reason"`
}

// 失稳判据（详见 README）。横向误差发散与"有界振荡"的区别在于：发散时误差
// 包络随时间持续扩张、且仿真结束时仍在扩张；有界振荡/收敛的包络最终走平或缩小。
const (
	// hardLimit 为横向误差硬限（m）：越过即判定发散，无需再看包络。
	defaultHardLimit = 3.0
	// burnSeconds 为起振保护时间：初始误差本身可能很大，不参与包络扩张判定。
	burnSeconds = 1.0
	// blockSeconds 为包络分块长度。
	blockSeconds = 2.0
	// minRunBlocks 为连续扩张所需的最少块数。
	minRunBlocks = 3
	// blockGrowth 为相邻块包络的最小增长比例（>1.2 才算"持续扩张"，
	// 避免把有界振荡中的正常起伏误判为发散）。
	blockGrowth = 1.2
	// breakoutNoise 为突破量噪声底（m）：末块包络至少要超过它才可能判发散。
	breakoutNoise = 0.25
)

// DetectInstability 判定横向误差序列是否发散并定位起始时间步。
// dt 为采样周期，hardLimit<=0 时使用 defaultHardLimit。
//
// 两条判据（取最先触发者）：
//  1. 硬限：|e| 首次越过 hardLimit —— 位移已经明显失控。
//  2. 包络持续扩张：跳过 burn-in 后把序列按 blockSeconds 分块，若存在连续
//     minRunBlocks 个块的最大值以 blockGrowth 倍递增、末块达到全程最大且
//     高于噪声底，则在"误差首次突破扩张前包络水平"的那一帧定位为发散起点。
//     扩张必须在序列末端仍在进行，因此先扩张后重新收敛/走平的有界振荡
//     不会被判为发散。
func DetectInstability(es []float64, dt, hardLimit float64) Stability {
	st := Stability{StartIndex: -1}
	if hardLimit <= 0 {
		hardLimit = defaultHardLimit
	}
	absE := make([]float64, len(es))
	for i, e := range es {
		absE[i] = math.Abs(e)
		if absE[i] > st.MaxAbs {
			st.MaxAbs = absE[i]
		}
	}
	if len(es) > 0 {
		st.FinalAbs = absE[len(absE)-1]
	}

	// 判据一：硬限。
	hardIdx := -1
	for i, a := range absE {
		if a > hardLimit {
			hardIdx = i
			break
		}
	}

	// 判据二：分块包络持续扩张。
	envIdx := envelopeBreakout(absE, dt)

	switch {
	case envIdx >= 0 && (hardIdx < 0 || envIdx <= hardIdx):
		st.Divergent = true
		st.StartIndex = envIdx
		st.Reason = "横向误差包络连续增长"
	case hardIdx >= 0:
		st.Divergent = true
		st.StartIndex = hardIdx
		st.Reason = "横向误差超过硬限"
	default:
		st.Reason = "横向误差收敛或保持有界"
	}
	if st.StartIndex >= 0 {
		st.StartTime = float64(st.StartIndex) * dt
	}
	return st
}

// envelopeBreakout 返回包络扩张判据定位的起始帧，未检出返回 -1。
func envelopeBreakout(absE []float64, dt float64) int {
	n := len(absE)
	burn := int(math.Ceil(burnSeconds / dt))
	w := int(math.Ceil(blockSeconds / dt))
	if burn < 3 {
		burn = 3
	}
	if w < 4 {
		w = 4
	}
	if n-burn < minRunBlocks*w {
		return -1 // 数据太短，无法支撑"持续"扩张的结论
	}
	// 块最大值。末尾不足一块长度的截断块直接丢弃：其包络天然偏低，
	// 会错误打断末端连续扩张链（真实仿真长度一般不会恰好整块结束）。
	var blocks []float64
	for i := burn; i+w <= n; i += w {
		m := 0.0
		for _, a := range absE[i : i+w] {
			if a > m {
				m = a
			}
		}
		blocks = append(blocks, m)
	}
	// 找末端结束的连续扩张段（发散在仿真结束时必须仍在进行）。
	end := len(blocks) - 1
	run := 0
	for k := end; k >= 1; k-- {
		if blocks[k] > blockGrowth*blocks[k-1] {
			run++
			continue
		}
		break
	}
	// run 计数的是"增长边"数，涉及块数 = run+1。
	if run+1 < minRunBlocks {
		return -1
	}
	firstBlock := end - run
	// 末块必须是全程最大值附近（允许 5% 余量）：扩张趋势没有在末端被重新压回。
	globalMax := 0.0
	for _, a := range absE {
		if a > globalMax {
			globalMax = a
		}
	}
	if blocks[end] < breakoutNoise || blocks[end] < 0.95*globalMax {
		return -1
	}
	// 起始帧：扩张段第一块内，误差首次超过扩张前包络（上一块最大值）
	// 的那一帧 —— 即"突破原有振荡包络"的精确时间步。
	prevEnv := 0.0
	if firstBlock > 0 {
		prevEnv = blocks[firstBlock-1]
	} else if burn > 0 {
		for _, a := range absE[:burn] {
			if a > prevEnv {
				prevEnv = a
			}
		}
	}
	startSample := burn + firstBlock*w
	for i := startSample; i < n; i++ {
		if absE[i] > prevEnv {
			return i
		}
	}
	return startSample
}
