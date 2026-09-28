package main

import (
	_ "embed"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"strings"
	"time"
)

func main() {
	out := flag.String("o", "report.html", "输出的静态 HTML 报告路径")
	flag.Parse()

	scenarios := buildScenarios()

	// 命令行摘要：便于在 CI 中快速确认各场景判定结果。
	for _, sc := range scenarios {
		status := "收敛/有界"
		if sc.Stability.Divergent {
			status = fmt.Sprintf("发散 (起始帧 %d, t=%.2fs, %s)",
				sc.Stability.StartIndex, sc.Stability.StartTime, sc.Stability.Reason)
		}
		fmt.Printf("%-34s | %-12s | 末误差 %8.4f | 峰值 %8.4f | %s\n",
			sc.Name, sc.Algo, sc.Stability.FinalAbs, sc.Stability.MaxAbs, status)
	}

	html, err := RenderReport(scenarios, time.Now())
	if err != nil {
		fmt.Fprintln(os.Stderr, "生成报告失败:", err)
		os.Exit(1)
	}
	if err := os.WriteFile(*out, html, 0o644); err != nil {
		fmt.Fprintln(os.Stderr, "写入报告失败:", err)
		os.Exit(1)
	}
	fmt.Println("\n报告已写入:", *out)
}

func buildScenarios() []*Scenario {
	const dt = 0.05
	const ds = 0.1
	v := 1.2

	var all []*Scenario

	// 一、直线：两种控制器均应收敛。
	line12 := LinePath(12, ds)
	all = append(all, Run(
		"直线 · Pure Pursuit", "pure-pursuit",
		map[string]float64{"Ld": 0.8},
		line12,
		NewPurePursuit(LookaheadPolicy{Kf: 0, LdMin: 0.8, LdMax: 0.8}),
		RunConfig{V: v, Dt: dt, MaxTime: 30, Start: Pose{0, 0.6, 0.1}, HardLimit: 3},
	))
	all = append(all, Run(
		"直线 · Stanley", "stanley",
		map[string]float64{"L": 0.4, "k": 1.2, "vSoft": 0.1},
		line12,
		NewStanley(StanleyParams{K: 1.2, Ks: 0.1, L: 0.4}),
		RunConfig{V: v, Dt: dt, MaxTime: 30, Start: Pose{0, 0.6, 0.1}, HardLimit: 3},
	))

	// 二、四分之一圆弧 R=5：持续曲率下收敛到恒定小横向误差。
	arc := QuarterCirclePath(5, ds)
	all = append(all, Run(
		"圆弧 R=5m · Pure Pursuit", "pure-pursuit",
		map[string]float64{"Ld": 0.9},
		arc,
		NewPurePursuit(LookaheadPolicy{Kf: 0, LdMin: 0.9, LdMax: 0.9}),
		RunConfig{V: v, Dt: dt, MaxTime: 30, Start: Pose{0, -0.4, 0.25}, HardLimit: 3},
	))
	all = append(all, Run(
		"圆弧 R=5m · Stanley", "stanley",
		map[string]float64{"L": 0.4, "k": 1, "vSoft": 0.1},
		arc,
		NewStanley(StanleyParams{K: 1, Ks: 0.1, L: 0.4}),
		RunConfig{V: v, Dt: dt, MaxTime: 30, Start: Pose{0, -0.4, 0.25}, HardLimit: 3},
	))

	// 三、发卡弯：入弯曲率由 0 突变为 1，检验前视点不跳变、转向指令不突变。
	hair := HairpinPath(ds)
	all = append(all, Run(
		"发卡弯 · Pure Pursuit", "pure-pursuit",
		map[string]float64{"Ld": 1.0},
		hair,
		NewPurePursuit(LookaheadPolicy{Kf: 0, LdMin: 1.0, LdMax: 1.0}),
		RunConfig{V: v, Dt: dt, MaxTime: 40, Start: Pose{0, -0.5, 0}, HardLimit: 3},
	))
	all = append(all, Run(
		"发卡弯 · Stanley", "stanley",
		map[string]float64{"L": 0.5, "k": 1.5, "vSoft": 0.1},
		hair,
		NewStanley(StanleyParams{K: 1.5, Ks: 0.1, L: 0.5}),
		RunConfig{V: v, Dt: dt, MaxTime: 40, Start: Pose{0, -0.5, 0}, HardLimit: 3},
	))

	// 四、自交叉 8 字：前视点在交叉点附近不得跳到另一枝。
	fig8 := FigureEightPath(ds)
	all = append(all, Run(
		"自交叉8字 · Pure Pursuit", "pure-pursuit",
		map[string]float64{"Ld": 1.2},
		fig8,
		NewPurePursuit(LookaheadPolicy{Kf: 0, LdMin: 1.2, LdMax: 1.2}),
		RunConfig{V: v, Dt: dt, MaxTime: 45, Start: Pose{0, -0.4, 0}, HardLimit: 3},
	))

	// 五、人为不稳定参数：必须被检测为发散。
	line40 := LinePath(40, ds)
	all = append(all, Run(
		"失稳案例 · Pure Pursuit (Ld 过小)", "pure-pursuit",
		map[string]float64{"Ld": 0.06},
		line40,
		NewPurePursuit(LookaheadPolicy{Kf: 0, LdMin: 0.06, LdMax: 0.06}),
		RunConfig{V: 2.5, Dt: 0.15, MaxTime: 45, Start: Pose{0, 0.3, 0.3}, HardLimit: 3},
	))
	all = append(all, Run(
		"失稳案例 · Stanley (L 过小/k 过大)", "stanley",
		map[string]float64{"L": 0.03, "k": 3, "vSoft": 0.1},
		line40,
		NewStanley(StanleyParams{K: 3, Ks: 0.1, L: 0.03}),
		RunConfig{V: 2, Dt: 0.25, MaxTime: 50, Start: Pose{0, 0.2, 0.2}, HardLimit: 3},
	))

	return all
}

//go:embed report_template.html
var reportTemplate string

type reportData struct {
	Generated string      `json:"generated"`
	BuiltAt   string      `json:"builtAt"`
	Scenarios []*Scenario `json:"scenarios"`
}

// RenderReport 用仿真结果填充模板，输出单一自包含静态 HTML。
func RenderReport(scenarios []*Scenario, built time.Time) ([]byte, error) {
	d := reportData{
		Generated: "path-tracking report",
		BuiltAt:   built.Format("2006-01-02 15:04:05"),
		Scenarios: scenarios,
	}
	raw, err := json.Marshal(d)
	if err != nil {
		return nil, err
	}
	out := []byte(reportTemplate)
	// json.Marshal 默认把 < > & 转义为  形式，
	// 因此 JSON 中不会出现字面量 "</script>"，可安全内联。
	out = []byte(strings.ReplaceAll(reportTemplate, "__REPORT_DATA__", string(raw)))
	return out, nil
}
