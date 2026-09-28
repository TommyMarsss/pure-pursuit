// Command demo 运行一组路径跟踪场景（直线/圆弧/急弯/自交叉/失稳配置），
// 并生成单文件 HTML 报告到 output/tracking_report.html。
package main

import (
	"fmt"
	"log"

	"github.com/TommyMarsss/pure-pursuit/controller"
	"github.com/TommyMarsss/pure-pursuit/pathgen"
	"github.com/TommyMarsss/pure-pursuit/report"
	"github.com/TommyMarsss/pure-pursuit/sim"
)

type scenario struct {
	name string
	desc string
	algo string
	path []controller.Point
	ctl  sim.Controller
	cfg  sim.Config
}

func main() {
	dt := 0.02

	scenarios := []scenario{
		{
			name: "直线-Pure Pursuit",
			desc: "参考路径为 12m 直线，机器人初始横向偏差 0.5m、航向偏差 20°。" +
				"前视距离 0.6m，误差应单调收敛并保持有界。",
			algo: "pure_pursuit",
			path: pathgen.Straight(12, 0, 400),
			ctl: sim.NewPurePursuit(&controller.PurePursuit{
				Lookahead: 0.6, Wheelbase: 0.5, SearchWindow: 1.5,
			}),
			cfg: sim.Config{
				DT: dt, TMax: 10, Speed: 1.5, SteerLagTau: 0.05, MaxOmega: 3,
				Start: controller.Pose{X: 0, Y: 0.5, Theta: 20 * 3.14159265 / 180},
			},
		},
		{
			name: "直线-Stanley",
			desc: "同一直线与初始偏差，Stanley 增益 k=1.0。航向误差项与横向误差项" +
				"共同作用，误差指数收敛，无稳态偏差。",
			algo: "stanley",
			path: pathgen.Straight(12, 0, 400),
			ctl: sim.NewStanley(&controller.Stanley{
				K: 1.0, KPsi: 1.0, KSoft: 0.1, Wheelbase: 0.5, SearchWindow: 1.5,
			}),
			cfg: sim.Config{
				DT: dt, TMax: 10, Speed: 1.5, SteerLagTau: 0.05, MaxOmega: 3,
				Start: controller.Pose{X: 0, Y: 0.5, Theta: 20 * 3.14159265 / 180},
			},
		},
		{
			name: "圆弧-Pure Pursuit",
			desc: "半径 4m 的 200° 圆弧（路径航向扫过 ±π 边界）。前视点沿弧长" +
				"单调推进，稳态横向误差有界。",
			algo: "pure_pursuit",
			path: pathgen.CircleArc(0, 4, 4, -3.14159265/2, -3.14159265/2+200*3.14159265/180, 600),
			ctl: sim.NewPurePursuit(&controller.PurePursuit{
				Lookahead: 0.7, K: 0.15, Wheelbase: 0.5, SearchWindow: 2,
			}),
			cfg: sim.Config{
				DT: dt, TMax: 14, Speed: 1.2, SteerLagTau: 0.05, MaxOmega: 2.5,
				Start: controller.Pose{X: 0.3, Y: 0.3, Theta: 0.2},
			},
		},
		{
			name: "圆弧-Stanley",
			desc: "同一 200° 圆弧。航向角在路径中途越过 ±π，wrapPi 处理保证" +
				"航向误差与转向指令在边界两侧连续。",
			algo: "stanley",
			path: pathgen.CircleArc(0, 4, 4, -3.14159265/2, -3.14159265/2+200*3.14159265/180, 600),
			ctl: sim.NewStanley(&controller.Stanley{
				K: 1.2, KPsi: 1.0, KSoft: 0.1, Wheelbase: 0.5, SearchWindow: 2,
			}),
			cfg: sim.Config{
				DT: dt, TMax: 14, Speed: 1.2, SteerLagTau: 0.05, MaxOmega: 2.5,
				Start: controller.Pose{X: 0.3, Y: 0.3, Theta: 0.2},
			},
		},
		{
			name: "急弯掉头-Pure Pursuit",
			desc: "进直道 + 半径 1.2m 的 180° 半圆急弯 + 出直道。前视点搜索限制" +
				"在最近弧长 ±1.2m 窗口内并强制弧长单调，大曲率处前视点不会" +
				"跳到折回的另一段直道，转向指令无突变。",
			algo: "pure_pursuit",
			path: pathgen.UTurn(4, 4, 1.2, 260),
			ctl: sim.NewPurePursuit(&controller.PurePursuit{
				Lookahead: 0.55, K: 0.1, Wheelbase: 0.5, SearchWindow: 1.2, MaxSteer: 0.9,
			}),
			cfg: sim.Config{
				DT: dt, TMax: 14, Speed: 1.0, SteerLagTau: 0.06, MaxOmega: 2.2,
				Start: controller.Pose{X: 0, Y: 0.25, Theta: 0},
			},
		},
		{
			name: "8字自交叉-Pure Pursuit",
			desc: "路径在原点自交叉。第二圈经过交叉点时，弧长窗口 + 单调约束" +
				"保证跟踪点仍在当前这一截路径上，不会回溯到第一圈的交叉段。",
			algo: "pure_pursuit",
			path: pathgen.FigureEight(3, 700),
			ctl: sim.NewPurePursuit(&controller.PurePursuit{
				Lookahead: 0.6, K: 0.1, Wheelbase: 0.5, SearchWindow: 1.5, MaxSteer: 0.9,
			}),
			cfg: sim.Config{
				DT: dt, TMax: 16, Speed: 1.1, SteerLagTau: 0.05, MaxOmega: 2.5,
				Start: controller.Pose{X: 0.0, Y: -0.2, Theta: 1.45},
			},
		},
		{
			name: "失稳-Pure Pursuit（前视过短+高速+大滞后）",
			desc: "人为不稳定配置：前视距离仅 0.15m、速度 3.5m/s、转向滞后 0.5s。" +
				"纯跟踪回路增益 ∝ 2v/Ld² 过大且执行器相位滞后严重，修正方向时" +
				"总是过度，横向误差在约 4.3s 后摆幅指数放大，检测器应判定发散。",
			algo: "pure_pursuit",
			path: pathgen.Straight(60, 0, 600),
			ctl: sim.NewPurePursuit(&controller.PurePursuit{
				Lookahead: 0.15, Wheelbase: 0.5, SearchWindow: 3,
			}),
			cfg: sim.Config{
				DT: dt, TMax: 8, Speed: 3.5, SteerLagTau: 0.5, MaxOmega: 0,
				Start: controller.Pose{X: 0, Y: 0.3, Theta: 0},
			},
		},
		{
			name: "失稳-Stanley（增益过大+滞后）",
			desc: "人为不稳定配置：横向增益 k=9、速度 2.2m/s、转向滞后 0.25s。" +
				"横向回路增益过高导致左右过修正，误差振荡发散，应被检测到。",
			algo: "stanley",
			path: pathgen.Straight(40, 0, 800),
			ctl: sim.NewStanley(&controller.Stanley{
				K: 9, KPsi: 1, KSoft: 0.1, Wheelbase: 0.5, SearchWindow: 3,
			}),
			cfg: sim.Config{
				DT: dt, TMax: 12, Speed: 2.2, SteerLagTau: 0.25, MaxOmega: 6,
				Start: controller.Pose{X: 0, Y: 0.3, Theta: 0},
			},
		},
	}

	rep := &report.Report{Title: "差速驱动机器人路径跟踪控制器仿真报告"}
	for i := range scenarios {
		sc := &scenarios[i]
		p := controller.NewPath(sc.path)
		res := sim.Run(sc.algo, p, sc.ctl, sc.cfg)
		rep.Scenarios = append(rep.Scenarios, &report.Scenario{
			Name: sc.name, Description: sc.desc, Result: res,
		})
		fmt.Printf("[%d] %-28s divergent=%-5v reached=%-5v max|e|=%.3f onset=%s\n",
			i+1, sc.name, res.Divergent, res.ReachedGoal, res.MaxAbsCTE,
			onsetText(res))
	}

	const out = "output/tracking_report.html"
	if err := rep.Write(out); err != nil {
		log.Fatalf("write report: %v", err)
	}
	fmt.Println("报告已生成:", out)
}

func onsetText(r *sim.Result) string {
	if !r.Divergent {
		return "-"
	}
	return fmt.Sprintf("step#%d(%.2fs)", r.DivergeStep, r.DivergeTime)
}
