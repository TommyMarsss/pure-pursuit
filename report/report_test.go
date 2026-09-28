package report

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/TommyMarsss/pure-pursuit/controller"
	"github.com/TommyMarsss/pure-pursuit/pathgen"
	"github.com/TommyMarsss/pure-pursuit/sim"
)

func TestWriteSingleFileReport(t *testing.T) {
	dir := t.TempDir()
	out := filepath.Join(dir, "nested", "report.html")

	r := &Report{Title: "测试报告"}
	res := sim.Run("pure_pursuit",
		controller.NewPath(pathgen.Straight(6, 0, 200)),
		sim.NewPurePursuit(&controller.PurePursuit{
			Lookahead: 0.6, Wheelbase: 0.5, SearchWindow: 2}),
		sim.Config{
			DT: 0.02, TMax: 6, Speed: 1.0,
			Start: controller.Pose{X: 0, Y: 0.3, Theta: 0.2},
		})
	r.Scenarios = []*Scenario{{Name: "场景一", Description: "单元测试场景", Result: res}}

	if err := r.Write(out); err != nil {
		t.Fatalf("Write: %v", err)
	}

	// 目录下只能有这一个产物文件（单文件报告）。
	var files []string
	err := filepath.Walk(dir, func(p string, info os.FileInfo, err error) error {
		if err != nil {
			return err
		}
		if !info.IsDir() {
			files = append(files, p)
		}
		return nil
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(files) != 1 {
		t.Fatalf("期望仅生成 1 个文件，实际 %v", files)
	}

	data, err := os.ReadFile(out)
	if err != nil {
		t.Fatal(err)
	}
	html := string(data)

	// 数据内嵌：必须包含仿真帧 JSON。
	if !strings.Contains(html, `"frames"`) {
		t.Error("报告中未找到内嵌仿真数据 frames")
	}
	if !strings.Contains(html, "场景一") {
		t.Error("报告中未找到场景名称")
	}
	// 无任何外部依赖：不得出现 http(s) 资源引用或外链标签。
	for _, bad := range []string{"http://", "https://", "<link", "<script src", "cdn."} {
		if strings.Contains(html, bad) {
			t.Errorf("单文件报告包含外部引用 %q", bad)
		}
	}
	// 脚本标签必须正确闭合（JSEscape 后数据内的 </script> 已被转义）。
	if strings.Count(html, "<script>") != 1 || strings.Count(html, "</script>") != 1 {
		t.Error("脚本标签数量异常")
	}
}
