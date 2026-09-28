// Package report 把一次或多次跟踪仿真结果生成为单一静态 HTML 文件：
// 数据与全部交互逻辑内嵌，不启动服务、不依赖任何第三方库，
// 浏览器直接打开即可逐帧回放并查看横向误差曲线。
package report

import (
	"encoding/json"
	"html/template"
	"os"
	"path/filepath"
	"strings"

	"github.com/TommyMarsss/pure-pursuit/sim"
)

// Scenario 报告中的一个仿真场景。
type Scenario struct {
	Name        string `json:"name"`
	Description string `json:"description"`
	*sim.Result
}

// Report 一次报告包含的全部场景。
type Report struct {
	Title     string
	Scenarios []*Scenario
}

// Write 将报告写入 outPath（自动创建父目录）。
func (r *Report) Write(outPath string) error {
	if err := os.MkdirAll(filepath.Dir(outPath), 0o755); err != nil {
		return err
	}
	blob, err := json.Marshal(struct {
		Scenarios []*Scenario `json:"scenarios"`
	}{Scenarios: r.Scenarios})
	if err != nil {
		return err
	}
	// 数据以 JS 对象字面量直接内嵌。JSON 是合法的 JS 字面量子集，
	// 只需把 "</" 序列转义为 '<\/'（JS 字符串中二者等价），防止数据中
	// 的 "</script>" 提前闭合脚本标签，其余字符原样可读。
	safe := strings.ReplaceAll(string(blob), "</", `<\/`)
	dataJS := template.JS(safe)

	t := template.Must(template.New("report").Parse(htmlTemplate))
	f, err := os.Create(outPath)
	if err != nil {
		return err
	}
	defer f.Close()
	return t.Execute(f, map[string]any{
		"Title":  r.Title,
		"DataJS": dataJS,
	})
}
