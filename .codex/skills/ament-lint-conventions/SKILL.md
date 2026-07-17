---
name: ament-lint-conventions
description: 'Fix ament lint failures in ROS 2 packages. Use when colcon test reports pep257 D213/D400/D415/D205/D209 docstring violations (especially with Chinese punctuation), flake8 F401 after deleting code, uncrustify divergence on new C++ files, cppcheck skipped tests, or set(ENV{PYTEST_ADDOPTS}) in CMakeLists having no effect.'
---

# ament lint 约定速查（Humble / ament_lint 0.12.x 实测）

> 来源：2026-07 lab_cobot_ws 新增 C++ gtest 与重写 Python 测试时的实测记录。
> ament 的 lint 全家桶约定与常见默认不同，新文件几乎必踩，本册按"症状→根因→修法"组织。

## 1. pep257（ament_pep257）

| 症状 | 根因 | 修法 |
|---|---|---|
| `D213: Multi-line docstring summary should start at the second line` | ament 约定集选了 D213（与常见 D212"摘要在第一行"**相反**） | 多行 docstring 摘要放第二行；或直接单行化 |
| 全中文 docstring 报 `D400/D415: First line should end with a period` | 检查器只认 ASCII 句点 `.`，**中文句号"。"不满足**【实测】 | docstring 用英文以 `.` 结尾；中文细节放 `#` 注释 |
| `D205`（摘要与正文间缺空行）/`D209`（多行结尾 `"""` 未单独成行）连环报 | 多行 docstring 的格式要求成套 | 最省事套路见下 |

**实测最省事套路**（一次通过，不与 D 系列缠斗）：

```python
def test_xxx():
    """Keep the breakaway force fuse disabled by default."""
    # 中文详细说明放普通注释:位姿驱动底盘下正常搬运的 ERP 校正力
    # 与异常力同量级,启用会把物块半路丢掉(实测),故必须保持禁用。
    ...
```

即：**docstring 一律单行英文句点结尾，所有中文解释降级为 `#` 注释**。模块级 docstring 同理。

## 2. flake8（ament_flake8）

| 症状 | 根因 | 修法 |
|---|---|---|
| 删除死代码后 `F401 'xxx' imported but unused` | launch 文件删了某段（如 TimerAction 包的动作）后 import 残留 | 删代码时同步清 import；提交前 `python3 -m flake8 <file> --select F401` 快查【实测】 |
| ament_python 包想让 flake8 明确覆盖 test/ 和 setup.py | 默认调用 `main_with_errors(argv=[])` 的扫描范围依赖 cwd | 显式传路径：`main_with_errors(argv=[str(pkg_dir), str(pkg_dir/"test"), str(pkg_dir/"setup.py")])`【本仓实测通过】 |

## 3. uncrustify（ament_uncrustify）

| 症状 | 根因 | 修法 |
|---|---|---|
| 新 C++ 文件 `Code style divergence` | ament 的 uncrustify 配置对花括号/空格有自己的约定 | **不要手改**：`ament_uncrustify --reformat <files>` 自动修复后重跑【实测：一条命令解决】 |

## 4. cppcheck（ament_cppcheck）

| 症状 | 根因 | 修法/认知 |
|---|---|---|
| 新 C++ 文件在 colcon test 里显示 **skipped**（不是 passed） | 本机 cppcheck 版本对该文件跳过分析【原因未验证，疑与 cppcheck 版本过旧有关】 | ⚠️ 认知要点：**skip ≠ 已检查**。"0 failures" 汇总里藏着未做静态分析的新代码；给关键逻辑补 gtest 行为测试兜底，别依赖 cppcheck |

## 5. CMake / 测试注册

| 症状 | 根因 | 修法 |
|---|---|---|
| CMakeLists 里 `set(ENV{PYTEST_ADDOPTS} "-p no:anyio")` 不生效 | 这是 **configure 期**赋值，测试运行期环境不继承【实测：删除后行为不变】 | 用 `ament_add_pytest_test(... ENV "PYTEST_ADDOPTS=-p no:anyio" ...)` 的 ENV 参数 |
| 长 E2E 混在目录级注册里以 ctest 超时收场，失败原因失真 | 注册的 TIMEOUT 是整轮预算，< 测试自身内部超时 | E2E 单独注册（`--ignore=` 从目录注册排除），TIMEOUT ≥ 内部预算总和【实测：360 装不下 420+80】 |
| ament_python 包（manipulation/perception 类）被加 ament_add_pytest_test 后测试重复跑 | ament_python 的 test/ 由 colcon 自动执行 | 只有 **ament_cmake** 包需要显式 `find_package(ament_cmake_pytest)` + `ament_add_pytest_test`；ament_python 包不要加 |
| 新增 gtest 后 CMake 配置失败 | 缺 find_package | `find_package(ament_cmake_gtest REQUIRED)` + `ament_add_gtest(name test/xxx.cpp)` + package.xml 补 `<test_depend>ament_cmake_gtest</test_depend>`【实测配方】 |

## 6. 常规豁免与 skip 来源（认知）

- `set(ament_cmake_copyright_FOUND TRUE)` / `set(ament_cmake_cpplint_FOUND TRUE)`：包内无版权头时的标准豁免（configure 期对 `_FOUND` 变量赋值**有效**——它影响的是 find 逻辑，不是运行期环境，与第 5 节 ENV 的失效情形不同）。
- colcon 汇总里的 skipped 常见构成：ament_python 包的 `test_copyright`（pytest.skip）+ cppcheck 对 C++ 文件的版本性跳过。审阅测试报告时先弄清 skip 是什么，再解读"全绿"。

## 7. 提交前最小自查

```bash
python3 -m flake8 <改动的py> --select F401          # import 残留
ament_uncrustify <改动的cpp/hpp>                     # C++ 风格(加 --reformat 自动修)
python3 -m pytest <包>/test/test_pep257.py -p no:anyio -q   # docstring 约定
colcon test --packages-select <包> && colcon test-result --test-result-base build/<包> --verbose
```
