---
name: ros2-launch-introspection-testing
description: 'Rewrite launch-file text-assertion tests as real introspection tests. Use when testing generate_launch_description() output, reading Node arguments/parameters, unwrapping TimerAction/RegisterEventHandler/OnProcessExit chains, performing substitutions, accessing ExecuteProcess cmd/env, or when pytest runs a test function that no longer exists in the source file.'
---

# ROS 2 launch 内省测试速查（Humble 实测）

> 来源：2026-07 lab_cobot_ws 把"read_text() 断言源码子串"假测试重写为真内省测试的实测记录。
> 环境：ROS 2 Humble，launch 1.0.x / launch_ros / launch_testing 1.0.14。私有属性名是
> Python name-mangling 产物，跨发行版可能变化——报 AttributeError 时先 `print(dir(obj))`。

## 1. 加载 launch 模块（不启动进程）

```python
import importlib.util

spec = importlib.util.spec_from_file_location("xxx_launch_test", launch_file_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ld = module.generate_launch_description()   # 【实测】纯构造,不 spawn 任何进程
```

- `get_package_share_directory` 在已 source 的 colcon 环境下直接可用；未安装环境用
  `monkeypatch.setattr(module, "get_package_share_directory", lambda pkg: str(SRC / pkg))`【实测两种都可】。

## 2. Action 属性速查表（均为 Humble 实测真名）

| 对象 | 要取什么 | 写法 | 状态 |
|---|---|---|---|
| `Node` | 可执行名/包名 | `node.node_executable` / `node.node_package` | 【实测】公开属性 |
| `Node` | arguments | `node._Node__arguments`（substitution 列表的列表） | 【实测】 |
| `Node` | condition | `node.condition`（IfCondition 实例可 isinstance 判断） | 【实测】 |
| `DeclareLaunchArgument` | 默认值 | `entity.name` + `entity._DeclareLaunchArgument__default_value` | 【实测】 |
| `TimerAction` | 内部动作 | `timer.actions`（公开） | 【实测】 |
| `RegisterEventHandler` | 事件处理器 | `getattr(e, "handler", None) or getattr(e, "_RegisterEventHandler__event_handler", None)` | 【实测其一有效，未逐一区分】 |
| `OnProcessExit` | 退出后动作 | `handler._OnActionEventBase__actions_on_event` | 【实测】⚠️ 不是 `__actions` 也不是 `_OnProcessExit__actions`（两个都试错过） |
| `ExecuteProcess` | 命令行 | `action.process_description.cmd`（substitution 列表的列表） | 【实测】 |
| `ExecuteProcess` | 环境变量 | `action.process_description.additional_env`（launch 里传 `additional_env=` 时 **不在** `.env` 里） | 【实测】保险写法：`list(pd.env or []) + list(pd.additional_env or [])`，元素是 `(key_subst, value_subst)` 对 |

## 3. Substitution 求值

```python
from launch import LaunchContext
from launch.utilities import perform_substitutions

context = LaunchContext()
text = perform_substitutions(context, list_of_substitutions)  # 【实测】列表 -> str
text = single_substitution.perform(context)                   # 单个对象
```

- `cmd` / `arguments` 的**每个元素**本身是 substitution 列表，需逐元素 perform：
  ```python
  def _text_list(values):
      out = []
      for v in values or []:
          if isinstance(v, (list, tuple)):
              out.append(perform_substitutions(LaunchContext(), list(v)))
          elif hasattr(v, "perform"):
              out.append(v.perform(LaunchContext()))
          else:
              out.append(str(v))
      return out
  ```
- 含 `LaunchConfiguration` 的 substitution 在空 context 下 perform 会抛错——测默认值时改从
  `DeclareLaunchArgument` 的 default_value 取【实测同仓范式】。

## 4. 事件链展开范式（拿到全部可内省动作）

launch 文件常见结构：`RegisterEventHandler(OnProcessExit(on_exit=[TimerAction(actions=[Node])]))`
链式嵌套。**递归**展开，单层不够【实测：单层漏掉链尾的控制器 spawner】：

```python
def _all_actions(ld):
    actions = []
    def _walk(entity):
        actions.append(entity)
        if isinstance(entity, TimerAction):
            for child in entity.actions:
                _walk(child)
        handler = getattr(entity, "handler", None) or getattr(
            entity, "_RegisterEventHandler__event_handler", None)
        for child in getattr(handler, "_OnActionEventBase__actions_on_event", None) or []:
            _walk(child)
    for entity in ld.entities:
        _walk(entity)
    return actions
```

## 5. 陷阱速查

| 症状 | 根因 | 修法 |
|---|---|---|
| pytest 报失败的测试函数在当前源文件里**不存在** | ① 文件其实没写成功（工具/编辑器静默失败）② `__pycache__` 陈旧字节码 | 先 `grep -n "def 该函数名" 源文件` 确认是否旧版；再 `rm -rf test/__pycache__`。本次实测根因是 ①——先核对文件再怀疑缓存 |
| 内省属性 AttributeError | name-mangling 真名与猜测不符（如 `__actions`） | `print(dir(obj))` 拿真名，别连猜【实测省时】 |
| 断言 env dict 取不到值 | `additional_env` 与 `env` 是两个属性；且是 (subst,subst) 对列表不是 dict | 见第 2 节保险写法 |
| launch 模块加载即抛包找不到 | 未 source 环境 | source install/setup.bash 或 monkeypatch share dir |

## 6. launch_testing 与 colcon 注册（实测）

- `@pytest.mark.launch_test` 文件可被 pytest 直接跑（launch_testing 自带 pytest 插件）。
- 目录级注册想排除某文件：`ENV "PYTEST_ADDOPTS=-p no:anyio --ignore=${CMAKE_CURRENT_SOURCE_DIR}/test/xxx.py"`【实测】。
- 重 E2E 单独注册并给足 TIMEOUT：注册的 TIMEOUT 必须 ≥ 测试自身内部超时总预算
  （实测反例：TIMEOUT 360 装不下内部 420s 任务超时 + 80s 服务等待，失败原因会失真为 ctest 超时）：
  ```cmake
  ament_add_pytest_test(${PROJECT_NAME}_pytest test/
    ENV "PYTEST_ADDOPTS=-p no:anyio --ignore=${CMAKE_CURRENT_SOURCE_DIR}/test/test_honest_e2e_launch.py"
    TIMEOUT 120)
  ament_add_pytest_test(${PROJECT_NAME}_honest_e2e test/test_honest_e2e_launch.py
    ENV "PYTEST_ADDOPTS=-p no:anyio" TIMEOUT 600)
  ```

## 7. 为什么内省优于文本断言

文本断言（`assert "xxx" in source`）重命名内部变量即红、逻辑写反照绿——是被审计判定的假测试
模式。内省测试断言的是 launch 的**产物结构**（哪个节点、什么参数、挂什么条件），实现怎么写
随意。迁移成本低：一个 `_all_actions` + `_text_list` 助手可复用于全仓 launch 测试。
