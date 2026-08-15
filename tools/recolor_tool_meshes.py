#!/usr/bin/env python3
"""给工装工具区 5 件工具的 DAE mesh 上色,让它像真实生活中的工具.

工具 mesh 的 diffuse 全部是白色(1 1 1 1),按 effect id 精准改色:
- 扳手: 银灰金属
- 钳子: 红手柄 + 银金属
- 电钻: 黄/蓝机身 + 银金属 + 黑握把
- 螺丝刀: 红手柄 + 银杆
- 卡尺: 黑主体 + 亮银刻度
"""
import xml.etree.ElementTree as ET
from pathlib import Path

NS = "http://www.collada.org/2005/11/COLLADASchema"
ET.register_namespace("", NS)

MODELS = (
    Path(__file__).resolve().parents[1]
    / "src" / "lab_cobot_gazebo" / "models"
)

# effect id -> (r, g, b)   (id 省略末尾 "-effect", 由脚本拼接)
RECOLOR = {
    "fixture_box_plain": {
        "meshes/adjustable_wrench.dae": {
            "Aluminum_-_Bead_Blasted": (0.72, 0.75, 0.80),
        },
    },
    "tooling_hand_tools": {
        "meshes/pliers_closed.dae": {
            "Aluminum_-_Polished": (0.85, 0.86, 0.90),
            "Aluminum_-_Satin": (0.70, 0.73, 0.78),
            "ABS__White_": (0.82, 0.16, 0.12),  # 红手柄
        },
    },
    "pcb_test_fixture": {
        "meshes/drill.dae": {
            "Paint_-_Metallic__Yellow_": (0.92, 0.72, 0.08),
            "Paint_-_Metallic__Silver_": (0.80, 0.80, 0.84),
            "Paint_-_Enamel_Glossy__Blue_": (0.16, 0.30, 0.60),
            "Paint_-_Metallic__Blue_": (0.10, 0.20, 0.45),
            "Paint_-_Metallic__Green_": (0.20, 0.52, 0.26),
            "Paint_-_Enamel_Glossy__Yellow_": (0.92, 0.80, 0.20),
            "Paint_-_Enamel_Glossy__Black_": (0.08, 0.08, 0.10),
            "ABS__White_": (0.92, 0.92, 0.94),
            "Acetal_Resin__White_": (0.90, 0.90, 0.92),
            "Paek__Beige_": (0.82, 0.76, 0.64),
        },
    },
    "safety_probe_kit": {
        "meshes/screwdriver.dae": {
            "Paint_-_Enamel_Glossy__Dark_Grey_": (0.80, 0.16, 0.12),  # 红手柄
            "Aluminum_-_Satin": (0.78, 0.80, 0.84),  # 银杆
        },
    },
    "igbt_module_plain": {
        "meshes/digital_caliper.dae": {
            "Paint_-_Enamel_Glossy__Black_": (0.08, 0.08, 0.10),
            "Aluminum_-_Polished": (0.85, 0.86, 0.90),
            "Aluminum_-_Bead_Blasted": (0.72, 0.75, 0.80),
        },
    },
}


def _set_diffuse(effect, rgb):
    color = effect.find(f".//{{{NS}}}diffuse/{{{NS}}}color")
    if color is None:
        return False
    color.text = " ".join(f"{v:.4f}" for v in rgb) + " 1"
    return True


def main() -> None:
    changed_total = 0
    for model, files in RECOLOR.items():
        for rel, mapping in files.items():
            path = MODELS / model / rel
            root = ET.parse(path).getroot()
            by_id = {}
            for eff in root.iter(f"{{{NS}}}effect"):
                by_id[eff.get("id")] = eff
            for material, rgb in mapping.items():
                eff_id = f"{material}-effect"
                if eff_id not in by_id:
                    raise KeyError(f"{model}/{rel}: effect id {eff_id!r} 未找到")
                if not _set_diffuse(by_id[eff_id], rgb):
                    raise RuntimeError(
                        f"{model}/{rel}: {eff_id} 无 diffuse color"
                    )
                changed_total += 1
            # 写回,保持默认命名空间
            path.write_text(
                ET.tostring(root, encoding="unicode"),
                encoding="utf-8",
            )
            print(f"{model}/{rel}: 改写 {len(mapping)} 个材质")
    print(f"共改写 {changed_total} 个材质")


if __name__ == "__main__":
    main()
