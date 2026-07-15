"""Planning scene obstacle helpers for station surfaces and the carried sample."""
# 中文说明:move_group 只发布机器人自身描述,规划场景没有任何环境碰撞体,
# 机械臂 approach/收臂的关节空间弧会扫过工位台面(实测穿模);attach 后的
# 样件也不在规划场景,持物段规划对样件同样不避障。本模块提供:
# - 台面碰撞盒/持物样件附着盒的几何推导(纯函数,可单测)
# - PlanningScene diff 消息构造(纯函数,可单测)
# - ApplyPlanningScene service 客户端(带 DDS 启动竞态 3 次重试)
# vendored pymoveit2 只有 mesh 接口(无 box/attach),故自建消息走标准服务。
import time

# moveit_msgs/shape_msgs 为 ROS 系统包(经 pymoveit2 传递依赖可用),
# 非 pip 重依赖,顶层 import 不违反离线纪律。
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

# ---- 场景几何事实(来源 lab.world 与既有实测标定) ----
TABLE_HEIGHT = 0.75
SAMPLE_HALF_HEIGHT = 0.035
# base_link 离地 0.155m(实测标定,与 test_mission_place_pose.py 同源);
# 两站台面同高,台顶在 base_link 系是常量——不从检测 z 推(检测 z 有误差)。
BASE_LINK_WORLD_Z = 0.155
TABLE_TOP_Z_IN_BASE = TABLE_HEIGHT - BASE_LINK_WORLD_Z
# 方盒边长预算:停靠 yaw 容差 0.25rad 下 0.8x0.6 台面的轴对齐包络为
# 0.8*cos(0.25)+0.6*sin(0.25)=0.924,加检测误差余量取 0.95。上限约束:
# 名义车心-样件距离 0.88,盒前缘 0.88-0.475=0.405 必须大于车头半长
# 0.28+停靠容差 0.12=0.40——0.95 已是极限,禁止加大,否则停靠偏近时
# 起始位形陷入碰撞盒,所有规划直接失败。
SURFACE_BOX_XY = 0.95
# 附着盒底部上收 0.02(与悬空释放余量同源):lift 起点样件底与台面零距,
# 不收则持物笛卡尔路径首个路径点即报碰撞,抓取序列全灭。
ATTACHED_SAMPLE_BOTTOM_TRIM = 0.02
# 持有期样件中心相对 gripper_tcp 的 z 偏移,2026-07-12 DG-2 深抓探针
# 两轮实测中点(-0.013055/-0.017907),与 test_mission_place_pose.py 同源。
HELD_SAMPLE_CENTER_FROM_TCP_Z = -0.015481

STATION_SURFACE_BOX_ID = "station_surface"
CARRIED_SAMPLE_BOX_ID = "carried_sample"
GRIPPER_ATTACH_LINK = "gripper_tcp"
# 与持物样件允许接触的机器人 link(URDF parallel_gripper.xacro 事实)。
GRIPPER_TOUCH_LINKS = (
    "gripper_left_finger",
    "gripper_right_finger",
    "gripper_base",
)

APPLY_SCENE_SERVICE = "/apply_planning_scene"
APPLY_SCENE_ATTEMPTS = 3
APPLY_SCENE_TIMEOUT_SEC = 5.0


def station_surface_box(pos) -> dict:
    """Return center and size of the station surface box in base_link frame."""
    # 只取 xy 定盒中心;z 用常量台顶(两站同高,车在平地 base_link 高度恒定)。
    center_z = TABLE_TOP_Z_IN_BASE - TABLE_HEIGHT / 2.0
    return {
        "center": [float(pos[0]), float(pos[1]), center_z],
        "size": [SURFACE_BOX_XY, SURFACE_BOX_XY, TABLE_HEIGHT],
    }


def carried_sample_box() -> dict:
    """Return center and size of the attached sample box relative to the TCP."""
    size_z = 2.0 * SAMPLE_HALF_HEIGHT - ATTACHED_SAMPLE_BOTTOM_TRIM
    center_z = HELD_SAMPLE_CENTER_FROM_TCP_Z + ATTACHED_SAMPLE_BOTTOM_TRIM / 2.0
    return {
        "center": [0.0, 0.0, center_z],
        "size": [2.0 * SAMPLE_HALF_HEIGHT, 2.0 * SAMPLE_HALF_HEIGHT, size_z],
    }


def _add_box_collision_object(object_id, box, frame_id) -> CollisionObject:
    obj = CollisionObject()
    obj.header.frame_id = str(frame_id)
    obj.id = str(object_id)
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(v) for v in box["size"]]
    pose = Pose()
    pose.position.x = float(box["center"][0])
    pose.position.y = float(box["center"][1])
    pose.position.z = float(box["center"][2])
    pose.orientation.w = 1.0
    obj.primitives = [primitive]
    obj.primitive_poses = [pose]
    return obj


def _remove_collision_object(object_id) -> CollisionObject:
    obj = CollisionObject()
    obj.id = str(object_id)
    obj.operation = CollisionObject.REMOVE
    return obj


def make_world_box_scene(object_id, box, frame_id) -> PlanningScene:
    """Build a diff scene adding one box to the planning world."""
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [
        _add_box_collision_object(object_id, box, frame_id)
    ]
    return scene


def make_attach_scene(object_id) -> PlanningScene:
    """Build a diff scene attaching the carried sample box to the TCP link."""
    attached = AttachedCollisionObject()
    attached.link_name = GRIPPER_ATTACH_LINK
    attached.touch_links = list(GRIPPER_TOUCH_LINKS)
    attached.object = _add_box_collision_object(
        object_id, carried_sample_box(), GRIPPER_ATTACH_LINK
    )
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [attached]
    return scene


def make_detach_scene(object_id) -> PlanningScene:
    """Build a diff scene detaching the sample and removing its world copy."""
    # attached REMOVE 会把物体放回 world,同一 diff 里必须同时 REMOVE
    # world 副本才是干净移除。
    attached = AttachedCollisionObject()
    attached.link_name = GRIPPER_ATTACH_LINK
    attached.object = _remove_collision_object(object_id)
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [attached]
    scene.world.collision_objects = [_remove_collision_object(object_id)]
    return scene


class PlanningSceneClient:
    """Thin ApplyPlanningScene client with startup race retries."""

    def __init__(self, node, service_name=APPLY_SCENE_SERVICE):
        self._node = node
        self._client = node.create_client(ApplyPlanningScene, service_name)

    def apply(
        self,
        scene,
        attempts=APPLY_SCENE_ATTEMPTS,
        timeout_sec=APPLY_SCENE_TIMEOUT_SEC,
    ) -> bool:
        """Apply a PlanningScene diff and return True on confirmed success."""
        # DDS 启动竞态:节点刚起时单次 service call 会假超时,一律多次重试;
        # 响应由外部 executor 线程处理,这里只轮询 future,不再自旋节点。
        for _attempt in range(int(attempts)):
            if not self._client.wait_for_service(timeout_sec=timeout_sec):
                continue
            request = ApplyPlanningScene.Request()
            request.scene = scene
            future = self._client.call_async(request)
            deadline = time.monotonic() + float(timeout_sec)
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not future.done():
                future.cancel()
                continue
            response = future.result()
            if response is not None and response.success:
                return True
        return False
