"""场地坐标：Home 靠场内边线中点为原点；车头朝场内时 x 向前、y 向右。

单位均为 mm。标记中心来自规则 PDF 的示意图，必须现场测量后再用于行驶。
"""

FIELD_WIDTH_MM = 3660
FIELD_DEPTH_MM = 2440
HOME_INNER_EDGE_MM = (0, 0)
HOME_DEPTH_ESTIMATE_MM = 320  # 根据图面比例估算，规则未单独标注

# 圆环图案中心；按图中从左到右、从上到下命名，非机械抓取位。
ESTIMATED_MARKERS_MM = {
    "远端左": (1980, -270),
    "远端中": (1980, 0),
    "远端右": (1980, 270),
    "场内左上": (1090, -640),
    "场内中上": (1090, 220),
    "场内左下": (810, -430),
    "场内中下": (810, 335),
    "场内右下": (880, 845),
}


def relative_move_m(current_mm, target_mm):
    """两个场地坐标之差，供 set_distance(x=..., y=...) 使用；不跟踪实际位置。"""
    return {
        "x": (target_mm[0] - current_mm[0]) / 1000,
        "y": (target_mm[1] - current_mm[1]) / 1000,
    }
