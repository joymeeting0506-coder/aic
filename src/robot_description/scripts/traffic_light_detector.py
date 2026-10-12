#!/usr/bin/env python3
"""红绿灯颜色识别节点。

订阅相机图像，用 HSV 颜色分割识别红绿灯当前状态（红 / 绿 / 黄），并把结果
发布成话题 + 打印到终端 + 输出带标注框的可视化图像，满足比赛「终端与图像两处
同步呈现」的要求。

识别思路（针对本项目的红绿灯模型）：
  红绿灯是「深色横板 + 三块并排灯板」，同一时刻只有一块亮（饱和的红/绿/黄），
  其余两块近乎黑。为避免场景里其它红/绿/黄物体干扰，先找深色横板定位红绿灯
  整体，再在板内判断哪块灯亮；找不到横板时退回全图颜色块检测。

输出话题：
  /traffic_light/state       std_msgs/String   "RED" | "GREEN" | "YELLOW" | "UNKNOWN"
  /traffic_light/confidence  std_msgs/Float32  0~1
  /traffic_light/detections  std_msgs/String   所有识别到的红绿灯（JSON 列表）
  /traffic_light/annotated   sensor_msgs/Image 画了框 + 标注 + 置信度的图像
"""
import json
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageDraw, ImageFont
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32


# ---- HSV 颜色阈值（OpenCV：H∈[0,179]，S/V∈[0,255]）----------------------
# 针对仿真里红绿灯灯板的饱和色调的实测值，如现场色偏可在 launch 里调参覆盖。
RED_HUE_A = (0, 10)      # 红色在 HSV 色相环上绕 0 度
RED_HUE_B = (170, 180)
GREEN_HUE = (40, 85)     # 绿 ≈ 60
YELLOW_HUE = (20, 40)    # 黄 ≈ 30

# 亮灯判据：亮度(V)是关键分界。熄灭的灯板纹理仍带饱和色（红/黄/绿 S 可达
# 130~185）但很暗（V max ≈ 141）；点亮的灯板 V 高达 177。统一用 S/V 门槛 +
# 各自色相区间，把熄灭灯板挡在门外。
LIT_S_MIN = 80
LIT_V_MIN = 130

# 中文字体候选（cv2.putText 的 Hershey 字体不支持中文，标签改用 PIL 画）。
_CJK_FONT_CANDIDATES = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
    '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
]


def _load_cjk_font(size):
    for path in _CJK_FONT_CANDIDATES:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
    return ImageFont.load_default()


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.bridge = CvBridge()

        # ---- 参数 ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('min_board_area', 250)   # 深色横板最小像素面积
        self.declare_parameter('min_lamp_area', 60)     # 亮灯板最小像素面积
        self.declare_parameter('dark_v_max', 25)        # 判定「深色板」的 V 上限（灯板近纯黑，地板约 V>28）
        self.declare_parameter('lit_s_min', LIT_S_MIN)  # 亮灯板饱和度下限
        self.declare_parameter('lit_v_min', LIT_V_MIN)  # 亮灯板亮度下限（关键分界）
        self.declare_parameter('debounce_frames', 2)    # 连续 N 帧一致才切换状态
        self.declare_parameter('label_font_size', 18)   # 标注文字字号
        self.image_topic = self.get_parameter('image_topic').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.min_board_area = self.get_parameter('min_board_area').value
        self.min_lamp_area = self.get_parameter('min_lamp_area').value
        self.dark_v_max = self.get_parameter('dark_v_max').value
        self.lit_s_min = self.get_parameter('lit_s_min').value
        self.lit_v_min = self.get_parameter('lit_v_min').value
        self.debounce_frames = self.get_parameter('debounce_frames').value
        self.label_font_size = self.get_parameter('label_font_size').value

        # 中文字体（用于标注标签）
        self._font = _load_cjk_font(self.label_font_size)

        # ---- QoS：相机走 BEST_EFFORT ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.create_subscription(Image, self.image_topic, self.image_cb, sensor_qos)

        self.state_pub = self.create_publisher(String, '/traffic_light/state', 10)
        self.conf_pub = self.create_publisher(Float32, '/traffic_light/confidence', 10)
        self.det_pub = self.create_publisher(String, '/traffic_light/detections', 10)
        self.ann_pub = self.create_publisher(Image, '/traffic_light/annotated', sensor_qos)

        # 去抖：连续 debounce_frames 帧识别到同一状态才发布/打印
        self._candidate = None
        self._candidate_count = 0
        self._published_state = 'UNKNOWN'

        self.get_logger().info(f'订阅 {self.image_topic}，开始红绿灯颜色识别')

    # ------------------------------------------------------------------
    def image_cb(self, msg):
        img = self._to_bgr(msg)
        if img is None:
            return
        detections = self.detect(img)

        state, conf = self._primary(detections)
        self._publish(state, conf, detections)

        if self.publish_annotated:
            ann = self.annotate(img, detections)
            self.ann_pub.publish(self.bridge.cv2_to_imgmsg(ann, encoding='bgr8'))

    # ------------------------------------------------------------------
    def _to_bgr(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')
            return None
        enc = msg.encoding.lower()
        if enc == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == 'bgr8':
            return img
        if enc in ('mono8', '8uc1'):
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        # 其它编码尽量按 BGR 理解
        return img

    def _masks(self, hsv):
        """返回 {颜色: 二值 mask}，只保留「亮 + 饱和」的灯板像素，已做形态学去噪。"""
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        h = hsv[:, :, 0]
        bright = (s >= self.lit_s_min) & (v >= self.lit_v_min)

        red = (((h <= RED_HUE_A[1]) & (h >= RED_HUE_A[0])) |
               ((h >= RED_HUE_B[0]) & (h <= RED_HUE_B[1]))) & bright
        green = ((h >= GREEN_HUE[0]) & (h <= GREEN_HUE[1])) & bright
        yellow = ((h >= YELLOW_HUE[0]) & (h <= YELLOW_HUE[1])) & bright

        masks = {}
        for color, m in (('red', red), ('green', green), ('yellow', yellow)):
            m = (m.astype(np.uint8) * 255)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            masks[color] = m
        return masks

    def _dark_board_boxes(self, hsv):
        """用低 V 找深色横板，返回候选红绿灯整体包围盒列表 [x, y, w, h]。"""
        dark = (hsv[:, :, 2] < self.dark_v_max).astype(np.uint8) * 255
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        cnts, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            if w * h < self.min_board_area:
                continue
            # 红绿灯（横板+立柱）整体宽略大于高，过滤细长竖杆/噪点
            if w < 0.8 * h:
                continue
            boxes.append([x, y, w, h])
        boxes.sort(key=lambda b: -b[2] * b[3])
        return boxes

    def _classify_roi(self, masks, box):
        """在 box 内统计三种颜色像素，返回 (color, confidence) 或 (None, 0)。"""
        x, y, w, h = box
        roi = (slice(max(y, 0), y + h), slice(max(x, 0), x + w))
        counts = {c: int(np.count_nonzero(m[roi])) for c, m in masks.items()}
        total = sum(counts.values())
        if total < self.min_lamp_area:
            return None, 0.0
        color = max(counts, key=counts.get)
        conf = counts[color] / total
        return color, conf

    def detect(self, img):
        """识别一帧里的所有红绿灯。返回 [{'label','confidence','bbox'}...]。"""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        masks = self._masks(hsv)
        detections = []

        # 1) 板锚定：深色横板内含亮灯板才算红绿灯
        for box in self._dark_board_boxes(hsv):
            # 略微外扩，让灯板完整落进 ROI
            pad = int(0.1 * max(box[2], box[3]))
            ex = [box[0] - pad, box[1] - pad, box[2] + 2 * pad, box[3] + 2 * pad]
            color, conf = self._classify_roi(masks, ex)
            if color is None:
                continue
            detections.append({'label': color.upper(), 'confidence': round(conf, 3),
                               'bbox': box})

        # 2) 兜底：没定位到板时，用全图最大颜色块
        if not detections:
            best = None
            for color, m in masks.items():
                cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
                if not cnts:
                    continue
                c = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area < self.min_lamp_area:
                    continue
                if best is None or area > best[0]:
                    x, y, w, h = cv2.boundingRect(c)
                    conf = round(self._fallback_conf(masks, color, area), 3)
                    best = (area, color.upper(), conf, [x, y, w, h])
            if best:
                _, label, conf, box = best
                detections.append({'label': label, 'confidence': conf, 'bbox': box})

        detections.sort(key=lambda d: -d['bbox'][2] * d['bbox'][3])
        return detections

    @staticmethod
    def _fallback_conf(masks, color, area):
        others = sum(int(np.count_nonzero(m)) for c, m in masks.items() if c != color)
        return area / (area + others) if area + others else 0.0

    def _primary(self, detections):
        if not detections:
            return 'UNKNOWN', 0.0
        d = detections[0]
        return d['label'], d['confidence']

    # ------------------------------------------------------------------
    def _publish(self, state, conf, detections):
        # 去抖：连续 N 帧一致才真正对外发布/打印
        if state == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = state
            self._candidate_count = 1
        if self._candidate_count < self.debounce_frames:
            return

        self.state_pub.publish(String(data=state))
        self.conf_pub.publish(Float32(data=float(conf)))

        payload = json.dumps(detections, ensure_ascii=False)
        self.det_pub.publish(String(data=payload))

        if state != self._published_state:
            self._published_state = state
            zh = {'RED': '红灯', 'GREEN': '绿灯', 'YELLOW': '黄灯', 'UNKNOWN': '未识别'}
            self.get_logger().info(
                f'红绿灯：{zh[state]}（{state}），置信度 {conf:.2f}，'
                f'共识别 {len(detections)} 个目标')
            # 终端按识别顺序打印，与图像一一对应
            print(f'[红绿灯] 类别=红绿灯 内容={zh[state]} 置信度={conf:.2f}')

    def annotate(self, img, detections):
        out = img.copy()
        color_map = {'RED': (0, 0, 255), 'GREEN': (0, 200, 0),
                     'YELLOW': (0, 220, 255), 'UNKNOWN': (128, 128, 128)}
        zh = {'RED': '红灯', 'GREEN': '绿灯', 'YELLOW': '黄灯', 'UNKNOWN': '未知'}

        # 1. 目标框（OpenCV，无文字）
        for d in detections:
            x, y, w, h = d['bbox']
            bgr = color_map.get(d['label'], (128, 128, 128))
            cv2.rectangle(out, (x, y), (x + w, y + h), bgr, 2)

        # 2. 中文标签（PIL：cv2.putText 的 Hershey 字体不支持中文会画成 ??）
        rgb = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)
        draw = ImageDraw.Draw(pil)
        for d in detections:
            x, y, w, h = d['bbox']
            bgr = color_map.get(d['label'], (128, 128, 128))
            rgb_color = (bgr[2], bgr[1], bgr[0])
            text = f"{zh[d['label']]} {d['confidence']:.2f}"
            l, t, r, b = draw.textbbox((0, 0), text, font=self._font)
            tw, th = r - l, b - t
            draw.rectangle([x, y - th - 8, x + tw + 8, y], fill=rgb_color)
            draw.text((x + 4, y - th - 6), text, fill=(255, 255, 255),
                      font=self._font)
        return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
