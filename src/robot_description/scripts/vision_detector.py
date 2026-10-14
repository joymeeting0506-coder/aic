#!/usr/bin/env python3
"""人偶身份识别 + 车牌 OCR 视觉识别节点。

按需触发：巡检节点停在 view_* / read_plate_* 航点时，通过 /vision/trigger 请求一次
识别；本节点对缓存的最新相机帧跑 vision_training 的推理（人偶身份 SVM + 车牌逐字符
OCR），发布标注图像 + JSON 结果，并在终端按顺序打印「类别 / 内容 / 置信度」，同时
累计整个街区的社区/非社区人数统计。

话题：
  /vision/trigger    std_msgs/String   {"id":N,"mode":"person"|"plate"|"both"|"reset"|"stats"}
  /vision/result     std_msgs/String   结果 JSON：{"id","mode","people","plates","stats"}
  /vision/annotated  sensor_msgs/Image 带框+标注的 bgr8 图像

触发模式：
  person —— 人偶身份识别；plate —— 车牌 OCR；both —— 两者都跑；
  reset —— 清空街区统计；stats —— 只回传统计，不做识别。
"""
import json
import os
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _default_vision_dir():
    """vision_training 是本仓库里与 robot_description 平级的非 ROS 包。

    symlink-install 下 __file__ 指向源码 src/robot_description/scripts/...，
    因此退两级即 src/，再拼 vision_training。现场也可用 vision_dir 参数覆盖。
    """
    here = os.path.realpath(__file__)
    return os.path.abspath(os.path.join(os.path.dirname(here), '..', '..', 'vision_training'))


class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        self.bridge = CvBridge()

        # ---- 参数 ----
        self.declare_parameter('vision_dir', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('person_max_width', 480)   # 人偶滑窗前降采样宽，越小越快
        self.declare_parameter('person_threshold', 0.0)   # 人偶 SVM 决策阈值
        self.declare_parameter('publish_annotated', True)

        vision_dir = self.get_parameter('vision_dir').value or _default_vision_dir()
        self.image_topic = self.get_parameter('image_topic').value
        self.person_max_width = int(self.get_parameter('person_max_width').value)
        self.person_threshold = float(self.get_parameter('person_threshold').value)
        self.publish_annotated = self.get_parameter('publish_annotated').value

        # ---- 载入 vision_training 模型 ----
        if vision_dir not in sys.path:
            sys.path.insert(0, vision_dir)
        try:
            import vision_app
        except ImportError as e:
            self.get_logger().error(f'无法导入 vision_app（vision_dir={vision_dir}）：{e}')
            raise
        self.vision_app = vision_app
        (self.detector, self.identity_model, self.plate_model, self.metadata,
         self.province_svm, self.char_svm,
         self.province_labels, self.char_labels) = vision_app.load_models()
        self.get_logger().info(
            f'模型已加载：人偶 {len(self.metadata.get("person_labels", []))} 类、'
            f'车牌模板 {len(self.metadata.get("plate_labels", []))} 个、'
            f'OCR={"逐字符" if self.province_svm is not None else "整牌回退"}')

        # ---- 订阅相机，缓存最新帧 ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.create_subscription(Image, self.image_topic, self._image_cb, sensor_qos)
        self._frame = None

        # ---- 触发 ----
        self.create_subscription(String, '/vision/trigger', self._trigger_cb, 10)

        # ---- 输出 ----
        self.result_pub = self.create_publisher(String, '/vision/result', 10)
        self.ann_pub = self.create_publisher(Image, '/vision/annotated', sensor_qos)

        # ---- 街区统计 ----
        self._seen = {}   # name -> identity
        self.get_logger().info(f'订阅 {self.image_topic}，等待 /vision/trigger 触发')

    # ------------------------------------------------------------------
    def _image_cb(self, msg):
        self._frame = self._to_bgr(msg)

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
        return img

    # ------------------------------------------------------------------
    def _trigger_cb(self, msg):
        try:
            req = json.loads(msg.data)
        except Exception:
            self.get_logger().warn(f'触发消息不是合法 JSON：{msg.data!r}')
            return
        req_id = req.get('id')
        mode = req.get('mode', 'both')

        if mode == 'reset':
            self._seen = {}
            self.get_logger().info('已清空街区统计')
            self._publish_result(req_id, mode, [], [], self._stats_payload())
            return
        if mode == 'stats':
            self._publish_result(req_id, mode, [], [], self._stats_payload())
            return
        if self._frame is None:
            self.get_logger().warn(f'尚未收到相机帧（{self.image_topic}），跳过识别')
            self._publish_result(req_id, mode, [], [], self._stats_payload())
            return

        frame = self._frame
        people, plates = [], []
        if mode in ('person', 'both'):
            people = self.vision_app.detect_people(
                frame, self.detector, self.identity_model, self.metadata,
                threshold=self.person_threshold, max_width=self.person_max_width)
            self._accumulate(people)
            self._report_people(people)
        if mode in ('plate', 'both'):
            plates = self.vision_app.detect_plates(
                frame, self.plate_model, self.metadata.get('plate_labels', []),
                self.province_svm, self.char_svm,
                self.province_labels, self.char_labels)
            self._report_plates(plates)

        if self.publish_annotated:
            annotated = self.vision_app.draw_results(frame.copy(), people, plates)
            self.ann_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

        self._publish_result(req_id, mode, people, plates, self._stats_payload())

    def _accumulate(self, people):
        for *_, label, _conf in people:
            if isinstance(label, dict):
                self._seen[label.get('name', '')] = label.get('identity', '')

    def _stats_payload(self):
        community = sum(1 for i in self._seen.values() if i == 'community')
        noncommunity = sum(1 for i in self._seen.values() if i == 'noncommunity')
        return {
            'community': community,
            'noncommunity': noncommunity,
            'total_seen': len(self._seen),
            'persons': sorted(self._seen.keys()),
        }

    def _publish_result(self, req_id, mode, people, plates, stats):
        def _p(p):
            label = p[4] if isinstance(p[4], dict) else {}
            return {
                'name': label.get('name', ''),
                'identity': label.get('identity', ''),
                'confidence': round(float(p[5]), 3),
            }

        def _pl(p):
            return {'text': p[4], 'confidence': round(float(p[5]), 3)}

        payload = {
            'id': req_id,
            'mode': mode,
            'people': [_p(p) for p in people],
            'plates': [_pl(p) for p in plates],
            'stats': stats,
        }
        self.result_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _report_people(self, people):
        if not people:
            self.get_logger().info('[人偶] 未识别到人偶')
            return
        self.get_logger().info(f'[人偶] 本帧识别到 {len(people)} 个人偶')
        for *_, label, conf in people:
            if not isinstance(label, dict):
                continue
            ident = label.get('identity', '')
            zh = {'community': '社区人员', 'noncommunity': '非社区人员'}.get(ident, ident)
            line = f'[人偶] 类别=人偶 内容={zh}({label.get("name", "")}) 置信度={conf:.2f}'
            print(line)

    def _report_plates(self, plates):
        if not plates:
            self.get_logger().info('[车牌] 未识别到车牌')
            return
        for *_, text, score in plates:
            line = f'[车牌] 类别=车牌 内容={text} 置信度={score:.2f}'
            print(line)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'视觉识别节点异常：{e}')
        else:
            print(f'初始化失败：{e}', file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
