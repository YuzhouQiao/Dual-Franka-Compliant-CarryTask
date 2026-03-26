#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VisionCoreNode(Node):
    def __init__(self):
        super().__init__('vision_core_node')
        
        # ROS 2 CV Bridge
        self.bridge = CvBridge()
        
        # 订阅 mujoco_ros 自动生成的原生相机画面
        self.sub = self.create_subscription(
            Image,
            '/cameras/overhead_camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        # 重新发布为纯净的全局视觉话题（也可在此处进行图像处理）
        self.pub = self.create_publisher(
            Image,
            '/overhead_camera/image_raw',
            10
        )
        self.get_logger().info('✅ 极简视觉桥接节点已启动：正在桥接并准备进行机器视觉处理...')

    def image_callback(self, msg):
        try:
            # 1. 将 ROS Image 转换为 OpenCV 格式 (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # --- [在这里添加你的 OpenCV 图像处理算法 (如颜色阈值、轮廓提取、计算物块位姿)] ---
            # cv2.circle(cv_image, (320, 240), 5, (0, 255, 0), -1) 
            # ...
            # --------------------------------------------------------------------------
            
            # 2. 将(处理后的)画面重新发布到目标话题
            new_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            new_msg.header = msg.header # 保持相同的时间戳和坐标系
            self.pub.publish(new_msg)
            
        except Exception as e:
            self.get_logger().error(f"视觉处理异常: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionCoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()