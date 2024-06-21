import rclpy

from tiny_gestures.DetectionRosNode import DetectionRosNode

#############################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = DetectionRosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
