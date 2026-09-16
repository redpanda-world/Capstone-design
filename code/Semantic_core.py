import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json
import os
import math
import yaml
from tf2_ros import TransformListener, Buffer

class SemanticFodSystem(Node):
    def __init__(self):
        super().__init__('semantic_fod_system')
         
        self.model = YOLO('best.pt')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_markers', 10)
        
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10
        )
        
        #  FOD: 거리, Threshold 
        self.target_class = "FOD"
        self.stop_threshold = 0.05
        self.camera_fov = 60.0  
        self.est_distance = 0.4  
        
        # 접근 제어용 변수들
        self.state = 'SEARCHING' # 상태: SEARCHING, TRACKING, STOPPED
        self.tracking_twist = Twist()
        self.track_speed = 0.08   # 접근할 때의 직진 속도 
        self.turn_gain = 0.003    # 각속도 민감도 
        
        # 모터 제어 명령
        self.control_timer = self.create_timer(0.01, self.active_control_loop)
        
        self.json_file = 'semantic_map.json'
        self.map_pgm_file = os.path.expanduser('~/map.pgm')
        self.map_yaml_file = os.path.expanduser('~/map.yaml')
        self.detected_items = []
        
        if os.path.exists(self.json_file):
            with open(self.json_file, 'r') as f:
                self.detected_items = json.load(f)
                
        self.generate_visual_map()
        self.get_logger().info(" FOD System Ready [자율 추적 및 접근 모드]")

    def active_control_loop(self):
        #정지 명령 블럭
        if self.state == 'STOPPED':
            # 영구 정지 
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
        elif self.state == 'TRACKING':
            # FOD로 접근
            self.cmd_vel_pub.publish(self.tracking_twist)

    def get_actual_object_pose(self, bbox_center_x, img_width):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', now, timeout=rclpy.duration.Duration(seconds=0.2)
            )
            x_rob = trans.transform.translation.x
            y_rob = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            fov_rad = math.radians(self.camera_fov)
            offset_ratio = (bbox_center_x - (img_width / 2)) / (img_width / 2) 
            angle_offset = -offset_ratio * (fov_rad / 2)
            
            final_angle = yaw + angle_offset
            obj_x = x_rob + (self.est_distance * math.cos(final_angle))
            obj_y = y_rob + (self.est_distance * math.sin(final_angle))
            return obj_x, obj_y
        except Exception as e:
            return None, None

    def image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        results = self.model(frame, verbose=False)
        annotated_frame = results[0].plot()

        if self.state != 'STOPPED':
            best_box = None
            max_area = 0
            
            # 화면에 있는 박스들 중 가장 큰 FOD 찾기
            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    label = self.model.names[cls_id].upper() # 대문자로 변환

                    # NOTFOD는 무시하고 FOD일 경우에만 진행하도록 함.
                    if label == self.target_class:
                        x1, y1, x2, y2 = box.xyxy[0]
                        area_ratio = ((x2 - x1) * (y2 - y1)) / (w * h)
                        if area_ratio > max_area:
                            max_area = area_ratio
                            best_box = (x1, y1, x2, y2)

            if best_box is not None:
                x1, y1, x2, y2 = best_box
                bbox_center_x = (x1 + x2) / 2.0
                
                # 목표물이 충분히 가까워졌을 때 정지함
                if max_area >= self.stop_threshold:
                    self.state = 'STOPPED'
                    obj_x, obj_y = self.get_actual_object_pose(bbox_center_x, w)
                    if obj_x is not None:
                        self.save_and_publish("FOD", obj_x, obj_y)
                
                # 아직 멀리 있을 때
                else:
                    self.state = 'TRACKING'
                    
                    # 화면 중심과 FOD 중심의 차이를 계산해서 각도 조절
                    center_offset = (w / 2.0) - bbox_center_x 
                    
                    twist = Twist()
                    twist.linear.x = float(self.track_speed) # 앞으로 감
                    twist.angular.z = float(center_offset * self.turn_gain) # 좌우로 각도 조절
                    self.tracking_twist = twist
            else:
                # FOD를 놓치면 다시 Nav가 로봇을 조종함
                self.state = 'SEARCHING'

        cv2.imshow("FOD Monitor Feed", annotated_frame)
        cv2.waitKey(1)

    def save_and_publish(self, label, x, y):
        print(f"\n [정지] {label} 포착 완료")
        print(f" 좌표 Map X: {x:.3f}, Y: {y:.3f}\n")

        is_duplicate = False
        for old_item in self.detected_items:
            dist = math.sqrt((old_item['x'] - x)**2 + (old_item['y'] - y)**2)
            if dist < 0.4: 
                is_duplicate = True
                break
        
        if not is_duplicate:
            item = {"class": label, "x": x, "y": y}
            self.detected_items.append(item)
            with open(self.json_file, 'w') as f:
                json.dump(self.detected_items, f, indent=4)
            self.generate_visual_map()

        self.publish_all_markers()

    def generate_visual_map(self):
        if not os.path.exists(self.map_pgm_file) or not os.path.exists(self.map_yaml_file): return
        map_img = cv2.imread(self.map_pgm_file)
        if map_img is None: return

        try:
            with open(self.map_yaml_file, 'r') as f:
                map_data = yaml.safe_load(f)
                res = map_data['resolution']
                orig_x = map_data['origin'][0]
                orig_y = map_data['origin'][1]
        except Exception: return

        for item in self.detected_items:
            px = int((item['x'] - orig_x) / res)
            py = map_img.shape[0] - int((item['y'] - orig_y) / res)
            if 0 <= px < map_img.shape[1] and 0 <= py < map_img.shape[0]:
                cv2.circle(map_img, (px, py), 4, (0, 0, 255), -1)
                text = f"FOD ({item['x']:.2f}, {item['y']:.2f})"
                cv2.putText(map_img, text, (px + 5, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        save_path = os.path.join(os.getcwd(), 'fod_result_map.png')
        cv2.imwrite(save_path, map_img)

    def publish_all_markers(self):
        marker_array = MarkerArray()
        for i, item in enumerate(self.detected_items):
            dot = Marker()
            dot.header.frame_id = "map"; dot.header.stamp = self.get_clock().now().to_msg()
            dot.ns = "fod_dots"; dot.id = i * 2; dot.type = Marker.SPHERE
            dot.pose.position.x = item['x']; dot.pose.position.y = item['y']; dot.pose.position.z = 0.05  
            dot.scale.x = 0.1; dot.scale.y = 0.1; dot.scale.z = 0.1; dot.color.a = 1.0; dot.color.r = 1.0
            marker_array.markers.append(dot)

            text = Marker()
            text.header.frame_id = "map"; text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "fod_labels"; text.id = i * 2 + 1; text.type = Marker.TEXT_VIEW_FACING
            text.pose.position.x = item['x']; text.pose.position.y = item['y']; text.pose.position.z = 0.2  
            text.scale.z = 0.15; text.color.a = 1.0; text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0
            text.text = f"FOD ({item['x']:.2f}, {item['y']:.2f})"
            marker_array.markers.append(text)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticFodSystem()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
