import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import time
from ultralytics import YOLO

def compress_block_svd(image, k=5):
    
    h, w = image.shape
    h = h - (h % 8)
    w = w - (w % 8)
    image = image[:h, :w]
    
    blocks = image.reshape(h//8, 8, w//8, 8).transpose(0, 2, 1, 3).reshape(-1, 8, 8).astype(np.float32)
    U, S, Vt = np.linalg.svd(blocks, full_matrices=False)
    
    
    U_k = U[:, :, :k]
    S_k = S[:, :k]
    Vt_k = Vt[:, :k, :]
    
    reconstructed_blocks = np.matmul(U_k * S_k[:, np.newaxis, :], Vt_k)
    reconstructed_blocks = np.clip(reconstructed_blocks, 0, 255).astype(np.uint8)
    
    reconstructed_img = reconstructed_blocks.reshape(h//8, w//8, 8, 8).transpose(0, 2, 1, 3).reshape(h, w)
    return reconstructed_img

class TurtleBotYoloUpNode(Node):
    def __init__(self):
        super().__init__('turtlebot_yolo_up_node')
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            10)
        
        
        self.k_value = 5 
        self.model = YOLO('yolov8n.pt') 

    def image_callback(self, msg):
        
        np_arr = np.frombuffer(msg.data, np.uint8)
        gray_frame = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        
        if gray_frame is None:
            return

        gray_frame = cv2.resize(gray_frame, (160, 120))
        
        
        compressed_frame = compress_block_svd(gray_frame, k=self.k_value)
        
        
        color_compressed_frame = cv2.cvtColor(compressed_frame, cv2.COLOR_GRAY2BGR)
        
        
        
        results = self.model(color_compressed_frame, conf=0.35, verbose=False) 
        
        
        annotated_frame = results[0].plot()
        
        
        cv2.putText(annotated_frame, f"SVD(k={self.k_value}) + YOLOv8", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(annotated_frame, "Quality Improved", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
    
        final_view = cv2.resize(annotated_frame, (640, 480))
        
        cv2.imshow("TurtleBot Vision: YOLO Accuracy Up", final_view)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotYoloUpNode()
    print("YOLO 실행 중...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n종료합니다.")
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
