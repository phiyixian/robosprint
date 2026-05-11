#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import glob

class DatasetProcessorNode(Node):
    def __init__(self):
        super().__init__('debug_dataset_generator')
        
        # 1. Declare ROS 2 Parameters for dynamic folder paths
        self.declare_parameter('input_folder', 'templates')
        self.declare_parameter('output_folder', 'debug_templates')
        
        # Run the processing task immediately upon node creation
        self.generate_debug_dataset()

    def resize_with_pad(self, img, size=50):
        """Exactly the same padding math as the main script."""
        h, w = img.shape
        scale = size / max(h, w)
        resized = cv2.resize(img, (int(w * scale), int(h * scale)))
        
        top = (size - resized.shape[0]) // 2
        bottom = size - resized.shape[0] - top
        left = (size - resized.shape[1]) // 2
        right = size - resized.shape[1] - left
        
        return cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)

    def generate_debug_dataset(self):
        # 2. Get the values from the ROS 2 parameters
        input_folder = self.get_parameter('input_folder').value
        output_folder = self.get_parameter('output_folder').value

        # 3. Use ROS logging instead of print()
        self.get_logger().info(f"🔍 Inspecting Dataset from '{input_folder}'...")
        
        if not os.path.exists(input_folder):
            self.get_logger().error(f"Could not find the '{input_folder}' folder.")
            return

        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
            
        found_files = glob.glob(f"{input_folder}/*.*")
        if not found_files:
            self.get_logger().error(f"No images found inside '{input_folder}'.")
            return

        for file in found_files:
            filename = os.path.basename(file)
            letter = filename.split('.')[0].upper()
            
            # --- FIX: Read in COLOR, not grayscale ---
            img = cv2.imread(file, cv2.IMREAD_COLOR)
            if img is None:
                continue
                
            # --- FIX: Use the exact same HSV filter as the live camera ---
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            _, _, v_channel = cv2.split(hsv)
            
            blurred = cv2.GaussianBlur(v_channel, (5, 5), 0)
            _, thresh_inv = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            
            contours, _ = cv2.findContours(thresh_inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Ignore tiny noise
                if cv2.contourArea(largest_contour) < 50:
                    continue
                    
                x, y, w, h = cv2.boundingRect(largest_contour)
                letter_crop = thresh_inv[y:y+h, x:x+w]
                
                final_template = self.resize_with_pad(letter_crop, 50)
                
                # Save the processed image
                save_path = os.path.join(output_folder, f"{letter}_processed.png")
                cv2.imwrite(save_path, final_template)
                self.get_logger().info(f"✅ Processed and saved: {save_path}")
            else:
                self.get_logger().warn(f"❌ Failed to find a shape in: {filename}")

        self.get_logger().info(f"🎉 Done! Open the '{output_folder}' folder to see the solid letters.")

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node (which runs the dataset generation in its __init__)
    node = DatasetProcessorNode()
    
    # Since this is a batch processing script (runs once and finishes),
    # we don't need rclpy.spin(node). We just destroy and shutdown when done.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
