import cv2
import numpy as np
import os
import glob

def resize_with_pad(img, size=50):
    """Exactly the same padding math as the main script."""
    h, w = img.shape
    scale = size / max(h, w)
    resized = cv2.resize(img, (int(w * scale), int(h * scale)))
    
    top = (size - resized.shape[0]) // 2
    bottom = size - resized.shape[0] - top
    left = (size - resized.shape[1]) // 2
    right = size - resized.shape[1] - left
    
    return cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)

def generate_debug_dataset(input_folder="templates", output_folder="debug_templates"):
    print(f"🔍 Inspecting Dataset from '{input_folder}'...")
    
    if not os.path.exists(input_folder):
        print(f"Error: Could not find the '{input_folder}' folder.")
        return

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        
    found_files = glob.glob(f"{input_folder}/*.*")
    if not found_files:
        print(f"Error: No images found inside '{input_folder}'.")
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
            
            final_template = resize_with_pad(letter_crop, 50)
            
            # Save the processed image
            save_path = os.path.join(output_folder, f"{letter}_processed.png")
            cv2.imwrite(save_path, final_template)
            print(f"✅ Processed and saved: {save_path}")
        else:
            print(f"❌ Failed to find a shape in: {filename}")

    print(f"\n🎉 Done! Open the '{output_folder}' folder to see the solid letters.")

if __name__ == '__main__':
    generate_debug_dataset()
