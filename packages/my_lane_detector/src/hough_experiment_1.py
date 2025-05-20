#!/usr/bin/env python3
import sys
import os
import cv2
import numpy as np

# File with spaces
image_filename = 'Raw Camera Image_screenshot_07.05.2025.png'
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, image_filename)

# Load image
img = cv2.imread(image_path, cv2.IMREAD_COLOR)
if img is None:
    print(f"Error: Could not load image from {image_path}")
    sys.exit(1)

print(f"Successfully loaded image: {image_path}")

# Crop bottom half
height, width, _ = img.shape
crop_img = img[int(height / 2):, :]

# Convert to grayscale
gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

# Apply Canny
edges = cv2.Canny(gray, 100, 200)

# Function to apply Hough and return image
def apply_hough(threshold):
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=threshold, minLineLength=40, maxLineGap=10)
    output = crop_img.copy()
    if lines is not None:
        for l in lines:
            x1, y1, x2, y2 = l[0]
            cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return output

# Apply Hough with different thresholds
hough_30 = apply_hough(30)
hough_50 = apply_hough(50)
hough_70 = apply_hough(70)

# Display images
try:
    while True:
        cv2.imshow("Original Cropped", crop_img)
        cv2.imshow("Canny Edges", edges)
        cv2.imshow("Hough Threshold 30", hough_30)
        cv2.imshow("Hough Threshold 50", hough_50)
        cv2.imshow("Hough Threshold 70", hough_70)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
    print("Windows closed.")

