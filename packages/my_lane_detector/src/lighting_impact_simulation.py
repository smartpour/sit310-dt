#!/usr/bin/env python3
import sys
import os
import cv2
import numpy as np

# Image file with spaces
image_filename = 'Raw Camera Image_screenshot_07.05.2025.png'
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, image_filename)

# Load original image
img = cv2.imread(image_path, cv2.IMREAD_COLOR)
if img is None:
    print(f"Error: Could not load image from {image_path}")
    sys.exit(1)

print("Successfully loaded image.")

# Crop bottom half
height, width, _ = img.shape
crop_img = img[int(height / 2):, :]

# Simulate darker and brighter images
darker = (crop_img * 0.5).astype(np.uint8)
brighter = cv2.convertScaleAbs(crop_img, alpha=1.5, beta=30)

# Define fixed HSV range for yellow detection
lower_yellow_hsv = np.array([15, 80, 80])
upper_yellow_hsv = np.array([45, 255, 255])

# Function to apply yellow mask and Hough transform
def detect_yellow_and_lines(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
    result = cv2.bitwise_and(image, image, mask=mask)

    # Optional: convert to grayscale for Hough
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 200)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30, minLineLength=40, maxLineGap=10)
    output = image.copy()
    if lines is not None:
        for l in lines:
            x1, y1, x2, y2 = l[0]
            cv2.line(output, (x1, y1), (x2, y2), (0, 255, 255), 2)
    return output

# Apply on dark and bright images
output_dark = detect_yellow_and_lines(darker)
output_bright = detect_yellow_and_lines(brighter)

# Show all results
try:
    while True:
        cv2.imshow("Original Cropped", crop_img)
        cv2.imshow("Darkened + Lane Detection", output_dark)
        cv2.imshow("Brightened + Lane Detection", output_bright)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
    print("Windows closed.")
