import cv2
import numpy as np
 
# Read the original image
img = cv2.imread('tree.jpg') 
img2 = cv2.imread('cube.png') 

# Display original image
cv2.imshow('Original', img)
cv2.imshow('Original2', img2)
 
# Convert to graycsale
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

# Blur the image for better edge detection
img_blur = cv2.GaussianBlur(img_gray, (3,3), 0)
img2_blur = cv2.GaussianBlur(img2_gray, (3,3), 0) 

 
# # Sobel Edge Detection
# sobel = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=3) # Combined X and Y Sobel Edge Detection
# # Display Sobel Edge Detection Images
# cv2.imshow('Sobel X and Y', sobel)

# cv2.waitKey(0)
# cv2.destroyAllWindows()
 
# Canny 
# Edge Detection
t1 = 100
t2 = 200
edges = cv2.Canny(image=img_blur, threshold1=t1, threshold2=t2) # Canny Edge Detection
edges2 = cv2.Canny(image=img2_blur, threshold1=t1, threshold2=t2) # Canny Edge Detection

# Display Canny Edge Detection Image
cv2.imshow('Tree, t1= ' + str(t1) + ", t2= " + str(t2), edges)
# cv2.imshow('Cube, t1= ' + str(t1) + ", t2= " + str(t2), edges2)
print(np.shape(edges))

cv2.waitKey(0)
 
cv2.destroyAllWindows()

