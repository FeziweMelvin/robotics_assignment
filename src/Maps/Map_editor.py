import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import binary_erosion, binary_opening, square

# Read the image
img = cv2.imread('map.png', 0)  # Read as grayscale (0)

# Flip the image vertically
img_flipped = np.flipud(img)

# Plot the flipped original image
plt.imshow(img_flipped, cmap='gray')
plt.title('Flipped Original Image')
plt.colorbar()  # Optional color bar
plt.gca().invert_yaxis()  # Invert y-axis to label from bottom to top
plt.show()

# Thresholding to binary image
ret, bw_img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

# Change the image to binary form (255 for black, 0 for white)
bw_img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)[1]

# Perform binary opening to smooth the image
kernel = square(17)
bw_img = binary_opening(bw_img, kernel)

# Perform binary erosion to enlarge black areas (obstacles)
bw_img = binary_erosion(bw_img, kernel)

# Display the processed image
plt.imshow(bw_img, cmap='gray')
plt.title('Processed Image')

# Example: Plot a point at (100, 200)
plt.scatter([401], [400], color='red', marker='x', label='Default Start Position (401, 400)')
plt.legend()

plt.show()

# Convert processed image to obstacle map (1 for obstacles, 0 for open space)
Processed_Image = np.where(bw_img == 0, 1, 0)

# Save the processed image
cv2.imwrite('Processed_Image.png', Processed_Image * 255)  # Multiply by 255 to convert binary image to grayscale

# Save image array to text file
np.savetxt("OccupancyMap.txt", Processed_Image, delimiter=",", fmt='%d')

# Read the saved mapArray.txt to verify
imgArr = np.loadtxt("OccupancyMap.txt", delimiter=",").astype(int)
print("Shape of loaded array:", imgArr.shape)
