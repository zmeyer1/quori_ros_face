import cv2
import time

# Load the image
image = cv2.imread('image.jpg')

# Create a named window with flags to remove the frame
cv2.namedWindow('Frameless Image', cv2.WINDOW_NORMAL)

# Show the image
cv2.imshow('Frameless Image', image)
cv2.setWindowProperty('Frameless Image', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)


# Wait for a key press and then close
cv2.waitKey(1)


cv2.imshow('Frameless Image', cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))

cv2.waitKey(1)

# Show the image
cv2.imshow('Frameless Image', image)

cv2.waitKey(1)
cv2.destroyAllWindows()