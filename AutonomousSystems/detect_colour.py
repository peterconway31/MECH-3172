# import packages
import numpy as np
import cv2

# load image
im_name = 'AIML_edit2'
image_raw = cv2.imread(im_name+'.jpg')
image = cv2.resize(image_raw, (1008,756))

# Lower and upper bound defining landing pad colour
boundaries = [
	([110, 155, 200], [150, 205, 250])
]
(lower, upper) = boundaries[0]
lower = np.array(lower, dtype = "uint8")
upper = np.array(upper, dtype = "uint8")

# Create mask with only colours within boundary
mask = cv2.inRange(image, lower, upper)
output = cv2.bitwise_and(image, image, mask = mask)

# findContours function relies on a pure black and white image
grayImage = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
(thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)

# find contours
contours, hierarchy = cv2.findContours(blackAndWhiteImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
ctr = contours[0:4]
image_cts = image.copy()
cv2.drawContours(image_cts, ctr, 0, (0, 255, 0), 2)

# show and save results
cv2.imshow("images", np.hstack([image, image_cts]))
cv2.imwrite(im_name+'_ctr'+'.jpg', image_cts)

# kill program on key press
cv2.waitKey(0)
cv2.destroyAllWindows()

