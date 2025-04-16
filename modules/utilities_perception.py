import cv2
import numpy as np
CAMERA_MAIN_RESOLUTION = (900,780)

#TODO: TUNE THRESHOLDS FOR RED,GREEN & BLUE
#GREEN
LOWER_GREEN = np.array([45, 60, 165])#145
UPPER_GREEN = np.array([70, 255, 255])
#RED
LOWER_RED = np.array([0, 145, 75])
UPPER_RED = np.array([5, 255, 255])
#BLUE
LOWER_BLUE = np.array([70, 60, 95])
UPPER_BLUE = np.array([120, 255, 255])

#
FONT = cv2.FONT_HERSHEY_PLAIN
#
PIXEL_ANGLE = 0.0616 #angle in degrees that represents a pixel movement
CENTER_X_IMAGE = int(CAMERA_MAIN_RESOLUTION[0]//2)
BLOCK_COLORS = {
	'green':
		{
			'lower': LOWER_GREEN,
			'upper': UPPER_GREEN
		},
	'red':
		{
			'lower': LOWER_RED,
			'upper': UPPER_RED
		},
	'blue':
		{
			'lower': LOWER_BLUE,
			'upper': UPPER_BLUE
		}
}

#TODO: IMPROVE THE THRESHOLDS FOR ARROW
LOWER_GREEN_ARROW = np.array([75, 95, 60]) #50,100,100
UPPER_GREEN_ARROW = np.array([95, 255, 220]) #70,255,255

#COLORS FOR CONTOURS
CIRCLE_COLOR = (255, 255, 0)
CIRCLE_BORDER = 5
RECT_COLOR= (0, 0, 255)
RECT_BORDER = 2
GENERAL = (255, 0, 0)
GENERAL_BORDER = 3


####
GAUSS_KERNEL =  (9,9)
MEDIAN_BLUR_KERNEL = 5
BLOCK_SIZE = 5
APERTURE_SIZE = 7 #sobel kernel 7x7
FREE_K_SIZE = 0.05
UNITARY_VECTOR = [1,0]
FONT1 = cv2.FONT_HERSHEY_PLAIN
def convert_bgr_to_hsv(image):
	return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def create_hsv_mask(image, lower_color, upper_color):
	mask_HSV = cv2.inRange(image, lower_color, upper_color)
	#result_HSV = cv2.bitwise_and(image, image, mask = mask_HSV)
	return mask_HSV

def blurry_image(image_gray):
	return cv2.GaussianBlur(image_gray, GAUSS_KERNEL, 0, 0, cv2.BORDER_DEFAULT)

def median_blurry_filter(image_gray):
	return cv2.medianBlur(image_gray, MEDIAN_BLUR_KERNEL)
def combine_stack(*images):
	stacked_image = np.hstack(images)
	return stacked_image

def get_contours(image):
	contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	print(f'contours found: {len(contours)}')
	return contours

def draw_contours(image, contours):
	return cv2.drawContours(image, contours, -1, GENERAL,GENERAL_BORDER)

def get_contour_circle(contours):
	#calculate moments
	#moments = cv2.Momments(contours[0])
	#get center of the contour
	if not contours:
		return (0,0,0)
	(center_x,center_y), radius = cv2.minEnclosingCircle(contours[0])
	return (center_x, center_y, radius)

def get_contour_box(contour):
	if not len(contour):
		return (0,0,0,0)
	left_pos_x,left_pos_y,width,height = cv2.boundingRect(contour)
	return (left_pos_x,left_pos_y,width,height)


def draw_min_enclosing_rectangle(image, contour):
	#info rectangle
	left_pos_x,left_pos_y,width,height = contour
	return cv2.rectangle(image,(left_pos_x,left_pos_y),(left_pos_x+width,left_pos_y+height),RECT_COLOR,RECT_BORDER)

def draw_min_enclosing_circle(image, contour):
	#inner circle
	#cv2.circle(image, (int(contour[0]), int(contour[1])), CIRCLE_BORDER, CIRCLE_COLOR, -1)
	return cv2.circle(image, (int(contour[0]), int(contour[1])), int(contour[2]), CIRCLE_COLOR, CIRCLE_BORDER)

def convert_bgr_to_hsv(image):
	return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
def get_features(image_gray, num_features = 7):#7 DEFAULT TO DETECT ARROWS
	features = cv2.goodFeaturesToTrack(image_gray,num_features,0.01,10)
	if features is None:
		return None
	features = np.intp(features)
	features = np.squeeze(features)
	return features
def find_tip_arrow(features):
	## * the distance between the tip and the points that define the tail of the arrow should be the highest distance
	##* save the coordinate points
	##* get the 4 top distances and store the 8 coordinates (sort), remove dupiclates between the 8 coordinates
	##* between these tuples there might be an index that's more encountered than the others, that's the tip

	if np.size(features,0) != 7:
		return None
	comparison_array = features.copy()
	distances = [] #seven interest point each one has certain legnth with other length
	for idx,point1 in enumerate(features):
		for idx2,point2 in enumerate(comparison_array):
			distance = np.linalg.norm(point1 - point2)
			if not distance:
				continue
			distances.append((distance, (idx,idx2)))
	# for element in distances:
	# 	print(element)
	distances.sort(reverse=True)
	#get the top 4 distances
	top_4 = distances[:4]
	triangle_points = np.array([ point[1] for point in top_4 ])
	triangle_numbers = []
	#flatten array
	triangle_numbers = triangle_points.ravel()
	index_tip = get_mode(triangle_numbers)
	return features[index_tip]

def get_mode(numbers):
	potential_mode = numbers[0]
	max_count = 0
	elements_to_check = set(numbers)
	for element in elements_to_check:
		count = 0

		for point in numbers:
			if element == point:
				count += 1
		if count > max_count:
			max_count = count
			potential_mode = element
	return potential_mode

def bounded_rectangle_rotate(contour_pts):
	return cv2.minAreaRect(contour_pts)

def get_orientation(tip,contours):
	#* apply the bounded_rectangle_rotate function to get the center of the rotated min rectangle
	#* get the vector between the tip and the center of the rotated min rectangle
	#* get the angle with respect to the horizontal axis
	#*determine its cuadrant and hence a tolerance to determine if it's up,down,left or right
	center,_,_= bounded_rectangle_rotate(contours)
	vector_direction = tip-center #hed of arrow must be tip
	#* get the angle via calculating the dot product or trigonomrtry
	# print(f'vector direction is {vector_direction}')
	return np.arctan2(vector_direction[1], vector_direction[0])

def find_block(image, color):
	hsv_image = convert_bgr_to_hsv(image)
	mask_hsv = create_hsv_mask(hsv_image,BLOCK_COLORS[color]['lower'], BLOCK_COLORS[color]['upper'])
	blurry_mask = median_blurry_filter(mask_hsv)
	contours = get_contours(blurry_mask)
	print(f'contours found: {len(contours)}')
	info_contours = []
	for cnt in contours:
		info_cnt = get_contour_box(cnt)
		info_contours.append(info_cnt)
	image_to_draw = image.copy()
	for inf_cnt in info_contours:
		image_to_draw = draw_min_enclosing_rectangle(image_to_draw, inf_cnt)
	# image_block_detected = draw_min_enclosing_rectangle(image, info_contour)
	# image_contours = draw_contours(image, contours)
	return image_to_draw, info_contours


def center_area_aspect_ratio(contour):
	left_pos_x,left_pos_y,width,height = contour
	#* get the area of the rectangle that holds the contour
	area = width * height
	aspect_ratio = float(width/height)
	#* get center of the rectangle
	center_x = int(width//2)+ left_pos_x
	center_y = int(height//2) + left_pos_y
	center = (center_x,center_y)
	return area, aspect_ratio,center

#get the angle in degrees from the center of the rectangle to the center of the image
def angle_block_gripper_by(center_block):
	return (CENTER_X_IMAGE - center_block[0]) * PIXEL_ANGLE

def verify_gripper_by(state):
    pass

def distance_to_block_by(area):
	aprox_distance = -0.002015*area + 69.156
	# if area > 42500:
	# 	aprox_distance = area
	return aprox_distance

