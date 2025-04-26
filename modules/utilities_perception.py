import cv2
import numpy as np
from utilities_camera import CAMERA_MAIN_RESOLUTION

#TODO: TUNE THRESHOLDS FOR RED,GREEN & BLUE
#TODO: UNDERSTAND HOW ILLUMINATION CAN AFFECT THE MEASUREMENTS
#GREEN
LOWER_GREEN = np.array([45, 60, 100])#145 #155
UPPER_GREEN = np.array([70, 255, 255])
#RED
LOWER_RED = np.array([0, 145, 100])
UPPER_RED = np.array([5, 255, 255])
#BLUE
LOWER_BLUE = np.array([80, 65, 100])
UPPER_BLUE = np.array([150, 255, 255])

#BLACK
LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 255, 55])

#
FONT = cv2.FONT_HERSHEY_PLAIN
#
NOISY_CONTOUR_AREA = 1000 #minimum area of the contour to be considered
PIXEL_ANGLE = 0.061 #angle in degrees that represents a pixel movement #0.0605
CENTER_X_IMAGE = int(CAMERA_MAIN_RESOLUTION[0]//2)
GRIPPER_COLOR = 'black'
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
		},
	'black':
		{
			'lower': LOWER_BLACK,
			'upper': UPPER_BLACK
		}
}

#TODO: IMPROVE THE THRESHOLDS FOR ARROW
LOWER_GREEN_ARROW = np.array([75, 95, 60]) #50,100,100
UPPER_GREEN_ARROW = np.array([95, 255, 220]) #70,255,255

#COLORS FOR CONTOURS
CIRCLE_COLOR = (255, 255, 0)
CIRCLE_BORDER = 5
RECT_COLOR= (128, 0, 128)
RECT_BORDER = 2
GENERAL = (255, 0, 0)
GENERAL_BORDER = 3
CROSS_COLOR = (0, 255, 255)
CROSS_BORDER = 2
CROSS_LENGTH = 50
####
GAUSS_KERNEL =  (9,9)
MEDIAN_BLUR_KERNEL = 9
BLOCK_SIZE = 7
APERTURE_SIZE = 7 #sobel kernel 7x7
FREE_K_SIZE = 0.05
FONT1 = cv2.FONT_HERSHEY_PLAIN

#LOOK UP TABLE FOR DEPTH ESTIMATION
# Lookup table data (area in cm², distance in cm)
# DISTANCES_Y = np.array([67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 
#                         39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16])
# AREAS_X = np.array([2400, 2542, 2583, 2646, 2688, 2925, 2970, 2992, 3220, 3337, 3456, 3577, 3626, 3675, 3850, 3978, 4240, 4374, 4510,
#                     4704, 4902, 5192, 5310, 5460, 5828, 6048, 6336, 6666, 7072, 7314, 7776, 8103, 8562, 9009, 9360, 10044, 10668,
#                     10920, 12193, 13080, 13630, 14453, 15500, 16480, 17596, 19203, 20585, 22440, 24375, 26187, 28408, 22605])

AREAS_X = (16848, 17976, 19314, 20520, 22066, 23912, 25452, 28329, 30360, 33120, 35787, 36729, 37350,
		38566, 39083, 39991, 40512, 41175, 41211, 41600, 40752)
DISTANCES_Y = (30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10)

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
	#print(f'contours found: {len(contours)}')
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

def process_image_contours(image,color):
	hsv_image = convert_bgr_to_hsv(image)
	mask_hsv = create_hsv_mask(hsv_image,BLOCK_COLORS[color]['lower'], BLOCK_COLORS[color]['upper'])
	blurry_mask = median_blurry_filter(mask_hsv)
	blurry_mask = blurry_image(mask_hsv)
	contours = get_contours(blurry_mask)
	return contours
#TODO: Check how to filter noisy contours, areaa too small or big
def find_block(image, color):
	contours = process_image_contours(image,color)
	print(f'contours found: {len(contours)}')
	info_contours = [ get_contour_box(cnt) for cnt in contours ]
	image_to_draw = image.copy()
	#determine what to do
	for inf_cnt in info_contours:
		image_to_draw = draw_min_enclosing_rectangle(image_to_draw, inf_cnt)
	# image_block_detected = draw_min_enclosing_rectangle(image, info_contour)
	# image_contours = draw_contours(image, contours)
	return image_to_draw, info_contours


def area_aspect_ratio_center(info_contour):
	left_pos_x,left_pos_y,width,height = info_contour
	#* get the area of the rectangle that holds the contour
	area = width * height
	aspect_ratio = float(width/height)
	#* get center of the rectangle
	center_x = int(width//2)+ left_pos_x
	center_y = int(height//2) + left_pos_y
	center = (center_x,center_y)
	return area, aspect_ratio,center

def get_nearest_block(info_contours):
	areas_all = [width*height for _,_,width,height in info_contours]
	filter_noisy_contours = [area for area in areas_all if area >= NOISY_CONTOUR_AREA]
	return info_contours.index(max(filter_noisy_contours))

def inform_block(image, color_block):
	#process the image to find potential blocks
	_, info_contours = find_block(image, color_block)
	#check if there are any blocks detected
	if len(info_contours):
		#find the block with the greatest area that is also the closest one to the robot
		target_block = get_nearest_block(info_contours)
		return target_block
	return False

def show_area(image, block_color):
	block_shape, info_contours = find_block(image, block_color)
	# print(f'image shape: {image.shape}')
	# # plt.imshow(block_shape)
	# # plt.show()
	# print("Running tests...")
	if len(info_contours) == 0:
		return block_shape
	block_test = info_contours[0]
	area, aspect_ratio,center = area_aspect_ratio_center(block_test)
	#print(f'area: {area}, aspect ratio: {aspect_ratio}, center: {center}')
	#image_center = draw_center_image(block_test)
	aprox_distance = distance_to_block_by(area, aspect_ratio)
	aprox_angle = angle_block_gripper_by(center)
	image_area = cv2.putText(block_shape,f'Area: {area} pix2',(10,50),FONT,2,(0,255,255),2)
	# text_distance = round(aprox_distance,2) if aprox_distance is not None else aprox_distance
	image_distance = cv2.putText(image_area,f'dist: {aprox_distance} cm',(10,100),FONT,2,(255,0,255),2)
	image_aspect = cv2.putText(image_distance,f'aspect: {round(aspect_ratio,3)}',(10,150),FONT,2,(255,255,0),2)
	image_angle = cv2.putText(image_aspect,f'angle: {round(aprox_angle,2)} deg',(10,200),FONT,2,(255,255,128),2)
	return image_angle

#get the angle in degrees from the center of the rectangle to the center of the image
def angle_block_gripper_by(center_block):
	return (CENTER_X_IMAGE - center_block[0]) * PIXEL_ANGLE


def draw_center_image(image, center = CAMERA_MAIN_RESOLUTION):
	#draw center of the image
	cv2.circle(image, (int(center[0]), int(center[1])), 5, CROSS_COLOR, -1)
	cv2.line(image, (int(center[0])-CROSS_LENGTH, int(center[1])), (int(center[0]+CROSS_LENGTH), int(center[1] )), CROSS_COLOR, CROSS_BORDER)
	cv2.line(image, (int(center[0]), int(center[1])-CROSS_LENGTH), (int(center[0]), int(center[1] +CROSS_LENGTH)), CROSS_COLOR, CROSS_BORDER)
	return image
def crop_half_image(image, center = CAMERA_MAIN_RESOLUTION):
	image[:center[1], :] = 255
	return image

def distance_to_block_by(area, aspect_ratio):
	#aprox_distance= (np.log(45923/area))/0.056
	#aprox_distance = -0.002015*area + 69.156
	# Handle edge cases: area outside the table range
	#*aspect ratio plays an important role in the close distance
	if aspect_ratio < 1:
		if area < AREAS_X[0] or area < 1000:
			return 'Far Object'
		if area == AREAS_X[-1]:
			return DISTANCES_Y[-1]
		# Search for the area in the table
		for idx in range(len(AREAS_X) - 1):
			if area == AREAS_X[idx]:  # Exact match
				return DISTANCES_Y[idx]
			elif AREAS_X[idx] < area < AREAS_X[idx + 1]:  # Area is between AREAS_X[idx] and AREAS_X[idx+1]
				# Return the average of the corresponding DISTANCES_Y
				return round( (DISTANCES_Y[idx] + DISTANCES_Y[idx + 1]) / 2 ,2)
	else:
		if 1 <= aspect_ratio < 3:
			return 'Close Object'
		if aspect_ratio > 3:
			return 'In range to catch'

#TODO: Check if gripper is open or close by an image
def check_gripper_state(image):
	contours = process_image_contours(image,GRIPPER_COLOR)
	#potentially the gripper is open
	if len(contours) == 2:
		return 'OPEN'
	#potentially the gripper is close
	elif len(contours) == 1:
		return 'CLOSE'
	else:
		return False

#check if block is in the gripper
def check_block_gripper(image, color):
	closest_block = inform_block(image, color)
	if closest_block:
		info_contour = get_contour_box(closest_block)
		_, aspect, center = area_aspect_ratio_center(info_contour)
		#check if the block is in the gripper
		if aspect > 3 and (CENTER_X_IMAGE -200 <center[0] < CENTER_X_IMAGE +200) and(560 < center[1] < CAMERA_MAIN_RESOLUTION[1]//2):
			return True
		return False
	#check if the block is in the gripper
	return False

#TODO: Play with homography to generate a bird view of the robot

