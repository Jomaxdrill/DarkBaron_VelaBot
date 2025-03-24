

import cv2
import numpy as np

LOWER_GREEN = np.array([50, 210, 35]) #50,100,100
UPPER_GREEN = np.array([100, 255, 255]) #70,255,255
#TODO: IMPROVE THE THRESHOLDS FOR ARROW
LOWER_GREEN_ARROW = np.array([75, 95, 60]) #50,100,100
UPPER_GREEN_ARROW = np.array([95, 255, 220]) #70,255,255
GAUSS_KERNEL =  (9,9)
MEDIAN_BLUR_KERNEL = 3
BLOCK_SIZE = 5
APERTURE_SIZE = 7 #sobel kernel 7x7
FREE_K_SIZE = 0.05
UNITARY_VECTOR = [1,0]
FONT1 = cv2.FONT_HERSHEY_PLAIN
def convert_rgb_to_hsv(image):
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
	return contours

def draw_contours(image, contours):
	return cv2.drawContours(image, contours, 5, (255, 0, 0), 3)

def get_info_contour(contours):
	#calculate moments
	#moments = cv2.Momments(contours[0])
	#get center of the contour
	if not contours:
		return (0,0,0)
	(center_x,center_y), radius = cv2.minEnclosingCircle(contours[0])
	return (center_x, center_y, radius)

def draw_min_enclosing_circle(image, contour):
	#center circle
	cv2.circle(image, (int(contour[0]), int(contour[1])), 10, (0,0, 255), -1)
	return cv2.circle(image, (int(contour[0]), int(contour[1])), int(contour[2]), (255,0, 255), 5)

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
	#* get the angle via calculating the dot product
	# print(f'vector direction is {vector_direction}')
	angle = np.arctan2(vector_direction[1], vector_direction[0])
	#TODO: IMPROVE THE CONDITION
	if angle <= 0:
		return 'UP'
		# angle = 2*np.pi - angle
	angle = np.degrees(angle)
	#determine direction		
	if (angle > 315 and angle <=360) or angle <= 45:
		return 'RIGHT'
	elif angle > 45 and angle <= 135:
		return 'DOWN'
	elif angle > 135 and angle <= 225:
		return 'LEFT'
	# elif angle > 225 and angle <= 315:
	# 	return 'UP'
	else:
		return '...'

def show_features_in_gray(image_gray,features):
	image_features = cv2.cvtColor(image_gray, cv2.COLOR_GRAY2RGB)
	for i in features:
		x,y = i.ravel()
		cv2.circle(image_features,(x,y),3,255,-1)
	return image_features
def show_features_in_color(image_rgb,features):
	image_features = image_rgb.copy()
	for i in features:
		x,y = i.ravel()
		cv2.circle(image_features,(x,y),3,255,-1)
	return image_features

def add_text(image, text):
	return cv2.putText(image,text,(30,30),FONT1,2,(0,255,255),4)

def detect_green(image):
	hsv_image = convert_rgb_to_hsv(image)
	mask_hsv = create_hsv_mask(hsv_image,LOWER_GREEN, UPPER_GREEN)
	blurry_mask = median_blurry_filter(mask_hsv)
	contours = get_contours(blurry_mask)
	info_contour = get_info_contour(contours)
	image_green_detected = draw_min_enclosing_circle(image, info_contour)
	return image_green_detected

def detect_arrow(image):
	hsv_image = convert_bgr_to_hsv(image)
	mask_hsv = create_hsv_mask(hsv_image,LOWER_GREEN_ARROW, UPPER_GREEN_ARROW)
	blurry_mask = blurry_image(mask_hsv)
	median_blurry = median_blurry_filter(blurry_mask)
	features_image = get_features(median_blurry)
	if features_image is None:
		#print('No tip arrow found')
		return image
	tip_arrow = find_tip_arrow(features_image)
	if tip_arrow is None:
		#print('No tip arrow found')
		return image
	orientation_arrow = get_orientation(tip_arrow, features_image)
	contours_original_image = show_features_in_color(image, features_image)
	return add_text(contours_original_image, f'{orientation_arrow}')

# def define_orientation(center,tip):
#     if center[0]


def detect_corners(image_gray, num_corners):
    return cv2.goodFeaturesToTrack(image_gray,
                                   num_corners,0.01,10)