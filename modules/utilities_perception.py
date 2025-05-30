import cv2
import numpy as np
from .constants_perception import *

#*------------------------
###* FILTERING
#*------------------------
def convert_bgr_to_hsv(image):
	return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

def create_hsv_mask(image, color):
	color_info = BLOCK_COLORS[color]
	ranges = color_info['ranges']
	mask = np.zeros(image.shape[:2], dtype=np.uint8)
	# get ranges and combine them
	for range_info in ranges:
		lower = range_info['lower']
		upper = range_info['upper']
		range_mask = cv2.inRange(image, lower, upper)
		mask = cv2.bitwise_or(mask, range_mask)
	return mask

def blurry_image(image_gray):
	return cv2.GaussianBlur(image_gray, GAUSS_KERNEL, 0, 0, cv2.BORDER_DEFAULT)

def median_blurry_filter(image_gray):
	return cv2.medianBlur(image_gray, MEDIAN_BLUR_KERNEL)

def erosion_to_dilation(image_gray):
	erosion = cv2.morphologyEx(image_gray, cv2.MORPH_OPEN, KERNEL_MORPH)
	return erosion
def dilation_to_erosion(image_gray):
	dilation = cv2.morphologyEx(image_gray, cv2.MORPH_CLOSE, KERNEL_MORPH)
	return dilation

def process_image_contours(image, color, crop=True):
	#remove upper zone that doesn't matter
	cropped_image = crop_half_image(image) if crop else image
	#blurrying can help reduce noise
	blurry_mask = blurry_image(cropped_image)
	hsv_image = convert_bgr_to_hsv(blurry_mask)
	#convert to hsv
	mask_hsv = create_hsv_mask(hsv_image,color)
	#median blur can filter pepper noise
	blurry_median = median_blurry_filter(mask_hsv)
	morph_1 = erosion_to_dilation(blurry_median)
	#closing small holes inside the foreground objects, or small black points on the object.
	final_mask = dilation_to_erosion(morph_1)
	contours = get_contours(final_mask)
	return contours

#*------------------------
###* CONTOUR DETECTION AND DISPLAYING
#*------------------------

def get_contours(image):
	contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	return contours

def draw_contours(image, contours,color):
	return cv2.drawContours(image, contours, -1, GENERAL,GENERAL_BORDER)

def get_contour_circle(contours):
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
	left_pos_x,left_pos_y,width,height = contour
	return cv2.rectangle(image,(left_pos_x,left_pos_y),(left_pos_x+width,left_pos_y+height),RECT_COLOR,RECT_BORDER)

def draw_min_enclosing_circle(image, contour):
	# add inner circle
	#cv2.circle(image, (int(contour[0]), int(contour[1])), CIRCLE_BORDER, CIRCLE_COLOR, -1)
	return cv2.circle(image, (int(contour[0]), int(contour[1])), int(contour[2]), CIRCLE_COLOR, CIRCLE_BORDER)

def get_features(image_gray, num_features = 7):#7 DEFAULT TO DETECT ARROWS
	features = cv2.goodFeaturesToTrack(image_gray,num_features,0.01,10)
	if features is None:
		return None
	features = np.intp(features)
	features = np.squeeze(features)
	return features

#*------------------------
###* BLOCKS DETECTION AND FEATURES
#*------------------------

def inform_block(image, color_block, exhaustive=True):
	#process the image to find potential blocks
	_, info_contours = find_blocks(image, color_block,draw=False,crop=True)
	#check if there are any blocks detected
	if len(info_contours):
		#find the block with the greatest area that is also the closest one to the robot
		#in exhaustive find the first coincidence, important for search
		target_block = get_nearest_block(info_contours) if exhaustive else info_contours[0]
		return target_block
	return False

def find_blocks(image, color, draw=False, crop=True):
	contours = process_image_contours(image, color, crop)
	print(f'contours found of {color}: {len(contours)}')
	info_contours = [ get_contour_box(cnt) for cnt in contours ]
	image_to_draw = image.copy()
	if draw:
		for inf_cnt in info_contours:
			image_to_draw = draw_min_enclosing_rectangle(image_to_draw, inf_cnt)
	return image_to_draw, info_contours

def area_aspect_ratio_center(info_contour):
	left_pos_x,left_pos_y,width,height = info_contour
	area = width * height
	aspect_ratio = float(width/height)
	center_x = int(width//2) + left_pos_x
	center_y = int(height//2) + left_pos_y
	center = (center_x,center_y)
	return area, aspect_ratio, center

def get_info_all_blocks(image, color):
	_, info_contours = find_blocks(image, color, draw=False, crop=True)
	if not len(info_contours):
		return []
	areas_all = get_areas(info_contours)
	aspect_ratios = get_aspect_ratios(info_contours)
	#filter noisy contours
	return [contour for idx, contour in enumerate(info_contours)
        if areas_all[idx] >= NOISY_CONTOUR_AREA and
        0.4<= aspect_ratios[idx] <=6 ]

def get_areas(info_contours):
    return [width*height for _,_,width,height in info_contours]

def get_aspect_ratios(info_contours):
    return [width/height for _,_,width,height in info_contours]

def get_nearest_block(info_contours):
	if not len(info_contours):
		return False
	areas_all = get_areas(info_contours)
	aspect_ratios = get_aspect_ratios(info_contours)
	filtered_noisy_contours = [area for idx, area in enumerate(areas_all)
                            if areas_all[idx] >= NOISY_CONTOUR_AREA and
							0.4<= aspect_ratios[idx] <=6 and
							info_contours[idx][3]>10 and info_contours[idx][2]>20]
	if not len(filtered_noisy_contours):
		return False
	return info_contours[areas_all.index(max(filtered_noisy_contours))]

def angle_block_gripper_by(center_block):
	return (CENTER_X_IMAGE - center_block[0]) * PIXEL_ANGLE

#*THE LARGER THE AREA POTENTIALLY CLOSER TO THE OBJECT OR CHANGES IN ASPECT RATIO CAN DERIVE DISTANCE TO OBJECT
def obtain_distance(color, area, aspect_ratio):
	channel = DISTANCE_RANGES[color]
	if not channel:
		raise Exception (f"Color '{color}' not found in distance ranges.")
	if area <= 0:
		return 'Stop', 2
	if aspect_ratio < 0.55:
		return 'Away', 250
	if 0.55<= aspect_ratio <= 0.7:
		# Binary search or linear interpolation over 'low_ar'
		area_dist_list = channel['low_ar']
		if area <= area_dist_list[0][0]:
			return 'Away', 250
		if area >= area_dist_list[-1][0]:
			return 'Close', 28
		for idx in range(len(area_dist_list) - 1):
			a1, d1 = area_dist_list[idx]
			a2, d2 = area_dist_list[idx + 1]
			if a1 <= area <= a2:
				dist_aprox = (d1 + d2) / 2
				break
		print(f'dist aprox block {color} is {dist_aprox} cm')
		if area_dist_list[0][1]>= dist_aprox >=area_dist_list[11][1]:
			return 'Remote', dist_aprox
		elif area_dist_list[11][1]>= dist_aprox >=area_dist_list[22][1]:
			return 'Far', dist_aprox
		elif area_dist_list[22][1]>= dist_aprox >=area_dist_list[44][1]:
			return 'Near', dist_aprox
		else:
			return 'Not sure',dist_aprox
	if aspect_ratio > 0.7:
		if aspect_ratio >=6:
			return 'Stop', 2
		if 2 <= aspect_ratio <= 5.5 and area > 25000:
			return 'Catch', 0.5
		ap_dist_list = channel['high_ar']
		if aspect_ratio <= ap_dist_list[0][0]:
			return 'Near', 10
		if aspect_ratio >= ap_dist_list[-1][0]:
			return 'Stop', 2
		for idx in range(len(ap_dist_list) - 1):
			a1, d1 = ap_dist_list[idx]
			a2, d2 = ap_dist_list[idx + 1]
			if a1 <= aspect_ratio <= a2:
				dist_aprox = (d1 + d2) / 2
				break
		print(f'dist aprox block {color} is {dist_aprox} cm')
		if ap_dist_list[0][1]>= dist_aprox >=ap_dist_list[10][1]:
			return 'Close', dist_aprox
		else:
			return 'Not sure', dist_aprox

def check_block_gripper(image, color):
	closest_block = inform_block(image, color)
	print(f'checking block {color} to be considered caught')
	if closest_block:
		area, aspect_ratio, center = area_aspect_ratio_center(closest_block)
		angle  = angle_block_gripper_by(center)
		range_block, _ = obtain_distance(color, area, aspect_ratio)
		#*check angle is between (-1,1)
		if np.abs(angle) > 4:
			print(f'Not in proper angle to be considered caught, angle is {angle}')
			return False
		if range_block != 'Catch':
			print(f'Not in range to be considered caught, range is {range_block}')
			return False
		# #*on average aspect ratio is between 1.5-3
		if aspect_ratio < 1.5:
			print(f'Not aspect ratio to be considered caught, ap is {aspect_ratio}')
			return False
		final_check = in_gripper_zone(center, True)
		if not final_check:
			return False
		print(f'Block {color} was caught yeaaah')
		return True
	print(f'Block {color} non caught')
	return False

def in_gripper_zone(center, cropped= False):
	#*check if the block is in the horizontal zone
	if not (CENTER_X_IMAGE -200 < center[0] < CENTER_X_IMAGE +200):
		print(f'Not vert aligned to consider gripped,  \
		center x image is {CENTER_X_IMAGE} while center x block is {center[0]}')
		return False
	height =  CAMERA_MAIN_RESOLUTION[1]//2 if cropped else CAMERA_MAIN_RESOLUTION[1]
	if not (height-220 < center[1] < height):
		print(f'Not hort aligned to consider gripped,  \
		center y image is {height} while center  y block is {center[1]}')
		return False
	return True

def hide_caught_block(image):
	height, width = image.shape[:2]
	if height < CAMERA_MAIN_RESOLUTION[1]:
		image[int(height*0.50) :,int(0.25*width):int(0.75*width)] = 255
	else:
		image[int(height*0.75) :,int(0.25*width):int(0.75*width)] = 255
	return image

#useful for alignment of robot and gripper
def draw_center_image(image, center = CAMERA_MAIN_RESOLUTION):
	#draw center of the image
	cv2.circle(image, (int(center[0]), int(center[1])), 5, CROSS_COLOR, -1)
	cv2.line(image, (int(center[0])-CROSS_LENGTH, int(center[1])), (int(center[0]+CROSS_LENGTH), int(center[1] )), CROSS_COLOR, CROSS_BORDER)
	cv2.line(image, (int(center[0]), int(center[1])-CROSS_LENGTH), (int(center[0]), int(center[1] +CROSS_LENGTH)), CROSS_COLOR, CROSS_BORDER)
	return image

def add_white_area(image):
	blank_img = np.ones((CAMERA_MAIN_RESOLUTION[1]//2, CAMERA_MAIN_RESOLUTION[0], 3), dtype=np.uint8) * 255
	result_img = np.vstack((blank_img, image))
	return result_img

def convert_white_area(image):
	image[0:CAMERA_MAIN_RESOLUTION[1]//2, :] = 255
	return image

def crop_half_image(image, center = CAMERA_MAIN_RESOLUTION):
	cropped_image = image[center[1]//2:, 0:]
	return cropped_image

#*------------------------
###* FOR VISUALIZATION WITH VIDEO FEED
#*------------------------
def show_area(image, color_block, text_align = 10):
	block_shape, info_contours = find_blocks(image, color_block, True, False)
	if not len(info_contours):
		return block_shape
	block_test = get_nearest_block(info_contours)
	print(f'block {color_block} closest is:', block_test)
	if not block_test:
		return image
	area, aspect_ratio,center = area_aspect_ratio_center(block_test)
	aprox_range, aprox_distance = obtain_distance(color_block,area, aspect_ratio)
	aprox_angle = angle_block_gripper_by(center)
	image_area = cv2.putText(block_shape,f'{area} pix2',(text_align,50),FONT,2,COLORS_TEXTS[color_block],2)
	image_distance = cv2.putText(image_area,f'{aprox_range},{aprox_distance}cm',(text_align,100),FONT,2,COLORS_TEXTS[color_block],2)
	image_aspect = cv2.putText(image_distance,f'ap:{center} {round(aspect_ratio,2)}',(text_align,150),FONT,2,COLORS_TEXTS[color_block],2)
	image_angle = cv2.putText(image_aspect,f'{round(aprox_angle,2)} deg',(text_align,200),FONT,2,COLORS_TEXTS[color_block],2)
	return image_angle

#*###############################
#*BONUS FUNCTIONS FOR USE IN THE FUTURE
#*###############################
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


def top_view_plane(image):
# Apply perspective transformation
	return cv2.warpPerspective(image, HOMOGRAPHY_MATRIX,HOMOGRAPHY_SIZE)