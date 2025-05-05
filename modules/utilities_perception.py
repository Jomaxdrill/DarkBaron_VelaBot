import cv2
import numpy as np
from constants_perception import *

def convert_bgr_to_hsv(image):
	return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def create_hsv_mask(image, color):
    # Get the color ranges from BLOCK_COLORS
    color_info = BLOCK_COLORS[color]
    ranges = color_info['ranges']

    # Initialize an empty mask
    mask = np.zeros(image.shape[:2], dtype=np.uint8)

    # Apply cv2.inRange for each range and combine with bitwise OR
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
	#blurry_median = median_blurry_filter(mask_hsv)
	#remove initial noise
	blurry_mask = blurry_image(image)
	hsv_image = convert_bgr_to_hsv(blurry_mask)
	mask_hsv = create_hsv_mask(hsv_image,color)
	#removing noise
	morph_1 = erosion_to_dilation(mask_hsv)
	#closing small holes inside the foreground objects, or small black points on the object. 
	final_mask = dilation_to_erosion(morph_1)
	contours = get_contours(final_mask)
	return contours
#TODO: Check how to filter noisy contours, areaa too small or big
def find_block(image, color, draw=False):
	contours = process_image_contours(image,color)
	print(f'contours found: {len(contours)}')
	info_contours = [ get_contour_box(cnt) for cnt in contours ]
	image_to_draw = image.copy()
	#determine what to do
	if draw:
		for inf_cnt in info_contours:
			image_to_draw = draw_min_enclosing_rectangle(image_to_draw, inf_cnt)
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

def get_nearest_block(info_contours, pending = False):
	areas_all = [width*height for _,_,width,height in info_contours]
	#if we get in a scenario where there is a block missing but probably is too far to catch priorize the first max find
	if pending:
		return info_contours[areas_all.index(max(areas_all))]
	aspect_ratios = [width/height for _,_,width,height in info_contours]
	filter_noisy_contours = [area for idx, area in enumerate(areas_all) if area >= NOISY_CONTOUR_AREA or aspect_ratios[idx] <=6 ]
	if len(filter_noisy_contours) == 0:
		return False
	return info_contours[areas_all.index(max(filter_noisy_contours))]

def inform_block(image, color_block):
	#process the image to find potential blocks
	_, info_contours = find_block(image, color_block)
	#check if there are any blocks detected
	if len(info_contours):
		#find the block with the greatest area that is also the closest one to the robot
		target_block = get_nearest_block(info_contours)
		return target_block
	return False

def show_area(image, color_block, text_align = 10):
	block_shape, info_contours = find_block(image, color_block, True)
	# print(f'image shape: {image.shape}')
	# # plt.imshow(block_shape)
	# # plt.show()
	# print("Running tests...")
	if len(info_contours) == 0:
		return block_shape
	block_test = get_nearest_block(info_contours)
	area, aspect_ratio,center = area_aspect_ratio_center(block_test)
	#print(f'area: {area}, aspect ratio: {aspect_ratio}, center: {center}')
	#image_center = draw_center_image(block_test)
	aprox_distance = obtain_distance(color_block, aspect_ratio, area)#distance_to_block_by(area, aspect_ratio)
	aprox_angle = angle_block_gripper_by(center)
	image_area = cv2.putText(block_shape,f'Area: {area} pix2',(text_align,50),FONT,2,COLORS_TEXTS[color_block],2)
	image_distance = cv2.putText(image_area,f'dist: {aprox_distance} cm',(text_align,100),FONT,2,COLORS_TEXTS[color_block],2)
	image_aspect = cv2.putText(image_distance,f'aspect: {round(aspect_ratio,3)}',(text_align,150),FONT,2,COLORS_TEXTS[color_block],2)
	image_angle = cv2.putText(image_aspect,f'angle: {round(aprox_angle,2)} deg',(text_align,200),FONT,2,COLORS_TEXTS[color_block],2)
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
	elif area >= NOISY_CONTOUR_AREA:
			if 1 <= aspect_ratio < 3:
				return 'Close Object'
			if 3 < aspect_ratio < 4:
				return 'In range to catch'
			if aspect_ratio >= 4:
				return 'Too Close'
	else:
		return 'Far Object'

def obtain_distance(color, aspect_ratio, area):
    channel = DISTANCE_RANGES[color]
    if not channel:
        raise Exception (f"Color '{color}' not found in distance ranges.")
    if area <= 0:
        return 'Stop'
    if aspect_ratio < 0.55:
        return 'Away'
    if 0.55<= aspect_ratio <= 0.7:
        # Binary search or linear interpolation over 'low_ar'
        area_dist_list = channel['low_ar']
        if area <= area_dist_list[0][0]:
            return 'Away'
        if area >= area_dist_list[-1][0]:
            return 'Close'
        for idx in range(len(area_dist_list) - 1):
            a1, d1 = area_dist_list[idx]
            a2, d2 = area_dist_list[idx + 1]
            if a1 <= area <= a2:
                dist_aprox = (d1 + d2) / 2
                break
        print(f'dist aprox is {dist_aprox} cm')
        if area_dist_list[0][1]>= dist_aprox >=area_dist_list[11][1]:
            return 'Remote'
        elif area_dist_list[12][1]>= dist_aprox >=area_dist_list[22][1]:
            return 'Far'
        elif area_dist_list[23][1]>= dist_aprox >=area_dist_list[44][1]:
            return 'Near'
        else:
            return dist_aprox

    if aspect_ratio > 0.7:
        if aspect_ratio >=6:
            return 'Stop'
        if 3 <= aspect_ratio <= 4:
            return 'Catch'
        # Binary search or linear interpolation over 'low_ar'
        ap_dist_list = channel['high_ar']
        if aspect_ratio <= ap_dist_list[0][0]:
            return 'Near'
        if aspect_ratio >= ap_dist_list[-1][0]:
            return 'Stop'
        for idx in range(len(ap_dist_list) - 1):
            a1, d1 = ap_dist_list[idx]
            a2, d2 = ap_dist_list[idx + 1]
            if a1 <= aspect_ratio <= a2:
                dist_aprox = (d1 + d2) / 2
                break
        print(f'dist aprox is {dist_aprox} cm')
        if ap_dist_list[0][1]>= dist_aprox >=ap_dist_list[11][1]:
            return 'Close'
        else:
            return dist_aprox



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
	print(f'closest block: {closest_block}')
	if closest_block:
		_, aspect, center = area_aspect_ratio_center(closest_block)
		#check if the block is in the gripper
		if 1 < aspect < 4 and (CENTER_X_IMAGE -200 < center[0] < CENTER_X_IMAGE +200) and(560 < center[1] < CAMERA_MAIN_RESOLUTION[1]):
			print('I obtained the block')
			return True
		return False
	return False

#TODO: Play with homography to generate a bird view of the robot

def top_view_plane(image):
# Apply perspective transformation
	return cv2.warpPerspective(image, HOMOGRAPHY_MATRIX,HOMOGRAPHY_SIZE)