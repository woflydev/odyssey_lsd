import os
import numpy as np
import cv2
import tensorflow as tf
import math

def show(window_name, frame, show_img):
	if show_img:
		cv2.imshow(window_name, frame)

def roi(image):
	height, width = image.shape[:2]
	start_row, start_col = int(height * .5), int(0)
	end_row, end_col = int(height), int(width)
	cropped_img = image[start_row : end_row , start_col : end_col]
	return cropped_img, cropped_img.shape[0], cropped_img.shape[1]
	#return cropped_img, height, width
	#return image, image.shape[1], image.shape[0]

def add_to_mask(lines, mask_shape):
	mask = np.zeros((180, 640), dtype=np.uint8)
	mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

	for line in lines:
		x_start, y_start, x_end, y_end = [int(val) for val in line]
		cv2.line(mask, (x_start, y_start), (x_end, y_end), [255,255,255], 10)
	
	return mask

def calc_lines(frame, line_segments, height, width):
	"""
	If all line slopes are < 0: then only detected left lane
	If all line slopes are > 0: then only detected right lane
	"""
	lane_lines = []
	if line_segments is None:
		print('WARNING: No line_segment segments detected')
		return lane_lines

	left_fit = []
	right_fit = []

	boundary = 1 / 3
	left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
	right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

	for x1, y1, x2, y2 in line_segments:
		if x1 == x2:
			#print('skipping vertical line segment (slope=inf): %s' % line_segment)
			continue
		fit = np.polyfit((x1, x2), (y1, y2), 1)
		slope = fit[0]
		intercept = fit[1]
		if slope < 0:
			if x1 < left_region_boundary and x2 < left_region_boundary:
				left_fit.append((slope, intercept))
		else:
			if x1 > right_region_boundary and x2 > right_region_boundary:
				right_fit.append((slope, intercept))

	left_fit_average = np.average(left_fit, axis=0)
	if len(left_fit) > 0:
		lane_lines.append(make_points(frame, left_fit_average))

	right_fit_average = np.average(right_fit, axis=0)
	if len(right_fit) > 0:
		lane_lines.append(make_points(frame, right_fit_average))

	line_image = np.zeros_like(frame)
	if lane_lines is not None:
		for line in lane_lines:
			for x1, y1, x2, y2 in line:
				p1 = (x1, y1)
				p2 = (x2, y2)
				theta = np.arctan2(p1[1]-p2[1], p1[0]-p2[0])
				endpt_x = int(p1[0] - (width-560)*np.cos(theta))
				endpt_y = int(p1[1] - (width-560)*np.sin(theta))
				#cv2.line(line_image, (p1[0], p1[1]), (endpt_x, endpt_y), (0, 255, 0), 10)
				cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
	line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

	return line_image, lane_lines

def calc_steering(frame, lane_lines):
	if len(lane_lines) == 0:
		print('INFO: No lines detected...')
		return 90 # used to be -90, not sure

	height, width, _ = frame.shape
	if len(lane_lines) == 1:
		print('WARNING: Only 1 lane detected...')
		x1, _, x2, _ = lane_lines[0][0]
		x_offset = x2 - x1
	else:
		_, _, left_x2, _ = lane_lines[0][0]
		_, _, right_x2, _ = lane_lines[1][0]
		camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
		mid = int(width / 2 * (1 + camera_mid_offset_percent))
		x_offset = (left_x2 + right_x2) / 2 - mid

	# find the steering angle, which is angle between navigation direction to end of center line
	y_offset = int(height / 2)

	angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
	angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
	steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by front wheel

	#print('DEBUG: new steering angle: %s' % steering_angle)
	return steering_angle

def stabilize(current, new, num_lanes, max_confident_deviation=5, max_unsure_deviation=1):
	"""
	Using last steering angle to stabilize the steering angle
	This can be improved to use last N angles, etc
	if new angle is too different from current angle, only turn by max_angle_deviation degrees
	"""
	if num_lanes == 2:
		# if both lane lines detected, then we can deviate more
		max_angle_deviation = max_confident_deviation
	elif num_lanes == 1:
		# if only one lane detected, don't deviate too much
		max_angle_deviation = max_unsure_deviation
	elif num_lanes == 0:
		max_angle_deviation = 100
	
	angle_deviation = new - current
	if abs(angle_deviation) > max_angle_deviation:
		stabilized_steering_angle = int(current
										+ max_angle_deviation * angle_deviation / abs(angle_deviation))
	else:
		stabilized_steering_angle = new
	print('INFO: Proposed angle: %s, stabilized angle: %s' % (new, stabilized_steering_angle))
	return stabilized_steering_angle

def heading(frame, angle, height, width):
	heading_image = np.zeros_like(frame)
	height, width, _ = frame.shape

	# figure out the heading line from steering angle
	# heading line (x1,y1) is always center bottom of the screen
	# (x2, y2) requires a bit of trigonometry

	radians = angle / 180.0 * math.pi
	x1 = int(width / 2)
	y1 = height
	x2 = int(x1 - height / 2 / math.tan(radians))
	y2 = int(height / 2)

	cv2.line(heading_image, (x1, y1), (x2, y2), (0, 0, 255), 10)
	heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

	return heading_image

def make_points(frame, line):
	height, width, _ = frame.shape
	slope, intercept = line
	y1 = height  # bottom of the frame
	y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

	# bound the coordinates within the frame
	x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
	x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
	return [[x1, y1, x2, y2]]

def pwm(speed, theta):
	try:
		theta = ((theta + 180) % 360) - 180  # normalize value to [-180, 180)
		speed = min(max(0, speed), 100)              # normalize value to [0, 100]
		v_a = speed * (45 - theta % 90) / 45          # falloff of main motor
		v_b = min(100, 2 * speed + v_a, 2 * speed - v_a)  # compensation of other motor
		if theta < -90: return -v_b, -v_a
		if theta < 0:   return -v_a, v_b
		if theta < 90:  return v_b, v_a
		return [v_a, -v_b]
	except:
			print('error')

def pred_lines(image, interpreter, input_details, output_details, input_shape=[512, 512], score_thr=0.10, dist_thr=20.0):
	h, w, _ = image.shape
	h_ratio, w_ratio = [h / input_shape[0], w / input_shape[1]]

	resized_image = np.concatenate([cv2.resize(image, (input_shape[0], input_shape[1]), interpolation=cv2.INTER_AREA), np.ones([input_shape[0], input_shape[1], 1])], axis=-1)
	batch_image = np.expand_dims(resized_image, axis=0).astype('float32')
	interpreter.set_tensor(input_details[0]['index'], batch_image)
	interpreter.invoke()

	pts = interpreter.get_tensor(output_details[0]['index'])[0]
	pts_score = interpreter.get_tensor(output_details[1]['index'])[0]
	vmap = interpreter.get_tensor(output_details[2]['index'])[0]

	start = vmap[:,:,:2]
	end = vmap[:,:,2:]
	dist_map = np.sqrt(np.sum((start - end) ** 2, axis=-1))

	segments_list = []
	for center, score in zip(pts, pts_score):
		y, x = center
		distance = dist_map[y, x]
		if score > score_thr and distance > dist_thr:
			disp_x_start, disp_y_start, disp_x_end, disp_y_end = vmap[y, x, :]
			x_start = x + disp_x_start
			y_start = y + disp_y_start
			x_end = x + disp_x_end
			y_end = y + disp_y_end
			segments_list.append([x_start, y_start, x_end, y_end])
	
	lines = 2 * np.array(segments_list) # 256 > 512
	lines[:,0] = lines[:,0] * w_ratio
	lines[:,1] = lines[:,1] * h_ratio
	lines[:,2] = lines[:,2] * w_ratio
	lines[:,3] = lines[:,3] * h_ratio

	return lines

def pred_squares(image,
				 interpreter,
				 input_details,
				 output_details,
				 input_shape=[512, 512],
				 params={'score': 0.06,
						 'outside_ratio': 0.28,
						 'inside_ratio': 0.45,
						 'w_overlap': 0.0,
						 'w_degree': 1.95,
						 'w_length': 0.0,
						 'w_area': 1.86,
						 'w_center': 0.14}):
	h, w, _ = image.shape
	original_shape = [h, w]

	resized_image = np.concatenate([cv2.resize(image, (input_shape[0], input_shape[1]), interpolation=cv2.INTER_AREA), np.ones([input_shape[0], input_shape[1], 1])], axis=-1)
	batch_image = np.expand_dims(resized_image, axis=0).astype('float32')
	interpreter.set_tensor(input_details[0]['index'], batch_image)
	interpreter.invoke()

	pts = interpreter.get_tensor(output_details[0]['index'])[0]
	pts_score = interpreter.get_tensor(output_details[1]['index'])[0]
	vmap = interpreter.get_tensor(output_details[2]['index'])[0]
	
	start = vmap[:,:,:2] # (x, y)
	end = vmap[:,:,2:] # (x, y)
	dist_map = np.sqrt(np.sum((start - end) ** 2, axis=-1))

	junc_list = []
	segments_list = []
	for junc, score in zip(pts, pts_score):
		y, x = junc
		distance = dist_map[y, x]
		if score > params['score'] and distance > 20.0:
			junc_list.append([x, y])
			disp_x_start, disp_y_start, disp_x_end, disp_y_end = vmap[y, x, :]
			d_arrow = 1.0
			x_start = x + d_arrow * disp_x_start
			y_start = y + d_arrow * disp_y_start
			x_end = x + d_arrow * disp_x_end
			y_end = y + d_arrow * disp_y_end
			segments_list.append([x_start, y_start, x_end, y_end])
			
	segments = np.array(segments_list)
	
	####### post processing for squares
	# 1. get unique lines
	point = np.array([[0, 0]])
	point = point[0]
	start = segments[:,:2]
	end = segments[:,2:]
	diff = start - end
	a = diff[:, 1]
	b = -diff[:, 0]
	c = a * start[:,0] + b * start[:,1]

	d = np.abs(a * point[0] + b * point[1] - c) / np.sqrt(a ** 2 + b ** 2 + 1e-10)
	theta = np.arctan2(diff[:,0], diff[:,1]) * 180 / np.pi
	theta[theta < 0.0] += 180
	hough = np.concatenate([d[:,None], theta[:,None]], axis=-1)

	d_quant = 1
	theta_quant = 2
	hough[:,0] //= d_quant
	hough[:,1] //= theta_quant
	_, indices, counts = np.unique(hough, axis=0, return_index=True, return_counts=True)
	
	acc_map = np.zeros([512 // d_quant + 1, 360 // theta_quant + 1], dtype='float32')
	idx_map = np.zeros([512 // d_quant + 1, 360 // theta_quant + 1], dtype='int32') - 1
	yx_indices = hough[indices,:].astype('int32')
	acc_map[yx_indices[:,0], yx_indices[:,1]] = counts
	idx_map[yx_indices[:,0], yx_indices[:,1]] = indices
	
	acc_map_np = acc_map
	acc_map = acc_map[None,:,:,None]
	
	### fast suppression using tensorflow op
	acc_map = tf.constant(acc_map, dtype=tf.float32)
	max_acc_map = tf.keras.layers.MaxPool2D(pool_size=(5,5), strides=1, padding='same')(acc_map)
	acc_map = acc_map * tf.cast(tf.math.equal(acc_map, max_acc_map), tf.float32)
	flatten_acc_map = tf.reshape(acc_map, [1, -1])
	topk_values, topk_indices = tf.math.top_k(flatten_acc_map, k=len(pts))
	_, h, w, _ = acc_map.shape
	y = tf.expand_dims(topk_indices // w, axis=-1)
	x = tf.expand_dims(topk_indices % w, axis=-1)
	yx = tf.concat([y, x], axis=-1)
	###

	yx = yx[0].numpy()
	indices = idx_map[yx[:,0], yx[:,1]]
	topk_values = topk_values.numpy()[0]
	basis = 5 // 2

	merged_segments = []
	for yx_pt, max_indice, value in zip(yx, indices, topk_values):
		y, x = yx_pt
		if max_indice == -1 or value == 0:
			continue
		segment_list = []
		for y_offset in range(-basis, basis+1):
			for x_offset in range(-basis, basis+1):
				indice = idx_map[y+y_offset,x+x_offset]
				cnt = int(acc_map_np[y+y_offset,x+x_offset])
				if indice != -1:
					segment_list.append(segments[indice])
				if cnt > 1:
					check_cnt = 1
					current_hough = hough[indice]
					for new_indice, new_hough in enumerate(hough):
						if (current_hough == new_hough).all() and indice != new_indice:
							segment_list.append(segments[new_indice])
							check_cnt += 1
						if check_cnt == cnt:
							break
		group_segments = np.array(segment_list).reshape([-1, 2])
		sorted_group_segments = np.sort(group_segments, axis=0)
		x_min, y_min = sorted_group_segments[0,:]
		x_max, y_max = sorted_group_segments[-1,:]

		deg = theta[max_indice]
		if deg >= 90:
			merged_segments.append([x_min, y_max, x_max, y_min])
		else:
			merged_segments.append([x_min, y_min, x_max, y_max])

	# 2. get intersections
	new_segments = np.array(merged_segments) # (x1, y1, x2, y2)
	start = new_segments[:,:2] # (x1, y1)
	end = new_segments[:,2:] # (x2, y2)
	new_centers = (start + end) / 2.0
	diff = start - end
	dist_segments = np.sqrt(np.sum(diff ** 2, axis=-1))

	# ax + by = c
	a = diff[:,1]
	b = -diff[:,0]
	c = a * start[:,0] + b * start[:,1]
	pre_det = a[:,None] * b[None,:]
	det = pre_det - np.transpose(pre_det)

	pre_inter_y = a[:,None] * c[None,:]
	inter_y = (pre_inter_y - np.transpose(pre_inter_y)) / (det + 1e-10)
	pre_inter_x = c[:,None] * b[None,:]
	inter_x = (pre_inter_x - np.transpose(pre_inter_x)) / (det + 1e-10)
	inter_pts = np.concatenate([inter_x[:,:,None], inter_y[:,:,None]], axis=-1).astype('int32')
	
	# 3. get corner information
	# 3.1 get distance
	'''
	dist_segments:
		| dist(0), dist(1), dist(2), ...|
	dist_inter_to_segment1:
		| dist(inter,0), dist(inter,0), dist(inter,0), ... |
		| dist(inter,1), dist(inter,1), dist(inter,1), ... |
		...
	dist_inter_to_semgnet2:
		| dist(inter,0), dist(inter,1), dist(inter,2), ... |
		| dist(inter,0), dist(inter,1), dist(inter,2), ... |
		...
	'''

	dist_inter_to_segment1_start = np.sqrt(np.sum(((inter_pts - start[:,None,:]) ** 2), axis=-1, keepdims=True)) # [n_batch, n_batch, 1]
	dist_inter_to_segment1_end = np.sqrt(np.sum(((inter_pts - end[:,None,:]) ** 2), axis=-1, keepdims=True)) # [n_batch, n_batch, 1]
	dist_inter_to_segment2_start = np.sqrt(np.sum(((inter_pts - start[None,:,:]) ** 2), axis=-1, keepdims=True)) # [n_batch, n_batch, 1]
	dist_inter_to_segment2_end = np.sqrt(np.sum(((inter_pts - end[None,:,:]) ** 2), axis=-1, keepdims=True)) # [n_batch, n_batch, 1]
	
	# sort ascending
	dist_inter_to_segment1 = np.sort(np.concatenate([dist_inter_to_segment1_start, dist_inter_to_segment1_end], axis=-1), axis=-1) # [n_batch, n_batch, 2]
	dist_inter_to_segment2 = np.sort(np.concatenate([dist_inter_to_segment2_start, dist_inter_to_segment2_end], axis=-1), axis=-1) # [n_batch, n_batch, 2]

	# 3.2 get degree
	inter_to_start = new_centers[:,None,:] - inter_pts
	deg_inter_to_start = np.arctan2(inter_to_start[:,:,1], inter_to_start[:,:,0]) * 180 / np.pi
	deg_inter_to_start[deg_inter_to_start < 0.0] += 360
	inter_to_end = new_centers[None,:,:] - inter_pts
	deg_inter_to_end = np.arctan2(inter_to_end[:,:,1], inter_to_end[:,:,0]) * 180 / np.pi
	deg_inter_to_end[deg_inter_to_end < 0.0] += 360
	
	'''
	0 -- 1
	|    |
	3 -- 2
	'''
	# rename variables
	deg1_map, deg2_map = deg_inter_to_start, deg_inter_to_end
	# sort deg ascending
	deg_sort = np.sort(np.concatenate([deg1_map[:,:,None], deg2_map[:,:,None]], axis=-1), axis=-1)
	
	deg_diff_map = np.abs(deg1_map - deg2_map)
	# we only consider the smallest degree of intersect
	deg_diff_map[deg_diff_map > 180] = 360 - deg_diff_map[deg_diff_map > 180]
	
	# define available degree range
	deg_range = [60, 120]
	
	corner_dict = {corner_info: [] for corner_info in range(4)}
	inter_points = []
	for i in range(inter_pts.shape[0]):
		for j in range(i + 1, inter_pts.shape[1]):
			# i, j > line index, always i < j
			x, y = inter_pts[i, j, :]
			deg1, deg2 = deg_sort[i, j, :]
			deg_diff = deg_diff_map[i, j]
			
			check_degree = deg_diff > deg_range[0] and deg_diff < deg_range[1]

			outside_ratio = params['outside_ratio'] # over ratio >>> drop it!
			inside_ratio = params['inside_ratio'] # over ratio >>> drop it!
			check_distance = ((dist_inter_to_segment1[i,j,1] >= dist_segments[i] and \
								 dist_inter_to_segment1[i,j,0] <= dist_segments[i] * outside_ratio) or \
								(dist_inter_to_segment1[i,j,1] <= dist_segments[i] and \
								 dist_inter_to_segment1[i,j,0] <= dist_segments[i] * inside_ratio)) and \
							 ((dist_inter_to_segment2[i,j,1] >= dist_segments[j] and \
								 dist_inter_to_segment2[i,j,0] <= dist_segments[j] * outside_ratio) or \
								(dist_inter_to_segment2[i,j,1] <= dist_segments[j] and \
								 dist_inter_to_segment2[i,j,0] <= dist_segments[j] * inside_ratio))

			if check_degree and check_distance:
				corner_info = None

				if (deg1 >= 0 and deg1 <= 45 and deg2 >=45 and deg2 <= 120) or \
					 (deg2 >= 315 and deg1 >= 45 and deg1 <= 120):
					corner_info, color_info = 0, 'blue'
				elif (deg1 >= 45 and deg1 <= 125 and deg2 >= 125 and deg2 <= 225):
					corner_info, color_info = 1, 'green'
				elif (deg1 >= 125 and deg1 <= 225 and deg2 >= 225 and deg2 <= 315):
					corner_info, color_info = 2, 'black'
				elif (deg1 >= 0 and deg1 <= 45 and deg2 >= 225 and deg2 <= 315) or \
					 (deg2 >= 315 and deg1 >= 225 and deg1 <= 315):
					corner_info, color_info = 3, 'cyan'
				else:
					corner_info, color_info = 4, 'red' # we don't use it
					continue
				
				corner_dict[corner_info].append([x, y, i, j])
				inter_points.append([x, y])
	
	square_list = []
	connect_list = []
	segments_list = []
	for corner0 in corner_dict[0]:
		for corner1 in corner_dict[1]:
			connect01 = False
			for corner0_line in corner0[2:]:
				if corner0_line in corner1[2:]:
					connect01 = True
					break
			if connect01:
				for corner2 in corner_dict[2]:
					connect12 = False
					for corner1_line in corner1[2:]:
						if corner1_line in corner2[2:]:
							connect12 = True
							break
					if connect12:
						for corner3 in corner_dict[3]:
							connect23 = False
							for corner2_line in corner2[2:]:
								if corner2_line in corner3[2:]:
									connect23 = True
									break
							if connect23:
								for corner3_line in corner3[2:]:
									if corner3_line in corner0[2:]:
										# SQUARE!!!
										'''
										0 -- 1
										|    |
										3 -- 2
										square_list:
											order: 0 > 1 > 2 > 3
											| x0, y0, x1, y1, x2, y2, x3, y3 |
											| x0, y0, x1, y1, x2, y2, x3, y3 |
											...
										connect_list:
											order: 01 > 12 > 23 > 30
											| line_idx01, line_idx12, line_idx23, line_idx30 |
											| line_idx01, line_idx12, line_idx23, line_idx30 |
											...
										segments_list:
											order: 0 > 1 > 2 > 3
											| line_idx0_i, line_idx0_j, line_idx1_i, line_idx1_j, line_idx2_i, line_idx2_j, line_idx3_i, line_idx3_j |
											| line_idx0_i, line_idx0_j, line_idx1_i, line_idx1_j, line_idx2_i, line_idx2_j, line_idx3_i, line_idx3_j |
											...
										'''
										square_list.append(corner0[:2] + corner1[:2] + corner2[:2] + corner3[:2])
										connect_list.append([corner0_line, corner1_line, corner2_line, corner3_line])
										segments_list.append(corner0[2:] + corner1[2:] + corner2[2:] + corner3[2:])
	
	def check_outside_inside(segments_info, connect_idx):
		# return 'outside or inside', min distance, cover_param, peri_param
		if connect_idx == segments_info[0]:
			check_dist_mat = dist_inter_to_segment1
		else:
			check_dist_mat = dist_inter_to_segment2
		
		i, j = segments_info
		min_dist, max_dist = check_dist_mat[i, j, :]
		connect_dist = dist_segments[connect_idx]
		if max_dist > connect_dist:
			return 'outside', min_dist, 0, 1
		else:
			return 'inside', min_dist, -1, -1


	top_square = None

	try:
		map_size = input_shape[0] / 2
		squares = np.array(square_list).reshape([-1,4,2])
		score_array = []
		connect_array = np.array(connect_list)
		segments_array = np.array(segments_list).reshape([-1,4,2])
		
		# get degree of corners:
		squares_rollup = np.roll(squares, 1, axis=1)
		squares_rolldown = np.roll(squares, -1, axis=1)
		vec1 = squares_rollup - squares
		normalized_vec1 = vec1 / (np.linalg.norm(vec1, axis=-1, keepdims=True) + 1e-10)
		vec2 = squares_rolldown - squares
		normalized_vec2 = vec2 / (np.linalg.norm(vec2, axis=-1, keepdims=True) + 1e-10)
		inner_products = np.sum(normalized_vec1 * normalized_vec2, axis=-1) # [n_squares, 4]
		squares_degree = np.arccos(inner_products) * 180 / np.pi # [n_squares, 4]

		# get square score
		overlap_scores = []
		degree_scores = []
		length_scores = []
		
		for connects, segments, square, degree in zip(connect_array, segments_array, squares, squares_degree):
			'''
			0 -- 1
			|    |
			3 -- 2
			
			# segments: [4, 2]
			# connects: [4]
			'''
			
			###################################### OVERLAP SCORES
			cover = 0
			perimeter = 0
			# check 0 > 1 > 2 > 3
			square_length = []
			 
			for start_idx in range(4):
				end_idx = (start_idx + 1) % 4
				
				connect_idx = connects[start_idx] # segment idx of segment01
				start_segments = segments[start_idx]
				end_segments = segments[end_idx]
				
				start_point = square[start_idx]
				end_point = square[end_idx]
				
				# check whether outside or inside
				start_position, start_min, start_cover_param, start_peri_param = check_outside_inside(start_segments, connect_idx)
				end_position, end_min, end_cover_param, end_peri_param = check_outside_inside(end_segments, connect_idx)
				
				cover += dist_segments[connect_idx] + start_cover_param * start_min + end_cover_param * end_min
				perimeter += dist_segments[connect_idx] + start_peri_param * start_min + end_peri_param * end_min
				
				square_length.append(dist_segments[connect_idx] + start_peri_param * start_min + end_peri_param * end_min)
			
			overlap_scores.append(cover / perimeter)    
			######################################
			###################################### DEGREE SCORES
			'''
			deg0 vs deg2
			deg1 vs deg3
			'''
			deg0, deg1, deg2, deg3 = degree
			deg_ratio1 = deg0 / deg2
			if deg_ratio1 > 1.0:
				deg_ratio1 = 1 / deg_ratio1
			deg_ratio2 = deg1 / deg3
			if deg_ratio2 > 1.0:
				deg_ratio2 = 1 / deg_ratio2
			degree_scores.append((deg_ratio1 + deg_ratio2) / 2)
			######################################
			###################################### LENGTH SCORES
			'''
			len0 vs len2
			len1 vs len3
			'''
			len0, len1, len2, len3 = square_length
			len_ratio1 = len0 / len2 if len2 > len0 else len2 / len0
			len_ratio2 = len1 / len3 if len3 > len1 else len3 / len1
			length_scores.append((len_ratio1 + len_ratio2) / 2)

			######################################
		
		overlap_scores = np.array(overlap_scores)
		overlap_scores /= np.max(overlap_scores)
			
		degree_scores = np.array(degree_scores)
		#degree_scores /= np.max(degree_scores)
		
		length_scores = np.array(length_scores)

		###################################### AREA SCORES
		area_scores = np.reshape(squares, [-1, 4, 2])
		area_x = area_scores[:,:,0]
		area_y = area_scores[:,:,1]
		correction = area_x[:,-1] * area_y[:,0] - area_y[:,-1] * area_x[:,0]
		area_scores = np.sum(area_x[:,:-1] * area_y[:,1:], axis=-1) - np.sum(area_y[:,:-1] * area_x[:,1:], axis=-1)
		area_scores = 0.5 * np.abs(area_scores + correction)
		area_scores /= (map_size * map_size) #np.max(area_scores)
		######################################
		
		###################################### CENTER SCORES
		centers = np.array([[256 // 2, 256 // 2]], dtype='float32') # [1, 2]
		# squares: [n, 4, 2]
		square_centers = np.mean(squares, axis=1) # [n, 2]
		center2center = np.sqrt(np.sum((centers - square_centers) ** 2, axis=1))
		center_scores = center2center / (map_size / np.sqrt(2.0))


		'''
		score_w = [overlap, degree, area, center, length]
		'''
		score_w = [0.0, 1.0, 10.0, 0.5, 1.0]
		score_array = params['w_overlap'] * overlap_scores \
						+ params['w_degree'] * degree_scores \
						+ params['w_area'] * area_scores \
						- params['w_center'] * center_scores \
						+ params['w_length'] * length_scores

		best_square = []

		sorted_idx = np.argsort(score_array)[::-1]
		score_array = score_array[sorted_idx]
		squares = squares[sorted_idx]

	except Exception as e:
		pass
	
	try:
		new_segments[:,0] = new_segments[:,0] * 2 / input_shape[1] * original_shape[1]
		new_segments[:,1] = new_segments[:,1] * 2 / input_shape[0] * original_shape[0]
		new_segments[:,2] = new_segments[:,2] * 2 / input_shape[1] * original_shape[1]
		new_segments[:,3] = new_segments[:,3] * 2 / input_shape[0] * original_shape[0]
	except:
		new_segments = []
	
	try:
		squares[:,:,0] = squares[:,:,0] * 2 / input_shape[1] * original_shape[1]
		squares[:,:,1] = squares[:,:,1] * 2 / input_shape[0] * original_shape[0]
	except:
		squares = []
		score_array = []

	try:
		inter_points = np.array(inter_points)
		inter_points[:,0] = inter_points[:,0] * 2 / input_shape[1] * original_shape[1]
		inter_points[:,1] = inter_points[:,1] * 2 / input_shape[0] * original_shape[0]
	except:
		inter_points = []
		

	return new_segments, squares, score_array, inter_points
