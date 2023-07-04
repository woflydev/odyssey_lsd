from __future__ import annotations
import cv2
import numpy as np
from json import loads, dumps
from typing import Dict, List, Tuple, Union


from .profiling import time_this


def merge_dicts(d1: Dict, d2: Dict) -> Dict:
		"""
		Custom function for deep-merging config files.
		"""
		for key in d2:
				if not d1.get(key, False):
						d1[key] = d2[key]
				elif isinstance(d2[key], dict) and isinstance(d1[key], dict):
						merge_dicts(d1[key], d2[key])
				else:
						d1[key] = d2[key]
		return d1


class Vision(object):

		def __init__(self, debug: bool=False):
				"""
				This is a monolithic class designed specifically for the QUT Droid Racing Challenge of 2022. It attempts the following tasks;-
						- Loads and merges configuration settings from files and runtime arguments
						- Connects to and reads from capture devices & video files
						- Automatically detects the colours for left and right sidelines during start-up
						- Extracts obstacles and track indicators from still frames
						- Determines paths between said obstacles and track indicators
						- Parses path information into raw steering values
						- Establish when the finish line has been crossed
				"""
				self.debug = debug
				self.settings = {}
				self.info = {}
				self.frame_count = 0

				self.masks = {"left_line": [], "right_line": [], "obstacle": [], "car": []}
				self.finish_line_mask = []
				self.finish_line_contours = []

				# Obstacles and cars get rolled into 1 contour category: "obstruction"
				self.contours = {"left_line": [], "right_line": [], "obstruction": []}

				self.obstacles = []

				self.block_graph = ()


		@staticmethod
		def start(debug: bool=False, **kargs) -> Vision:
				"""
				Quick-start method which returns a fully-configured Vision object. Also accepts debug mode and configuration settings.
				"""
				v = Vision(debug)
				v.reload_configuration(**kargs)
				v.calibrate_sideline_colours()

				return v


		@time_this
		def next_frame(self) -> bool:
				"""
				Attempts to read a frame from the current capture device into **self.frame**.

				Returns **True** on success and **False** on failure.

				Throws if no capture device is configured.
				"""

				_, self.frame = self.capture.read()

				if self.frame is None:
						return False

				self.frame_count += 1

				# Apply RBG->HSV conversion and blurring
				cv2.medianBlur(cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV), 3, self.frame)

				return True


		@time_this
		def reload_configuration(self, **config_options) -> None:
				"""
				Merges multiple configuration settings, in the following priority;-
						1. **config_options**
						2. "./config.json"
						3. "./.default_config.json"
				
				Also sets/resets **self.capture** and **self.info**, allocates memory for frame masks.
				Throws if a frame can't be successfully read.
				"""

				# lowest priority settings
				with open("./.default_config.json", "r") as defaults:
						self.settings = loads(defaults.read())

				# Mid priority
				try:
						with open("./config.json", "r") as user_settings:
								as_dict = loads(user_settings.read())
								merge_dicts(self.settings, as_dict)
				except FileExistsError:
						pass

				# Highest priority
				if len(config_options) != 0:
						merge_dicts(self.settings, config_options)

				# OpenCV only accepts tuples or numpy arrays as colour range arguments in cv2.inRange(), so lists must be converted
				for key in self.settings["colour_ranges"].keys():
						self.settings["colour_ranges"][key] = tuple(
								(np.array(arr) for arr in self.settings["colour_ranges"][key])
						)

				camera_settings = self.settings["camera_settings"]

				# Determine whether the capture_code references a device or video file
				capture_type = (
						"live" if type(camera_settings["capture_code"]) == int else "playback"
				)

				# Windows has the special "DirectShow" API for live video streaming.
				# If the D_SHOW API is not specified, OpenCV may take a while to load the source.
				if self.settings.get("use_windows") and capture_type == "live":
						self.capture = cv2.VideoCapture(
								camera_settings["capture_code"], cv2.CAP_DSHOW
						)
				else:
						self.capture = cv2.VideoCapture(camera_settings["capture_code"])

				if capture_type == "live":
						# Width, height and FPS
						self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera_settings["width"])
						self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_settings["height"])
						self.capture.set(cv2.CAP_PROP_FPS, camera_settings["fps"])
						# The codec used to transfer frames from the camera to OpenCV
						self.capture.set(
								cv2.CAP_PROP_FOURCC,
								cv2.VideoWriter_fourcc(*(camera_settings["codec"])),
						)
						# Make sure we're only buffering 1 frame
						self.capture.set(cv2.CAP_PROP_BUFFERSIZE, camera_settings["buffer"])
						# Manually set focus to disable autofocus
						self.capture.set(cv2.CAP_PROP_FOCUS, camera_settings["focus"])

				# Ensure frames can be read
				if self.next_frame() is False:
						raise Exception(
								f"Could not load frame from video source with capture code '{camera_settings['capture_code']}'"
						)

				height, width, _ = self.frame.shape

				self.info = {
						"height": height,
						"width": width,
						"capture_type": capture_type,
						"line_order": ("blue_line", "yellow_line"),
						"centre": width // 2,
						"horizon_line": int(self.settings["pathfinding"]["horizon_line"] * height),
						"minimum_path_width": int(
								self.settings["pathfinding"]["minimum_path_width"] * height
						),
						"steering_span": self.settings["serial"]["maximum_steering_value"] - self.settings["serial"]["minimum_steering_value"],
						"neutral_steering_position": (
								(
										self.settings["serial"]["maximum_steering_value"]
										+ self.settings["serial"]["minimum_steering_value"]
								) // 2
						),
						"detected_finish_line": False,
						"finish_line_hits": [],
						"track_complete": False
				}

				# Populate the turning/steering buffer with centre values
				self.info["turning_buffer"] = [
						self.info["neutral_steering_position"]
				] * self.settings["serial"]["steering_buffer_size"]

				# Calculate the pixel size of each horizontal slice
				slicing_interval = int(
						height * self.settings["pathfinding"]["slicing_interval"]
				)

				# Sets the boundaries of each slicing range
				self.info["vertical_slicing_pairs"] = [(height - 1, height - slicing_interval)]
				while self.info["vertical_slicing_pairs"][-1][1] >= self.info["horizon_line"]:
						last_val = self.info["vertical_slicing_pairs"][-1][1]
						self.info["vertical_slicing_pairs"].append(
								(last_val - 1, last_val - slicing_interval)
						)
				self.info["vertical_slicing_pairs"].pop()

				# Initialise mask objects in memory with correct dimensions
				for key in self.masks:
						self.masks[key] = np.zeros(  # type: ignore
								(height, width),
								dtype=np.uint8,
						)


		@time_this
		def calibrate_sideline_colours(self) -> None:
				"""
				Attempts to determine which sideline colour will be on the left and which one will be on the right.
				This is done by comparing the bottom-left pixel positions for both colours.

				NOTE: Currently hard-coded to have yellow on the left and blue on the right.
				"""

				# Short circuit due to known line order for competition
				self.info["line_order"] = ("yellow_line", "blue_line")
				return

				colour_ranges = [
						tuple(self.settings["colour_ranges"][colour])
						for colour in self.info["line_order"]
				]

				masks = [cv2.inRange(self.frame, *colour) for colour in colour_ranges]

				# Extract the positions of every coloured pixel
				nonzeroes = [np.nonzero(mask) for mask in masks]

				try:
						# Records the x-positions of the bottom left
						lowest_xs = [nonzero[1][np.argmax(nonzero[0])] for nonzero in nonzeroes]
				except ValueError:
						# Throws if no coloured pixels are found
						raise ValueError("Could not find sideline pixels for line calibration.")

				# If the current line order is wrong, reverse it
				if lowest_xs[0] > lowest_xs[1]:
						self.info["line_order"] = (
								self.info["line_order"][1],
								self.info["line_order"][0],
						)


		@time_this
		def set_masks(self) -> None:
				"""
				Extracts colour masks from **self.frame** and stores them in **self.masks**.
				"""

				left_line_ranges = self.settings["colour_ranges"][self.info["line_order"][0]]
				right_line_ranges = self.settings["colour_ranges"][self.info["line_order"][1]]

				self.masks["left_line"] = cv2.inRange(  # type: ignore
						self.frame,
						left_line_ranges[0],
						left_line_ranges[1],
				)
				
				cv2.imshow("left_line", self.masks["left_line"])

				self.masks["right_line"] = cv2.inRange(  # type: ignore
						self.frame,
						right_line_ranges[0],
						right_line_ranges[1],
				)

				cv2.imshow("right_line", self.masks["right_line"])

				self.masks["obstacle"] = cv2.inRange(  # type: ignore
						self.frame,
						self.settings["colour_ranges"]["obstacle"][0],
						self.settings["colour_ranges"]["obstacle"][1],
				)
				
				cv2.imshow("obstacle", self.masks["obstacle"])
				
				self.masks["car"] = cv2.inRange(  # type: ignore
						self.frame,
						self.settings["colour_ranges"]["car"][0],
						self.settings["colour_ranges"]["car"][1],
				)

				cv2.imshow("car", self.masks["car"])
				
				cv2.waitKey(1)


		@time_this
		def find_contours(self) -> None:
				"""
				Determines the contours (i.e. edges) of values in **self.masks** using **cv2.findContours** with RETR_EXTERNAL and CHAIN_APPROX_NONE flags.
				Contours with an area less than **self.settings["pathfinding"]["minimum_contour_area"]** are excluded.

				NOTE: Obstacle and car masks are combined into the "obstruction" category
				"""
				min_area = self.settings["pathfinding"]["minimum_contour_area"]

				self.contours["left_line"] = [
						contour
						for contour in cv2.findContours(
								self.masks["left_line"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE  # type: ignore
						)[0]
						if cv2.contourArea(contour) >= min_area
				]
				
				self.contours["right_line"] = [
						contour
						for contour in cv2.findContours(
								self.masks["right_line"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE  # type: ignore
						)[0]
						if cv2.contourArea(contour) >= min_area
				]

				self.contours["obstruction"] = [
						contour
						for contour in cv2.findContours(
								self.masks["obstacle"] | self.masks["car"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE  # type: ignore
						)[0]
						if cv2.contourArea(contour) >= min_area
				]


		@time_this
		def find_bounds(self) -> List[List[Tuple[int, int, List[str]]]]:
				"""
				Runs through each vertical range in **self.info["vertical_slicing_pairs"]**, extracting the x-coordinates of every contour that falls into said range.
				The returned value (which is also placed into **self.obstacles**) is the same length as **self.info["vertical_slicing_pairs"]**.
				Entries in each row are in the format **(leftmost-x-coordinate, rightmost-x-coordinate, [obstacle_type])** where **obstacle_type** corresponds to a key in **self.contours**.

				NOTE: **self.obstacles** can be zipped together with **self.info["vertical_slicing_pairs"]** to find both the x and y bounds of each obstacle slice.
				"""

				vertical_bounding_pairs = self.info["vertical_slicing_pairs"]
				obstacles = [[] for _ in vertical_bounding_pairs]
				horizon_line = self.info["horizon_line"]

				for contour_type in self.contours.keys():
						# Reshape OpenCV array for easier parsing
						contours = [
								np.reshape(contour, (len(contour), 2))
								for contour in self.contours[contour_type]
						]

						for contour in contours:
								# For every slicing range
								for i, (lower_bound, upper_bound) in enumerate(vertical_bounding_pairs):
										if lower_bound < horizon_line:
												continue # Skip slicing ranges which fall above the horizon line

										# Use numpy to find the points in the contour that fall in the slicing range
										val = contour[
												np.where(
														np.logical_and(
																contour[:, 1] <= lower_bound,
																contour[:, 1] >= upper_bound,
														)
												)
										]

										if len(val) > 0:
												# Extract the bounding box for matching points
												start_x, _, width, _ = cv2.boundingRect(val)

												# Add the horizontal values to the corresponding vertical index of obstacles
												obstacles[i].append((start_x, start_x + width, [contour_type]))

				self.obstacles = obstacles
				return obstacles


		@time_this
		def generate_path_blocks(self) -> List[List[Tuple[int, int, Tuple[List[str], List[str]]]]]:
				"""
				This function performs multiple actions to transform **self.obstacles** into **self.gaps**;-
						1.) Sorts obstacles from left to right.
						2.) Merges obstacles that border each other.
						3.) Creates a list of gaps between adjacent objects, excluding gaps that are less than **self.info["minimum_path_width"]** pixels wide.
				
				Each gap is in the format **(leftmost-position, rightmost-position, (left_obstacle_types, right_obstacle_types))**.
				
				The result is both returned and stored in **self.gaps** with the following forma

				NOTE: **self.gaps** can be zipped together with **self.info["vertical_slicing_pairs"]** to find both the x and y boundaries of each gap.
				"""

				minimum_path_width = self.info["minimum_path_width"]
				frame_width = self.info["width"]
				gaps = []

				for obstacles in self.obstacles:

						# If no obstacles are found in a row, create a path that's the width of the frame and continue
						if len(obstacles) == 0:
								gaps.append(
										[(0, frame_width, (["left_border"], ["right_border"]))]
								)
								continue

						# NOTE: Lower_bound > upper_bound because y=0 is top of frame

						# Sort obstacles from left to right
						sorted_obstacles = sorted(obstacles, key=lambda bounds: bounds[0])

						# Add screen edges to the row of obstacles
						sorted_obstacles.insert(0, (0, 0, ["left_border"]))
						sorted_obstacles.append((frame_width, frame_width, ["right_border"]))

						# Merge overlapping obstacles, from left to right
						i = 0
						while i < len(sorted_obstacles):
								inner_obstacle = sorted_obstacles[i]
								n = i + 1
								while n < len(sorted_obstacles):
										if inner_obstacle[0] <= sorted_obstacles[n][0] <= inner_obstacle[1]:
												removed_obstacle = sorted_obstacles.pop(n)
												sorted_obstacles[i] = (
														inner_obstacle[0],
														max(removed_obstacle[1], inner_obstacle[1]),
														[*inner_obstacle[2], *removed_obstacle[2]],
												)

												inner_obstacle = sorted_obstacles[i]
										else:
												n += 1
								i += 1

						# Create the boundaries of the gap
						paths = []
						for i in range(len(sorted_obstacles) - 1):
								_, left_object_end, left_types = sorted_obstacles[i]
								right_object_start, _, right_types = sorted_obstacles[i + 1]

								if right_object_start - left_object_end >= minimum_path_width:
										paths.append((left_object_end, right_object_start, (left_types, right_types)))

						gaps.append(paths)

				self.gaps = gaps
				return gaps


		@time_this
		def find_path(self) -> Tuple[
				List[Tuple[int, int]],
				List[Tuple[int, int, Tuple[List[str], List[str]]]]
		]:
				"""
				Applies pathing logic to turn **self.gaps** into a list of gaps that form a viable path forward.
				The returned value is a list of points and a list of corresponding reference gaps. This value is both stored in **self.path** and returned.
				"""

				vertical_slice_ranges = self.info["vertical_slicing_pairs"]
				height, width, centre = (
						self.info["height"],
						self.info["width"],
						self.info["centre"],
				)

				points = []
				reference_blocks = []

				last_point, last_block = (centre, height), (-1, centre * 2 + 1, ([], []))

				for i, (row, (bottom, top)) in enumerate(
						zip(self.gaps, vertical_slice_ranges)
				):
						# Exclude "illegal" paths (i.e. gaps between the left border and left line) and paths that aren't lined up with the previous path
						passable_row = [
								block
								for block in row
								if not (
										("left_border" in block[2][0] and "left_line" in block[2][1])
										or ("right_border" in block[2][1] and "right_line" in block[2][0])
										or ("left_border" in block[2][0] and "left_border" in block[2][1])
										or ("right_border" in block[2][0] and "right_border" in block[2][1])
								)
								and (
										last_block[0] <= block[0] <= last_block[1]
										or block[0] <= last_block[0] <= block[1]
								)
						]

						# If no valid blocks are possible for the next path segment, try pathing to the left or right of frame
						# NOTE: This implementation is janky and possibly broken. I do not plan on fixing this
						if len(passable_row) == 0:
								left_border_exit = (0, last_point[1])
								right_border_exit = (width - 1, last_point[1])

								if i == 0:
										centremost_block = sorted(
												[block for block in row],
												key=lambda b: abs((b[0] + b[1]) // 2 - centre),
										)[0]

										if (
												"left_line" in centremost_block[2][0]
												and "left_line" in centremost_block[2][1]
										):
												points.append(right_border_exit)
										elif (
												"right_line" in centremost_block[2][0]
												and "right_line" in centremost_block[2][1]
										):
												points.append(left_border_exit)
										elif "left_line" in centremost_block[2][1]:
												points.append(right_border_exit)
										elif "right_line" in centremost_block[2][0]:
												points.append(left_border_exit)
										else:
												points.append(left_border_exit)

										break

								condition_bools = (
										"left_border" in last_block[2][0],
										"right_border" in last_block[2][1],
								)

								if condition_bools[0] and condition_bools[1]:
										if last_point[0] < centre:
												points.append(left_border_exit)
										else:
												points.append(right_border_exit)
								elif (
										"left_line" in last_block[2][0] and "left_line" in last_block[2][1]
								):
										points.append(right_border_exit)
								elif (
										"right_line" in last_block[2][0]
										and "right_line" in last_block[2][1]
								):
										points.append(right_border_exit)

								elif condition_bools[0]:
										points.append(left_border_exit)
								elif condition_bools[1]:
										points.append(right_border_exit)

								break

						row_height = (bottom + top) // 2

						# Find the next path segment that is closest to the previous one
						closest, displacement = passable_row[0], abs(
								(passable_row[0][0] + passable_row[0][1]) // 2 - last_point[0]
						)

						for block in passable_row[1:]:
								current_displacement = abs((block[0] + block[1]) // 2 - last_point[0])
								if current_displacement < displacement:
										closest = block
										displacement = current_displacement

						last_point = ((closest[0] + closest[1]) // 2, row_height)
						last_block = closest
						reference_blocks.append(closest)
						points.append(last_point)

				self.path = (points, reference_blocks)
				return points, reference_blocks

		@time_this
		def optimise_path(self):
				"""
				Attempts to "optimise" **self.path** by moving all path points closer to the centre of the frame.
				Maintains a minimum of **self.info["minimum_path_width"] // 2** pixels between points and bordering obstacles.
				Returns a list of coordinates and stores them in **self.chosen_path**.
				"""

				offset = self.info["minimum_path_width"] // 2
				points, reference_blocks = list(reversed(self.path[0])), list(
						reversed(self.path[1])
				)

				last_target = points[0]
				optimised_points = [last_target]

				for point, ref_block in zip(points[1:], reference_blocks[1:]):
						# we assume the point is centred in its block
						if point[0] == last_target[0]:
								optimised_points.append(point)
						elif last_target[0] < point[0]:
								if ref_block[0] + offset < last_target[0]:
										optimised_points.append((last_target[0], point[1]))
								else:
										optimised_points.append((ref_block[0] + offset, point[1]))
						else:
								if ref_block[1] - offset > last_target[0]:
										optimised_points.append((last_target[0], point[1]))
								else:
										optimised_points.append((ref_block[1] - offset, point[1]))
						last_target = optimised_points[-1]

				self.chosen_path = list(reversed(optimised_points))

				return self.chosen_path

		@time_this
		def finish_line_detection(self):
				"""
						Attempts to detect whether the finish line has been crossed.
						This is achieved by counting the number of times a relevant contour above a certain area is seen.
				"""

				self.finish_line_mask = cv2.inRange(  # type: ignore
						self.frame,
						self.settings["colour_ranges"]["finish_line"][0],
						self.settings["colour_ranges"]["finish_line"][1],
				)

				self.finish_line_contours = [
						(contour, cv2.contourArea(contour))
						for contour in cv2.findContours(
								self.finish_line_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE  # type: ignore
						)[0]
						if cv2.contourArea(contour)
						>= self.settings["finish_line"]["minimum_contour_area"]
				]

				if len(self.finish_line_contours) == 0:

						if (
								len(self.info["finish_line_hits"])
								< self.settings["finish_line"]["minimum_hits"]
						):
								self.info["detected_finish_line"] = False
								del self.info["finish_line_hits"][:]
						else:
								self.info["track_complete"] = True

						return self.info["track_complete"]

				largest_contour, area = self.finish_line_contours[0]
				for comparison_contour, comparison_area in self.finish_line_contours[1:]:
						if comparison_area > area:
								largest_contour, area = comparison_contour, comparison_area

				_, start_y, _, _ = cv2.boundingRect(largest_contour)

				self.info["detected_finish_line"] = True

				if len(self.info["finish_line_hits"]) == 0:
						self.info["finish_line_hits"].append(start_y)
						return False

				if (
						self.info["finish_line_hits"][-1]
						> start_y + self.settings["finish_line"]["minimum_pixel_offset"]
				):
						self.info["detected_finish_line"] = False
						del self.info["finish_line_hits"][:]
						return False

				self.info["finish_line_hits"].append(start_y)
				return False

		@time_this
		def turning_certainty(self) -> float:
				"""
				Determines how far **self.chosen_path** predicts the device will need to turn.
				Returns a float between zero and one, where zero represents no turning and one represents turning off the frame.
				"""
				return (
						max([abs(path[0] - self.info["centre"]) for path in self.chosen_path])
						/ self.info["centre"]
				)

		@time_this
		def boost_value(self) -> float:
				"""
				Calculates a recommended acceleration amount between 0.05 and 1, based on **self.turning_certainty()**.
				"""
				turning_value = self.turning_certainty()
				return -(0.95 * turning_value * turning_value) + 1

		@time_this
		def current_steering(self) -> int:
				"""
				Calculates the current steering value by adjusting it to steering ranges (from **self.settings**) and averaging buffered steering values.
				"""
				
				value = round(
						((np.mean(self.info["turning_buffer"])) / self.info["width"])
						* self.info["steering_span"]
						+ self.settings["serial"]["minimum_steering_value"]
				)
				
				return value

		@time_this
		def add_turn_destination(self, x_pos: int):
				"""
				Pushes a pixel offset value into the turning buffer.
				Calling **self.current_steering()** will then return adjusted steering values.
				"""
				self.info["turning_buffer"].append(x_pos)
				if (
						len(self.info["turning_buffer"])
						> self.settings["serial"]["steering_buffer_size"]
				):
						self.info["turning_buffer"].pop(0)

				return self.current_steering()

		def __del__(self):
				# Release capture device when the object is cleaned up
				if hasattr(self, "capture"):
						self.capture.release()

		def __str__(self):
				return f"Vision<INFO={dumps(self.info)}, CONFIG={dumps(self.settings)}>"
