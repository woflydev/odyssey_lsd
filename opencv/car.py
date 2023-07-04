"""
		Used for performance profiling and recording, try 'python benchmark.py --help' for more info.
"""

from time import time
import subprocess, argparse
from src import Vision, cv2

if __name__ == "__main__":

		v = Vision()
		v.reload_configuration()
		v.calibrate_sideline_colours()

		while v.next_frame():
				v.set_masks()
				v.find_contours()
				v.find_bounds()
				v.generate_path_blocks()
				v.find_path()
				v.optimise_path()
				v.add_turn_destination(v.chosen_path[0][0])
				v.boost_value()
				
				value = v.current_steering()
				print(value)
				
				v.finish_line_detection()
