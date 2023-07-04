"""
		Used for performance profiling and recording, try 'python benchmark.py --help' for more info.
"""

from time import time
import subprocess, argparse
from src import Vision, cv2

def do_test(filepath: str) -> float:
		"""
		Takes a path to a video file, runs it through src.Vision() and returns the average FPS for the entire process (includes initialisation).
		"""
		t0 = time()

		v = Vision()
		v.reload_configuration(camera_settings={"capture_code": filepath})
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
				v.current_steering()
				v.finish_line_detection()

		t1 = time()

		return v.capture.get(cv2.CAP_PROP_FRAME_COUNT) / (t1 - t0)


if __name__ == "__main__":
		parser = argparse.ArgumentParser()
		parser.add_argument("-f", "--videofile", required=True, help="Path of the video file used for benchmarking")
		parser.add_argument("-m", "--message", default="", help="Custom message which is added to benchmark.txt for documentation")
		parser.add_argument("-c", "--count", default=10, help="The number of benchmarking runs to attempt")
		args = parser.parse_args()

		#tree_hash = (
				#subprocess.check_output(["git", "rev-parse", "--short", "HEAD"])
				#.decode("utf8")
				#.split("\n")[0]
		#)

		results = []
		for i in range(int(args.count)):
				print(f"Test {i + 1}: ", end="")
				r = do_test(args.videofile)
				results.append(r)
				print(r, "fps")

		avg = round(sum(results) / len(results), 5)

		#formatted_str = f"[{tree_hash}] attempts={args.count} average={avg} {'# ' + args.message if args.message else ''}"
		formatted_str = f"attempts={args.count} average={avg} {'# ' + args.message if args.message else ''}"

		print(formatted_str)
		open("./testing/benchmark.txt", "a").write("\n" + formatted_str)
