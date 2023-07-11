from utils.motor_lib.driver import off, move, brake
import time

def victory():
	#Spin
  for i in range(4):
    move(-60, 60, 500)
    move(60, -60, 500)
    move(-60, -60, 500)
    move(60, 60, 500)
  off()

def wheelie():
  move(60, 60, 500)
  move(-60, -60, 500)
  move(60, 60, 600)
  move(70, 70, 100)
  move(80, 80, 250)
  move(90, 90, 300)

wheelie()
off()