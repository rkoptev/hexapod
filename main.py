from subprocess import call
import serial
import time
import hexapod

serialport = serial.Serial("/dev/ttyS0", 115200, timeout=0.5)

robot = hexapod.Hexapod()

state = "uninitialized"
relax_after_mooves = False

while True:
	char = serialport.read()
	if char == 'F':
		print "Forward"
		robot.go_forward()
		serialport.reset_input_buffer()
		state = "go_forward"
	elif char == "B":
		print "Backward"
		robot.go_backward()
		serialport.reset_input_buffer()
		state = "go_backward"
	elif char == "L":
		print "Left"
		robot.turn_left()
		serialport.reset_input_buffer()
		state = "rotate_left"
	elif char == "R":
		print "Right"
		robot.turn_right()
		serialport.reset_input_buffer()
		state = "rotate_right"
	elif char == "S":
		if state is not "stop":
			if relax_after_mooves:
				robot.relax()
			else:
				robot.stop()
			state = "stop"
	elif char == "x":
		relax_after_mooves = True
		state = ""
	elif char == "X":
		relax_after_mooves = False
		state = ""
	elif char == "V" or char == "v":
		call("sudo shutdown -h now", shell=True)
		exit()
	elif char == "1" or char == "2" or char == "3" or char == "4" or char == "5" or char == "6" or char == "7" or char == "8" or char == "9" or char == "q":
		if char == "q":
			char = "10"
		speed = int(char)
		# TODO fix not all range:
		coefficient = (1 / (speed/10.0) / 10.0) - 0.1
		step_time = hexapod.MIN_STEP_TIME + coefficient *(hexapod.MAX_STEP_TIME - hexapod.MIN_STEP_TIME)
		robot.set_step_time(step_time)
		print "Step time: " + str(step_time)


	else:
		print char
