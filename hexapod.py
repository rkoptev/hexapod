import Adafruit_PCA9685
import time

'''
0-5: coxa joints
8-13: tibia joints
Order is the following: front-left, left, back-left, front-right, right, back-right
'''

SERVO_PWM_FREQUENCY = 60

COXA_MIN_DELTA = 10
COXA_MAX_DELTA = 90
COXA_DEFAULT_DELTA = 70

TIBIA_MIN_DELTA = 10
TIBIA_MAX_DELTA = 90
TIBIA_DEFAULT_DELTA = 70

MIN_STEP_TIME = 0.1
MAX_STEP_TIME = 2.0
DEFAULT_STEP_TIME = MIN_STEP_TIME

servos_configuration = {
	"0": { "min": 190, "max": 500, "mid":350, "group":"coxa", "inverted": False},
	"1": { "min": 240, "max": 410, "mid":350, "group":"coxa", "inverted": False},
	"2": { "min": 180, "max": 540, "mid":350, "group":"coxa", "inverted": False},
	"3": { "min": 200, "max": 640, "mid":350, "group":"coxa", "inverted": True},
	"4": { "min": 290, "max": 460, "mid":350, "group":"coxa", "inverted": True},
	"5": { "min": 170, "max": 580, "mid":350, "group":"coxa", "inverted": True},
	"8": { "min": 240, "max": 430, "mid":400, "group":"tibia", "inverted": False},
	"9": { "min": 240, "max": 480, "mid":400, "group":"tibia", "inverted": False},
	"10": { "min":260, "max": 490, "mid":400, "group":"tibia", "inverted": False},
	"11": { "min":310, "max": 510, "mid":400, "group":"tibia", "inverted": True},
	"12": { "min":300, "max": 500, "mid":400, "group":"tibia", "inverted": True},
	"13": { "min":310, "max": 510, "mid":400, "group":"tibia", "inverted": True}
}

coxa_group = ["0", "1", "2", "3", "4", "5"]
coxa_a_group = ["0", "2", "4"]
coxa_b_group = ["1", "3", "5"]
coxa_left_group = ["0", "1", "2"]
coxa_right_group = ["3", "4", "5"]

tibia_group = ["8", "9", "10", "11", "12", "13"]
tibia_a_group = ["8", "10", "12"]
tibia_b_group = ["9", "11", "13"]
tibia_left_group = ["8", "9", "10"]
tibia_right_group = ["11", "12", "13"]

coxa_default_position = 350
tibia_default_position = 400

class Hexapod():
	def __init__(self):
		# Initialise the PCA9685 servo drivers
		self.__coxa_driver = Adafruit_PCA9685.PCA9685()
		self.__tibia_driver = Adafruit_PCA9685.PCA9685(0x41)

		# Set frequency to 60hz, good for servos
		self.__coxa_driver.set_pwm_freq(SERVO_PWM_FREQUENCY)
		self.__tibia_driver.set_pwm_freq(SERVO_PWM_FREQUENCY)

		# Initialize default movement parameters
		self.coxa_delta = COXA_DEFAULT_DELTA
		self.tibia_delta = TIBIA_DEFAULT_DELTA
		self.step_time = DEFAULT_STEP_TIME

		# Initialize joints positions
		self.joints_positions = {}
		for channel_index, channel in servos_configuration.iteritems():
			if channel["group"] == "coxa":
				self.joints_positions[channel_index] = coxa_default_position
			elif channel["group"] == "tibia":
				self.joints_positions[channel_index] = tibia_default_position

	# Example: __write("4", 370)
	def __write(self, channel, value, consider_inversion):
		if consider_inversion and servos_configuration[channel]["inverted"]:
			delta = value - servos_configuration[channel]["mid"]
			value -= delta * 2
		group = servos_configuration[channel]["group"]
		if group == "coxa":
			self.__coxa_driver.set_pwm(int(channel), 0, int(value))
		elif group == "tibia":
			self.__tibia_driver.set_pwm(int(channel), 0, int(value))


	def write_group(self, group, destination, move_time, consider_inversion=True):
		start_time = time.time()
		while True:
			time_passed = time.time() - start_time
			k = time_passed / move_time
			if k < 1:
				# Process move for each channel
				for channel in group:
					position = self.joints_positions[channel] + k * (destination - self.joints_positions[channel])
					self.__write(channel, position, consider_inversion)
			else:
				# Go to final position, remember those positions and return
				for channel in group:
					self.__write(channel, destination, consider_inversion)
					self.joints_positions[channel] = destination
				return

	def set_step_time(self, step_time):
		if MIN_STEP_TIME < step_time < MAX_STEP_TIME:
			self.step_time = step_time
		else:
			print "Wrong step time: " + str(step_time)

	def go_forward(self):
		# Stay on A group
		self.write_group(tibia_b_group, tibia_default_position + self.tibia_delta, self.step_time)
		self.write_group(tibia_a_group, tibia_default_position - self.tibia_delta, self.step_time)

		# Move B group forward, A group backward
		# Fastly move free joints
		self.write_group(coxa_a_group, coxa_default_position - self.coxa_delta, 0.01)
		# And slowly move joints on which robot stay
		self.write_group(coxa_b_group, coxa_default_position + self.coxa_delta, self.step_time)

		# Stay on B group
		self.write_group(tibia_a_group, tibia_default_position + self.tibia_delta, self.step_time)
		self.write_group(tibia_b_group, tibia_default_position - self.tibia_delta, self.step_time)

		# Move A group forward, B group backward
		# Fastly move free joints
		self.write_group(coxa_b_group, coxa_default_position - self.coxa_delta, 0.01)
		# And slowly move joints on which robot stay
		self.write_group(coxa_a_group, coxa_default_position + self.coxa_delta, self.step_time)

	def go_backward(self):
		# Stay on A group
		self.write_group(tibia_b_group, tibia_default_position + self.tibia_delta, self.step_time)
		self.write_group(tibia_a_group, tibia_default_position - self.tibia_delta, self.step_time)

		# Move A group forward, B group backward
		# Fastly move free joints
		self.write_group(coxa_b_group, coxa_default_position - self.coxa_delta, self.step_time)
		# And slowly move joints on which robot stay
		self.write_group(coxa_a_group, coxa_default_position + self.coxa_delta, 0.01)

		# Stay on B group
		self.write_group(tibia_a_group, tibia_default_position + self.tibia_delta, self.step_time)
		self.write_group(tibia_b_group, tibia_default_position - self.tibia_delta, self.step_time)

		# Move B group forward, A group backward
		# Fastly move free joints
		self.write_group(coxa_a_group, coxa_default_position - self.coxa_delta, self.step_time)
		# And slowly move joints on which robot stay
		self.write_group(coxa_b_group, coxa_default_position + self.coxa_delta, 0.01)

	def turn_right(self):
		# Stay on A group
		self.write_group(tibia_b_group, tibia_default_position + self.tibia_delta, self.step_time)
		self.write_group(tibia_a_group, tibia_default_position - self.tibia_delta, self.step_time)

		# Move B group CW, A group CCW
		# Fastly move free joints
		self.write_group(coxa_a_group, coxa_default_position - self.coxa_delta, 0.01, False)
		# And slowly move joints on which robot stay
		self.write_group(coxa_b_group, coxa_default_position + self.coxa_delta, self.step_time, False)

		# Stay on B group
		self.write_group(tibia_a_group, tibia_default_position + self.tibia_delta, self.step_time)
		self.write_group(tibia_b_group, tibia_default_position - self.tibia_delta, self.step_time)

		# Move A group CW, B group CCW
		# Fastly move free joints
		self.write_group(coxa_b_group, coxa_default_position - self.coxa_delta, 0.01, False)
		# And slowly move joints on which robot stay
		self.write_group(coxa_a_group, coxa_default_position + self.coxa_delta, self.step_time, False)

	def turn_left(self):
		# Stay on A group
		self.write_group(tibia_b_group, tibia_default_position + self.tibia_delta, self.step_time)
		self.write_group(tibia_a_group, tibia_default_position - self.tibia_delta, self.step_time)

		# Move A group CW, B group CCW
		# Fastly move free joints
		self.write_group(coxa_b_group, coxa_default_position - self.coxa_delta, self.step_time, False)
		# And slowly move joints on which robot stay
		self.write_group(coxa_a_group, coxa_default_position + self.coxa_delta, 0.01, False)

		# Stay on B group
		self.write_group(tibia_a_group, tibia_default_position + self.tibia_delta, self.step_time)
		self.write_group(tibia_b_group, tibia_default_position - self.tibia_delta, self.step_time)

		# Move B group CW, A group CCW
		# Fastly move free joints
		self.write_group(coxa_a_group, coxa_default_position - self.coxa_delta, self.step_time, False)
		# And slowly move joints on which robot stay
		self.write_group(coxa_b_group, coxa_default_position + self.coxa_delta, 0.01, False)

	def stop(self):
		self.write_group(coxa_group, coxa_default_position, self.step_time)
		self.write_group(tibia_group, tibia_default_position, self.step_time)

	def relax(self):
		# Turn off servos
		self.__coxa_driver.set_all_pwm(0, 0)
		self.__tibia_driver.set_all_pwm(0, 0)

	def __del__(self):
		self.relax()