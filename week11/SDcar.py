import threading
import time
import RPi.GPIO as GPIO

# Car control class
class MotorControl:
    def __init__(self):
        # Dictionary to store GPIO pin mappings
        self.pins = {
            "SW1": 5, "SW2": 6, "SW3": 13, "SW4": 19,
            "PWMA": 18, "AIN1": 22, "AIN2": 27,
            "PWMB": 23, "BIN1": 25, "BIN2": 24
        }
        self.setup_gpio()  # Setup the GPIO pins
        # PWM setup for left and right motors
        self.left_motor = GPIO.PWM(self.pins["PWMA"], 500)
        self.left_motor.start(0)
        self.right_motor = GPIO.PWM(self.pins["PWMB"], 500)
        self.right_motor.start(0)

    def setup_gpio(self):
        """Initialize GPIO settings."""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        # Setup switches as input pins with pull-down resistors
        GPIO.setup(self.pins["SW1"], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pins["SW2"], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pins["SW3"], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pins["SW4"], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # Setup motor control pins as output
        GPIO.setup(self.pins["PWMA"], GPIO.OUT)
        GPIO.setup(self.pins["AIN1"], GPIO.OUT)
        GPIO.setup(self.pins["AIN2"], GPIO.OUT)
        GPIO.setup(self.pins["PWMB"], GPIO.OUT)
        GPIO.setup(self.pins["BIN1"], GPIO.OUT)
        GPIO.setup(self.pins["BIN2"], GPIO.OUT)

    def cleanup_gpio(self):
        """Clean up GPIO setup."""
        GPIO.cleanup()

    def move_forward(self, speed):
        """Move the car forward at a specified speed."""
        GPIO.output(self.pins["AIN1"], 0)
        GPIO.output(self.pins["AIN2"], 1)
        self.left_motor.ChangeDutyCycle(speed)
        GPIO.output(self.pins["BIN1"], 0)
        GPIO.output(self.pins["BIN2"], 1)
        self.right_motor.ChangeDutyCycle(speed)

    def move_backward(self, speed):
        """Move the car backward at a specified speed."""
        GPIO.output(self.pins["AIN1"], 1)
        GPIO.output(self.pins["AIN2"], 0)
        self.left_motor.ChangeDutyCycle(speed)
        GPIO.output(self.pins["BIN1"], 1)
        GPIO.output(self.pins["BIN2"], 0)
        self.right_motor.ChangeDutyCycle(speed)

    def turn_left(self, speed):
        """Turn the car left at a specified speed."""
        GPIO.output(self.pins["AIN1"], 1)
        GPIO.output(self.pins["AIN2"], 0)
        self.left_motor.ChangeDutyCycle(speed)
        GPIO.output(self.pins["BIN1"], 0)
        GPIO.output(self.pins["BIN2"], 1)
        self.right_motor.ChangeDutyCycle(speed)

    def turn_right(self, speed):
        """Turn the car right at a specified speed."""
        GPIO.output(self.pins["AIN1"], 0)
        GPIO.output(self.pins["AIN2"], 1)
        self.left_motor.ChangeDutyCycle(speed)
        GPIO.output(self.pins["BIN1"], 1)
        GPIO.output(self.pins["BIN2"], 0)
        self.right_motor.ChangeDutyCycle(speed)

    def stop_motors(self):
        """Stop the motors."""
        GPIO.output(self.pins["AIN1"], 0)
        GPIO.output(self.pins["AIN2"], 1)
        self.left_motor.ChangeDutyCycle(0)
        GPIO.output(self.pins["BIN1"], 0)
        GPIO.output(self.pins["BIN2"], 1)
        self.right_motor.ChangeDutyCycle(0)

# Main execution block
if __name__ == '__main__':
    car = MotorControl()  # Create car object
    
    # Testing motor control with a sequence of movements
    car.move_forward(100)  # Move forward at full speed
    time.sleep(2)
    car.turn_left(100)  # Turn left at full speed
    time.sleep(2)
    car.turn_right(100)  # Turn right at full speed
    time.sleep(2)
    car.move_backward(100)  # Move backward at full speed
    time.sleep(2)
    
    # Clean up the GPIO pins when done
    car.cleanup_gpio()
