import Jetson.GPIO as GPIO
import time

# Pin Definitions
output_pin = [4,17,27,22,10,9,11,5]   # BOARD pin 12, BCM pin 18

def main():
    # Pin Setup:
    # Board   pin-numbering scheme
    GPIO.setmode(GPIO.BCM)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.HIGH
    try:
        
        while True:
            time.sleep(1)
            # Toggle the output every second
            print("Outputting {} to pin {}".format(curr_value, output_pin))
            GPIO.output(output_pin, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
