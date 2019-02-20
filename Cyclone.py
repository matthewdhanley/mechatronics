import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

LEDPINS = [20, 21, 22, 23, 24, 25, 26]
BUTTON = 4


def flashLed(pin):
    """
    Flashes the led 5 times with 0.3 seconds in between
    :param pin: GPIO pin to flash
    :return: None
    """
    for i in range(0, 5):
        GPIO.output(pin, 1)
        time.sleep(0.3)
        GPIO.output(pin, 0)
        time.sleep(0.3)


def turnOffAll():
    """
    Turns off all LEDs
    :return: None
    """
    for j in LEDPINS:
        GPIO.output(j, 0)


def cleanup():
    """
    Turns off the LEDs and cleans up gpio ports. Also quits.
    :return: None
    """
    turnOffAll()
    GPIO.cleanup()
    print("Done!")
    exit(0)


def setup():
    """
    Sets warnings to false, sets up output pins for leds, sets up input pin for button
    :return:
    """
    GPIO.setwarnings(False)
    for ii in LEDPINS:
        GPIO.setup(ii, GPIO.OUT)
    # need pull_up_down=GPIO.PUD_DOWN to make default low
    GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def main():
    setup()
    ii = 0
    change_time = 0.1
    last_update_time = 0
    last_reading = 0
    # Loop
    while 1:
        startTime = time.clock()
    
        if startTime - last_update_time > change_time:
            turnOffAll()
            GPIO.output(LEDPINS[ii], 1)
            ii += 1
            ii = ii % 7
            last_update_time = startTime
    
        tmp_reading = GPIO.input(BUTTON)
        if tmp_reading != last_reading:
            reading = tmp_reading
        else:
            reading = 0
        last_reading = tmp_reading
    
        if reading and LEDPINS[ii-1] == 23:
            flashLed(23)
            turnOffAll()
            cleanup()


if __name__ == "__main__":
    main()
