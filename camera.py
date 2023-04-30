import sensor, image, time, pyb, uos, machine, mjpeg

# Set up the GPIO pin
PIN_NUMBER = "P5"  # Use your desired GPIO pin number here
pin = pyb.Pin(PIN_NUMBER, pyb.Pin.IN, pyb.Pin.PULL_DOWN)

# Setup camera.
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
c=time.clock()

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

blue_led.on()
blue_led.off()
red_led.on()
red_led.off()
green_led.on()
green_led.off()

# Create the mjpeg object.
m = mjpeg.Mjpeg("video.mjpeg")

# Video recording duraation
VIDEO_DURATION = 30000  # 30 seconds

def record_video(duration):
    blue_led.on()
    green_led.on()
    red_led.on()

    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration:
        c.tick()
        m.add_frame(sensor.snapshot())

    blue_led.off()
    green_led.off()
    red_led.off()
    m.close(c.fps())

try:
    while True:
        # Check if the GPIO pin is HIGH
        if pin.value() == 1:
            print("GPIO pin is HIGH. Starting 30-second video recording.")
            record_video(VIDEO_DURATION)  # Record for 30 seconds
            print("Video recording finished. Going into deep sleep.")
            machine.deepsleep()
            time.sleep(10)  # Adjust the polling delay as needed
except KeyboardInterrupt:
    # Handle the exception if the script is terminated
    pass
