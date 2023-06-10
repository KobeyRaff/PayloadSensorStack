import sensor, image, time, pyb, uos, machine

# Set up the GPIO pin
PIN_NUMBER = "P5"  # Use your desired GPIO pin number here
pin = pyb.Pin(PIN_NUMBER, pyb.Pin.IN, pyb.Pin.PULL_DOWN)

# Initialize the camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

led_blue = pyb.LED(1)
led_red = pyb.LED(2)
led_green = pyb.LED(3)
led_blue.off();
led_red.off();
led_green.off();

# Create a new directory for this power-on session
def get_next_session_dir():
    existing_dirs = [d for d in uos.ilistdir() if uos.stat(d[0])[0] & 0x4000]  # 0x4000 is the directory flag
    existing_nums = [int(d[0].replace("session_", "")) for d in existing_dirs if d[0].startswith("session_")]
    next_num = max(existing_nums) + 1 if existing_nums else 1
    return "/session_" + str(next_num)

session_dir = get_next_session_dir()
uos.mkdir(session_dir)

def record_images(duration):
    led_blue.on();
    led_red.on();
    led_green.on();
    frame_count = 0
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration:
        img = sensor.snapshot()
        img.save(session_dir + "/frame_%08d.jpg" % frame_count, quality=95)  # save frame
        frame_count += 1
          # Adjust the frame delay as needed for 30 fps

try:
    while True:
        # Check if the GPIO pin is HIGH
        if pin.value() == 1:
            print("GPIO pin is HIGH. Starting 30-second image recording.")
            record_images(120000)  # Record for 2 minutes
            print("Image recording finished. Going into deep sleep.")
            led_blue.off();
            led_red.off();
            led_green.off();
            machine.deepsleep()
        time.sleep(1)  # Adjust the polling delay as needed
except KeyboardInterrupt:
    # Handle the exception if the script is terminated
    pass
