import time
import os

class SysfsPWM:
    def __init__(self, chip, channel):
        self.base = f"/sys/class/pwm/pwmchip{chip}"
        self.channel = channel
        self.path = f"{self.base}/pwm{channel}"

        # export if needed
        if not os.path.exists(self.path):
            with open(f"{self.base}/export", "w") as f:
                f.write(str(channel))
            time.sleep(0.1)  # allow sysfs to create files

        # 20 ms period (50 Hz)
        self.period = 20_000_000

        with open(f"{self.path}/period", "w") as f:
            f.write(str(self.period))

        with open(f"{self.path}/enable", "w") as f:
            f.write("1")

    def duty(self, percent):
        percent = max(0, min(100, percent))
        duty = int(self.period * percent / 100)
        with open(f"{self.path}/duty_cycle", "w") as f:
            f.write(str(duty))

    def stop(self):
        with open(f"{self.path}/enable", "w") as f:
            f.write("0")


# channels you used earlier
left = SysfsPWM(0, 2)
right = SysfsPWM(0, 0)

print("Starting sysfs PWM test")

try:
    while True:
        for d in range(0, 101, 10):
            print(f"duty {d}%")
            left.duty(d)
            right.duty(d)
            time.sleep(0.5)

        for d in range(100, -1, -10):
            print(f"duty {d}%")
            left.duty(d)
            right.duty(d)
            time.sleep(0.5)

except KeyboardInterrupt:
    pass

finally:
    left.duty(0)
    right.duty(0)
    left.stop()
    right.stop()
