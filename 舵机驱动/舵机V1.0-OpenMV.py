
# 舵机控制示例。
#
# 这个例子演示了舵机扩展板。请按照以下步骤操作：
#
#   1. 连接舵机到任何PWM输出。
#   2. 将3.7v电池（或5V电源）连接到VIN和GND。
#   3. 将pca9685.py和servo.py复制到OpenMV并重置。
#   4. 在IDE中连接并运行此脚本。

import time
from servo import Servos
from machine import I2C, Pin

i2c = I2C(sda=Pin('P5'), scl=Pin('P4'))
servo = Servos(i2c, address=0x40, freq=50, min_us=650, max_us=2800, degrees=180)

while True:
    for i in range(0, 8):
        servo.position(i, 0)
    time.sleep_ms(500)
    for i in range(0, 8):
        servo.position(i, 180)
    time.sleep_ms(500)
