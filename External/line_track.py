# Untitled - By: 16212 - 周二 7月 11 2023

import sensor, image, lcd
from machine import UART
from fpioa_manager import fm
import math

l = 33

fm.register(18, fm.fpioa.UART1_TX, force=True)
fm.register(19, fm.fpioa.UART1_RX, force=True)
uart1 = UART(UART.UART1, 115200, 8, 0, 1, timeout=1000, read_buf_len=4096)

def uart1_send(data):
    # global uart;
    cmd = "<!" + str(data) + ">!\r\n"
    uart1.write(cmd);

lcd.init()
lcd.rotation(2)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.skip_frames()
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.run(1)

line_threshold = [(0, 100, 10, 44, -10, 29)] # 表示线的颜色范围
roi_line = [0,100,320,16] # 巡线敏感区

while True:
    img = sensor.snapshot()
    blobs_line = img.find_blobs(line_threshold, roi=roi_line, area_threshold=200, merge=True)
    # area_threshold 面积阈值; merge=True 将所有重叠的blob合并为一个; margin 边界, 如果色块距离小于 120 会被合并

    if blobs_line:
        for b in blobs_line:
            tmp = img.draw_rectangle(b[0:4])
            tmp = img.draw_cross(b[5], b[6])
            delta_x = (160 - b[5]) * (22 / 320)
            reciprocal_of_r = delta_x / (l * math.sqrt(pow(delta_x, 2) + pow(l, 2)))
            uart1_send(reciprocal_of_r)
    # lcd.display(img)
