# Untitled - By: 16212 - 周二 7月 11 2023

import sensor, image, lcd

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # 灰度
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.skip_frames() # 跳过3000张图片
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.run(1)

line_threshold = [(0, 100, 16, 127, 0, 75)] # 表示线的颜色范围
roi1 = [0,100,320,16] # 巡线敏感区
roi2 = [0,180,320,8] # 关键点敏感区

while True:
    img = sensor.snapshot()
    blobs1 = img.find_blobs(line_threshold, roi=roi1, area_threshold=200, merge=True)
    blobs2 = img.find_blobs(line_threshold, roi=roi2, area_threshold=120, merge=True, margin=120)
    # area_threshold 面积阈值; merge=True 将所有重叠的blob合并为一个; margin 边界, 如果色块距离小于 120 会被合并

    if blobs1:
        for b in blobs1:
            tmp=img.draw_rectangle(b[0:4])
            tmp=img.draw_cross(b[5], b[6])
            c=img.get_pixel(b[5], b[6])
    lcd.display(img)

            #actualValue=b[5]
            #err=actualValue-expectedValue
            #Speed_left = Speed - (Kp*err+Kd*(err-old_err))
            #Speed_right = Speed + (Kp*err+Kd*(err-old_err))
            #old_err= err
            #print("Speed_left,Speed_right")
            #print(int(Speed_left),int(Speed_right))
    #if blobs2:
        #for b in blobs:
            #tmp=img.draw_rectangle(b[0:4])
            #tmp=img.draw_cross(b[5], b[6])
            #c=img.get_pixel(b[5], b[6])
            #if b[2] >50:
                #Flag = 1
    #sending_data(int(Speed_left),int(Speed_right),Flag)
