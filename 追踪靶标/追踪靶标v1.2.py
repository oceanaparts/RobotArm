# Untitled - By: ocean - 周二 2月 23 2021
import sensor, image, time
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(False)                       #水平镜像关闭
sensor.set_vflip(False)                         #竖直镜像关闭
#sensor.skip_frames(time = 2000)
sensor.run(1)                                   #图像捕捉功能控制   1使能  2关闭
sum_x=0
sum_y=0
min_r=1000                                      #设置一个比较大的值，以使第一次比较成立
t=0
clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    c=img.find_circles(threshold = 1500, x_margin = 2, y_margin =2, r_margin =5,r_min = 50, r_max = 80, r_step =20)
    if c:
        #均值滤波
        for (i,n)in enumerate(c):
            sum_x=sum_x+n.x()               #以同心圆的共同圆心的均值为追踪圆心
            sum_y=sum_y+n.y()
            t=t+1
            if n.r()<min_r:                 #寻找最小半径
                min_r=n.r()
        x=int(sum_x/t)
        y=int(sum_y/t)
        sum_x=0
        sum_y=0
        t=0
        img.draw_circle(x, y, min_r, color = (255, 0, 0),thickness=3)
        min_r=1000
        print("坐标是X:%d,Y:%d"%(x,y))
    print(clock.fps())
