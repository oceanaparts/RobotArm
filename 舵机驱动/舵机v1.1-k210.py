import utime
from machine import I2C

i2c = I2C(I2C.I2C0,freq=100000, scl=28, sda=29)
#scl和sda看自己具体插哪里而更改的

def pca_setfreq(freqs):
    freqs *= 0.92
    prescaleval = 25000000
    prescaleval /= 4096
    prescaleval /= freqs
    prescaleval -= 1
    prescale =int(prescaleval + 0.5)

    oldmode = i2c.readfrom_mem(0x40,0x00,8)#地址可能是0X80

    newmode = (oldmode[0]&0x7F) | 0x10  #sleep

    i2c.writeto_mem(0x40,0x00,newmode) #go to sleep

    i2c.writeto_mem(0x40,0xFE, prescale)  #set the prescaler

    i2c.writeto_mem(0x40,0x00, oldmode)
    utime.sleep_ms(2)

    i2c.writeto_mem(0x40,0x00, oldmode[0]|0xa1)

def pca_setpwm(num,on,off):
    i2c.writeto_mem(0x40,0x06+4*num,on)
    i2c.writeto_mem(0x40,0x07+4*num,on>>8)
    i2c.writeto_mem(0x40,0x08+4*num,off)
    i2c.writeto_mem(0x40,0x09+4*num,off>>8)

def pca_init(hz,angle): #初始化函数
    off = 0
    i2c.writeto_mem(0x40,0x00,0x0)
    pca_setfreq(hz)
    off = int(82+angle*1.3259259+0.5)#这里可能有问题
    pca_setpwm(0,0,off)
    pca_setpwm(1,0,off)
    pca_setpwm(2,0,off)
    pca_setpwm(3,0,off)
    pca_setpwm(4,0,off)
    pca_setpwm(5,0,off)
    #pca_setpwm(6,0,off)
    #pca_setpwm(7,0,off)
    #pca_setpwm(8,0,off)
    #pca_setpwm(9,0,off)
    #pca_setpwm(10,0,off)
    #pca_setpwm(11,0,off)
    #pca_setpwm(12,0,off)
    #pca_setpwm(13,0,off)
    #pca_setpwm(14,0,off)
    #pca_setpwm(15,0,off)
    utime.sleep_ms(500)

def pca_mg90(num,angle):
        off=int(82+angle*1.3259259+0.5)#这里可能有问题--------待调试1.517037037
        pca_setpwm(num,0,off)
def PCA_MG9XX(num,start_angle,end_angle,mode,speed):
    off=0
    if mode==0:
        off=int(82+end_angle*1.3259259+0.5)
        pca_setpwm(num,0,off)
    elif mode==1:
        off=int(82+end_angle*1.3259259+0.5)
        pca_setpwm(num,0,off)
        if end_angle>start_angle:
            utime.sleep_ms(int((end_angle-start_angle)*8))
        else:
            utime.sleep_ms(int((start_angle-end_angle)*8))
    elif mode==2:
        if start_angle<end_angle:
            for i in range(start_angle,end_angle+1):
                off=int(82+i*1.3259259+0.5)
                pca_setpwm(num,0,off)
                utime.sleep_ms(5)
                utime.sleep_us(speed*300)
        elif start_angle>end_angle:
            for i in range(start_angle,end_angle+1,-1):
                off=int(82+i*1.3259259+0.5)
                pca_setpwm(num,0,off)
                utime.sleep_ms(5)
                utime.sleep_us(speed*300)

#pca_init(50,145)
#pca_init(50,70)
#pca_init(50,135)
#PCA_MG9XX(0,0,180,2,1)
#PCA_MG9XX(0,180,0,2,1)
while True:
    #PCA_MG9XX(0,90,0,2,0)#通道，起始角度，终点角度，模式，速度（模式才有2速度）
    #PCA_MG9XX(0,0,90,2,0)
    pca_mg90(0,124)
    utime.sleep_ms(1000)
    pca_mg90(1,30)
    ##utime.sleep_ms(1000)
    pca_mg90(2,78)
    ##utime.sleep_ms(1000)
    pca_mg90(3,182)
    #utime.sleep_ms(1500)
    pca_mg90(4,120)
    utime.sleep_ms(1000)
    #pca_mg90(6,180)
    #pca_mg90(1,20)

#while True:
    #PCA_MG9XX(0,0,180,2,1)
    #utime.sleep_ms(1000)
    #PCA_MG9XX(0,180,0,2,1)
    #utime.sleep_ms(1000)
