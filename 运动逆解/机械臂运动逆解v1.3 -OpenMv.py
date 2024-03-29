# openMv版
import utime
from servo import Servos
from machine import I2C, Pin
import math
import time
#v1.3
#1、该版本较1.2版本求解了两种解进行判断，优化了可到达空间的同时运动的连续性也可保证
#2、运动逆解坐标单位可修改
last_flag="angle_neg"
#使机械臂处于竖直状态时的各舵机角
motor0=135
motor1=135
motor2=135
motor3=135
h_angle=0
#PCA9685的I2C
i2c = I2C(sda=Pin('P5'), scl=Pin('P4'))
#实例化一个舵机对象
servo = Servos(i2c, address=0x40, freq=50, min_us=500, max_us=2500, degrees=270)
def Handle_angle(j0,j1,j2,j3):
    if((j0+motor0>0)and(j0+motor0<270)):
        j0=j0+motor0
    else:
        return 0
    if((motor1-90+j1>80)and(motor1-90+j1<270)):
        j1=motor1-90+j1
    else:
        return 0
    if((motor2+j2>30)and(motor2+j2<220)):
        j2=motor2+j2
    else:
        return 0
    if((motor3+j3>22)and(motor3+j3<230)):
        j3=motor3+j3
    else:
        return 0
    return j0,j1,j2,j3
def inverseKinematics(x,y,z,Alpha,unit=1):#这里的度数是以水平面逆时针旋转的角度，这里的xyz是第四个舵机轴的位置
    'unit可修改x,y,z单位，默认为mm当unit为10时，单位是1/10毫米以此类推，单位越小精度越高求解越慢'
    l1=104*unit
    l2=93*unit
    l3=400*unit
    theta0 = math.atan2(y, x)
    #theta2
    if(y!=0):
        x=x/math.cos(theta0)
    temp=(math.pow(x,2)+math.pow(z,2)-math.pow(l1,2)-math.pow(l2,2))/(2*l1*l2)
    if((temp>-1)and(temp<1)):
        theta2_pos=math.acos(temp)
        theta2_neg=-math.acos(temp)
    else:
        return 0
    #theta1
    a=math.pow(l2,2)-math.pow(x,2)-math.pow(z,2)-math.pow(l1,2)
    b=-2*l1*math.sqrt(math.pow(x,2)+math.pow(z,2))
    m=a/b
    if((m>-1)and(m<1)):
            theta1_pos=math.atan2(z,x)-math.acos(m)
            theta1_neg=math.atan2(z,x)+math.acos(m)
    else:
        return 0
    #theta3
    theta3_pos=Alpha-math.degrees(theta1_pos)-math.degrees(theta2_pos)
    theta3_neg=Alpha-math.degrees(theta1_neg)-math.degrees(theta2_neg)
    print("j2为正值的解为j0:%f,j1:%f,j2:%f,j3:%f,x:%f,y:%f,z:%f\r\n"%(math.degrees(theta0), math.degrees(theta1_pos),math.degrees(theta2_pos), theta3_pos, x, y, z))
    print("j2为负值值的解为j0:%f,j1:%f,j2:%f,j3:%f,x:%f,y:%f,z:%f\r\n"%(math.degrees(theta0), math.degrees(theta1_neg),math.degrees(theta2_neg), theta3_neg, x, y, z))
    return math.degrees(theta0),math.degrees(theta1_pos),math.degrees(theta2_pos),theta3_pos,math.degrees(theta0),math.degrees(theta1_neg),math.degrees(theta2_neg),theta3_neg


def write_position(x,y,z,a,mode,unit=1):#mode等于1表示不参考上一解，0表示参考上一解，如果上一次解是pos这次是neg则不写入跳过
    'mode=1，可使可到达空间增加，但对连续的运动，比如让他画一条直线，可能出现运动的跳跃'
    'mode=0，使可到达空间减小，但是运动连续'
    global last_flag
    flag,flag_neg,flag_pos=0,0,0
    an=inverseKinematics(x,y,z,a,unit)
    if(an!=0):
        j0,j1,j2,j3,j0_neg,j1_neg,j2_neg,j3_neg=an
        flag=1
    else:
        print("无解")
        flag=0
    #运动逆解有解的情况下，进行下面一步判断，舵机是否因角度受限到不了
    if(flag==1):
        an_pos=Handle_angle(j0,j1,j2,j3)
        an_neg=Handle_angle(j0_neg,j1_neg,j2_neg,j3_neg)
        if(an_neg!=0):
            angle_neg=an_neg
            flag_neg=1
        elif((an_pos!=0)):
            angle_pos=an_pos
            flag_pos=1
        else:
            print("角度超过舵机所限角度")
            flag=0
            flag_neg=0
            flag_pos=0
    else:
        return 0
    #舵机角不受限的情况下选择解
    if ((flag_neg==1)and(flag_pos==1)and((last_flag=="angle_neg")or(mode))):#两种都有解选neg
        for i in range(0,4):
            servo.position(i,angle_neg[i])
            print("有两解")
            print(angle_neg[i])
        last_flag="angle_neg"
        flag_neg=0                      #写完需要对标志位清零，否则后面的还会执行
        flag_pos=0
        return 1
    elif ((flag_pos==1)and((last_flag=="angle_pos")or(mode))):
        for i in range(0,4):
            servo.position(i,angle_pos[i])
            print("有pos解")
            print(angle_pos[i])
        last_flag="angle_pos"
        flag_pos=0
        return 1
    elif ((flag_neg==1)and((last_flag=="angle_neg")or(mode))):
        for i in range(0,4):
            servo.position(i,angle_neg[i])
            print("有neg解")
            print(angle_neg[i])
        last_flag="angle_neg"
        flag_neg=0
        return 1
    elif((flag_neg==1)or(flag_pos==1)):#此情况发生在mode为0下
        print("上下解不一，运动不连续，舍弃")
        #last_flag="no_solution"
    else:
        return 0

utime.sleep_ms(1000)

while(True):#x轴平移
    for j in range(121,60,-1):
        write_position(j,0,150,0,1,1)
    for j in range(60,121,1):
        write_position(j,0,150,0,1,1)
#while(True):#x轴平移
    #for j in range(160,130,-1):
        #write_position(10,0,j/10,0)
    #for j in range(130,160,1):
        #write_position(10,0,j/10,0)



