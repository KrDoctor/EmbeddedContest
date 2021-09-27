import serial
import matplotlib.pyplot as plt
import time

ser = serial.Serial('COM4',115200,timeout=0)
x_target = 99
y_target = 99
drone_target_x = 99
drone_target_y = 99
drone_x = 99
drone_y = 99
while 1:
    plt.xlim([0,5.12])
    plt.ylim([0,5.12])
    plt.grid()
    plt.pause(0.00001)
    # plt.pause(10)
    if ser.readable():
        plt.cla()
        msg = ser.readline()
        msg = msg.decode()

        newList = msg.replace('<','/').replace('>','/')
        newList = newList.split('/')
        if len(newList) != 1:
            print(msg)
            if newList[1] == 'target':
                try:
                    x_target = float(newList[2])
                    # print(x_target)
                    y_target = float(newList[3])
                    # print(y_target)
                    # plt.scatter(x_target,y_target,s = 10, c='r', alpha= 0.5)#s는 스케일 c 컬러 alpha는 불투명도
                except :
                    print('에러뜸')
            elif newList[1] == 'drone1':
                try:
                    # print(newList)
                    drone_x = float(newList[2])
                    drone_target_x = float(newList[3])
                    drone_y = float(newList[4])
                    drone_target_y = float(newList[5])

                    # plt.scatter(x_target, y_target, s=10, c='r', alpha=0.5)
                    # plt.scatter(drone_target_x, drone_target_y, s=10, c='g')
                    # plt.scatter(drone_x, drone_y, s=10, c='k')
                except :
                    print('에러뜸')
    plt.scatter(x_target, y_target, s=50, c='r')
    # plt.scatter(drone_target_x, drone_target_y, s=50, c='g')
    plt.scatter(drone_x, drone_y, s=50, c='k')
    plt.show(block=False)
    # if msg.split('/')[0] != 'target':
    #     print(msg.split('/')[1])
    #     print(msg.split('/')[1])
    #     if msg != '':
    #         print(msg)
    #         print(type(msg))