import synrpc_usbcon as usb
from time import sleep
import random
import time

current_milli_time = lambda: int(round(time.time() * 1000))
current_micro_time = lambda: int(round(time.time()))
import sys

ph = usb.PacketHandler("COM3")

start = current_milli_time()

def myhndl(msg):
    global start
    end = current_milli_time()
    print(f"### {end - start} millisec ###")
    newlist = []
    for duty, on in zip(msg.width, msg.t_on):
        if duty > 0:
            newlist.append(int(on / duty * 100))
        
    print(newlist)
    start = end

ph.addMessageHandler(usb.PulseReportMsg, myhndl)

ph.start()

while True:
    #msg = raw_input("Write Message: ")
    #dsp = int(raw_input("Write display: "))
    #line = int(raw_input("Write line:"))
    
    
    x = input("start benchmark")
    #ph.sendMessage(msg)
    #print("done")
    # for dsp in range(0, 4):
    #     for line in range(0, 4):
    #         msg = "bla " + str(dsp) + " " + str(line) + 
    #         mm = usb.WriteLineMsg()
    #         mm.message = msg
    #         mm.display = dsp
    #         mm.line = line
    #         ph.sendMessage(mm)
    # recv_datalist = []
    # datalist = []
    # data = [0] * 122
    # for i in range(4096):
    #     for i in range(122):
    #         data[i] = random.randint(0, 255)
    #     datalist.append(list(data))

    # start_loop = current_milli_time()
    # counter = 0
    # for d in datalist:
    #     counter += 1
    #     if counter == 100:
    #         sys.stdout.write('.')
    #         sys.stdout.flush()
    #         counter = 0
    #     msg.values = d
    #     #start = current_milli_time()
    #     ph.sendMessage(msg)
    #     #sleep(0.001)
    # end_loop = current_milli_time()

    # print(f"\n### {(end_loop - start_loop)} millisec ###")

    # minv = 1000
    # maxv = 0
    # for recv, send in zip(recv_datalist, datalist):
    #     minv = min(minv, recv[0])
    #     maxv = max(maxv, recv[0])
    #     if recv[1] != send:
    #         print("Data mismatch")
    # print(f"Min: {minv}\nMax: {maxv}")

    bench_started = 1
    start_loop = current_milli_time()
    while bench_started:
        sleep(0.01)
    end_loop = current_milli_time()

    print(f"\n### {(end_loop - start_loop)} millisec ###")
    # minv = 1000000
    # maxv = 0
    # prev = 0
    # for recv in recv_datalist:
    #     minv = min(minv, recv[0])
    #     maxv = max(maxv, recv[0])
    # print(f"Min: {minv}\nMax: {maxv}")



ph.shutdown()

