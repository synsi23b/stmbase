import synrpc_usbcon as usb
from time import sleep
import random
import time

current_milli_time = lambda: int(round(time.time() * 1000))
current_micro_time = lambda: int(round(time.time()))
import sys

ph = usb.PacketHandler("/dev/ttyACM0")

recv_datalist = []
start = end = current_micro_time()
bench_started = 0

def myhndl(msg):
    global start
    global bench_started
    # global end
    #end = current_micro_time()
    # #print(f"### {end - start} millisec ###")
    
    # recv_datalist.append((0, list(msg.values)))
    if bench_started:
        bench_started += 1
        if bench_started % 100 == 0:
            sys.stdout.write('.')
            sys.stdout.flush()
        if bench_started == 4096:
            bench_started = 0
    #start = end


ph.addMessageHandler(usb.SetPotentiometerMsg, myhndl)

ph.start()
#msg = usb.PerfTestMsg()
#maxval = 4 * 1000 * 1000 * 1000
msg = usb.SetPotentiometerMsg()

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

