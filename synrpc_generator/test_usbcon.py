import synrpc_usbcon as usb
from time import sleep
import random
import time

current_milli_time = lambda: int(round(time.time() * 1000))
import sys

ph = usb.PacketHandler("COM3")


start = end = current_milli_time()
def myhndl(msg):
    global start
    global end
    end = current_milli_time()
    print(f"### {end - start} millisec ###")
    start = end

ph.addMessageHandler(usb.PerfTestMsg, myhndl)

ph.start()
msg = usb.PerfTestMsg()
maxval = 4 * 1000 * 1000 * 1000

while True:
    #msg = raw_input("Write Message: ")
    #dsp = int(raw_input("Write display: "))
    #line = int(raw_input("Write line:"))

    # for dsp in range(0, 4):
    #     for line in range(0, 4):
    #         msg = "bla " + str(dsp) + " " + str(line) + 
    #         mm = usb.WriteLineMsg()
    #         mm.message = msg
    #         mm.display = dsp
    #         mm.line = line
    #         ph.sendMessage(mm)
    # datalist = []
    # data = [0] * 63
    # for i in range(4096):
    #     for i in range(63):
    #         data[i] = random.randint(0, maxval)
    #     datalist.append(list(data))

    # start = current_milli_time()
    # counter = 0
    # for d in datalist:
    #     counter += 1
    #     if counter == 100:
    #         sys.stdout.write('.')
    #         sys.stdout.flush()
    #         counter = 0
    #     msg.values = d
    #     start = current_milli_time()
    #     ph.sendMessage(msg)
    #     sleep(0.5)
    # end = current_milli_time()

    # print(f"### {(end - start)} millisec ###")

    sleep(10)

ph.shutdown()

