#!/usr/bin/python3


from __future__ import print_function
import time
from datetime import datetime
from RF24 import *
import RPi.GPIO as GPIO


radio = RF24(25,0, 5000000)

index = 0
counter = 0

def try_read_data(channel=0):
    global index
    global counter
    if radio.available():
        counter += 1
        if counter == 1:
                counter = 0
                mypl = bytes([index % 255] * 32)
                print("Putting mypl {}".format(mypl))
                radio.writeAckPayload(1, mypl)
                index += 1
        while radio.available():
            length = radio.getDynamicPayloadSize()
            receive_payload = radio.read(length)
            print('{} Got payload size={} -> {}'.format(datetime.now(), length, list(receive_payload)))


my_adr = bytes("RECV", 'utf-8')
mico_adr = bytes("SEND", 'utf-8')

millis = lambda: int(round(time.time() * 1000))

radio.begin()
radio.setPALevel(RF24_PA_MAX)
radio.setAddressWidth(4)
radio.enableDynamicPayloads()
radio.enableAckPayload()
radio.setRetries(3, 10)
radio.channel = 78

print(' ************ Role Setup *********** ')

print('Role: awaiting transmission')
#radio.openWritingPipe(mico_adr)
radio.openReadingPipe(1, my_adr)

radio.printPrettyDetails()

radio.startListening()

# forever loop
while 1:
        # Pong back role.  Receive each packet, dump it out, and send it back
#        radio.printDetails()
        try_read_data()
#        radio.printDetails()
        time.sleep(0.02)

