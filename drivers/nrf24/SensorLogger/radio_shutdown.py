#!/usr/bin/python3

from __future__ import print_function
import time
from RF24 import *
import RPi.GPIO as GPIO


radio = RF24(25,0, 5000000)

radio.begin()
radio.printPrettyDetails()
radio.powerDown()
radio.printPrettyDetails()

