#!/usr/bin/python3

import time
from RF24 import *
#import RPi.GPIO as GPIO
import struct

uint_16_packer = struct.Struct(">H")

radio = RF24(25, 0, 5000000)

counter_recv_total = 0
counter_send_total = 0

nodes_last_id = {}
node_duplicate_counter = {}


def start_listen(this_node_address, message_callbacks):
    radio.begin()
    radio.setPALevel(RF24_PA_MAX)
    radio.setAddressWidth(4)
    radio.enableDynamicPayloads()
    radio.enableAckPayload()
    # 1 millisecond 10 tries
    radio.setRetries(3, 10)
    radio.channel = 78
    radio.openReadingPipe(1, bytes(this_node_address, 'utf-8'))
    radio.printPrettyDetails()
    radio.startListening()

    while True:
        _read_all_data(message_callbacks)
        time.sleep(0.1)


def _unfold_address(raw_payload):
    folded_adr = uint_16_packer.unpack(raw_payload)[0]
    number = folded_adr & 0x3F
    folded_adr >>= 6
    second_char = (folded_adr & 0x1F) + ord('A')
    folded_adr >>= 5
    first_char = folded_adr + ord('A')
    if number < 10:
        return f"{chr(first_char)}{chr(second_char)}0{number}"
    else:
        return f"{chr(first_char)}{chr(second_char)}{number}"

def _read_all_data(message_callbacks):
    global counter_recv_total
    if radio.available():
        while radio.available():
            counter_recv_total += 1
            length = radio.getDynamicPayloadSize()
            raw_payload = radio.read(length)
            if length < 3:
                print("Error: Payload less than 5 bytes. length is to small, skipping")
                continue
            node_address = _unfold_address(raw_payload[:2])
            protocol = int(raw_payload[2])
            packet_id = protocol & 0xC0
            protocol = protocol & 0x3F
            payload = raw_payload[3:]
            protocol_sorter(message_callbacks, node_address, packet_id, protocol, payload)


def protocol_sorter(protocols, node, packet_id, protocol, payload):
    #print(f"{receive_time} - {node} - {packet_id} - {protocol} - {payload}")
    last_id = nodes_last_id.get(node, -1)
    if last_id != packet_id:
        nodes_last_id[node] = packet_id
        if protocol in protocols:
            try:
                retval = protocols[protocol](node, payload)
                if retval != None:
                    print(f"Error: Handler retunred: {retval}")
            except Exception as e:
                print(f"Exception during protocol execution: {e}")
        else:
            print(f"Error: Protocol unknown: {protocol}")
    else:
        # skip the packet, it is a repeated ID from the same node -> duplicate message
        if node in node_duplicate_counter:
            node_duplicate_counter[node] += 1
        else:
            node_duplicate_counter[node] = 1


def put_ack_payload(pipenum, bytes_data):
    radio.writeAckPayload(pipenum, bytes_data)
