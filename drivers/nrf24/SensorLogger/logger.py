from logger_base import start_listen
from datetime import datetime
from mongo import insert_message, insert_key_value, insert_task_status, insert_temperature_value, update_radio_state
import struct
import argparse

parser = argparse.ArgumentParser(description='RF24Logger -> Collect Sensor Data and send to MongoDB')
parser.add_argument('--verbose', dest='verbose', action='store_const',
                    const=True, default=False,
                    help='increase programm chatter (default: only errors)')

args = parser.parse_args()


int_16_packer = struct.Struct(">h")
uint_16_packer = struct.Struct(">H")
#uint_32_packer = struct.Struct(">I")

def request_address(node, payload):
    pass


def request_time(node, payload):
    pass


def _unpack_string(indata):
    idx = 0
    for c in indata:
        if c == 0:
            break
        idx += 1
    return [str(indata[:idx], encoding='utf-8')]


_unpack_splitters = {
    # uint8
    1: (struct.Struct(">B").unpack, 1),
    # int8
    2: (struct.Struct(">b").unpack, 1),
    # uint16
    3: (struct.Struct(">H").unpack, 2),
    # int16
    4: (struct.Struct(">h").unpack, 2),
    # string
    5: (_unpack_string, -1)
}


def _get_key_and_splitter(payload, startidx):
    idx = 0 + startidx
    while True:
        val = payload[idx]
        if val in _unpack_splitters:
            key = str(payload[startidx:idx], encoding='utf-8')
            splitter = _unpack_splitters[val]
            return key, splitter, idx + 1
        idx += 1

def _unpack_key_value(payload):
    #msg = str(payload, encoding='utf-8')
    values = {}
    try:
        index = 0
        while True:
            # first find the end of the key
            key, splitter, index = _get_key_and_splitter(payload, index)
            end_of_data = splitter[1]
            # than check data size and exctract it from payload
            if end_of_data > 0:
                value = splitter[0](payload[index : index + end_of_data])[0]
                index += end_of_data
            else:
                value = splitter[0](payload[index : ])[0]
                index += len(value)
            # finally, add to our result dictionary
            values[key] = value
    except Exception as e:
        #print(e)
        pass
    return values


def radio_state(node, payload):
    values = _unpack_key_value(payload)
    success = values['s']
    failure = values['f']
    lost_messages = values.get('l', 0)
    out_overflow = values.get('o', 0)
    update_radio_state(node, success, failure, lost_messages, out_overflow)


def log_key_value(node, payload):
    values = _unpack_key_value(payload)
    if values:
        insert_key_value(node, values)


def _msg(severity, node, payload):
    msg = str(payload, encoding='utf-8')
    print(f"{datetime.now()} {node} {severity}: {msg}")
    insert_message(severity, node, msg)


def message_info(node, payload):
    _msg("info", node, payload)


def message_warning(node, payload):
    _msg("warning", node, payload)


def message_error(node, payload):
    _msg("error", node, payload)


def _task(state, node, payload):
    percent = int(payload[0])
    remaining = uint_16_packer.unpack(payload[1:3])[0]
    description = str(payload[3:], encoding='utf-8')
    if(state == 'completed'):
        # the task completed successfull if we reach 100 percent, else it's consodered failed
        insert_task_status(state, node, description, remaining, percent, percent < 100)
    elif(state == 'canceled'):
        insert_task_status(state, node, description, remaining, percent, True)
    else:
        insert_task_status(state, node, description, remaining, percent, False)


def task_started(node, payload):
    _task('started', node, payload)


def task_progress(node, payload):
    _task('progressing', node, payload)


def task_completed(node, payload):
    _task('completed', node, payload)


def task_canceled(node, payload):
    _task('canceled', node, payload)


def temperature_value(node, payload):
    thermometer = payload[:8].hex()
    temperature = int_16_packer.unpack(payload[8:])[0]
    temperature = temperature * 0.0625
    insert_temperature_value(node, thermometer, temperature)


protocols = {
    0x00: request_address,
    0x01: request_time,
    0x20: radio_state,
    0x21: log_key_value,
    0x22: message_info,
    0x23: message_warning,
    0x24: message_error,
    0x25: task_started,
    0x26: task_progress,
    0x27: task_completed,
    0x28: task_canceled,
    0x29: temperature_value
}

node_last_receive_time = None


# doesn't return
start_listen("LG01", protocols, args.verbose)
