from logger_base import start_listen
from datetime import datetime
from mongo import insert_message, insert_key_value, insert_task_status, insert_temperature_value, insert_radio_state
import struct

int_16_packer = struct.Struct(">h")
uint_16_packer = struct.Struct(">H")
uint_32_packer = struct.Struct(">I")

def request_address(node, payload):
    pass


def request_time(node, payload):
    pass


def radio_state(node, payload):
    success = uint_32_packer.unpack(payload[:4])[0]
    failure = uint_32_packer.unpack(payload[4:8])[0]
    total = success + failure
    percent = 0.00
    if total > 0:
        percent = float(success) / float(total)
    insert_radio_state(node, success, failure, percent) 


def log_key_value(node, payload):
    msg = str(payload, encoding='utf-8')
    values = {}
    try:
        while True:
            value = None
            key, remainder = msg.split(':', 1)
            if remainder[0].isdigit() or remainder[0] == '-':
                for i in range(1, len(remainder)):
                    if not remainder[i].isdigit():
                        value = int(remainder[:i])
                        msg = remainder[i:]
                        break
                if value is None:
                    value = int(remainder)
            else:
                value, msg = remainder.split('\6', 1)
            values[key] = value
    except Exception as e:
        #print(e)
        pass
    if values:
        insert_key_value(node, values)


def _msg(severity, node, payload):
    msg = str(payload, encoding='utf-8')
    print(f"{datetime.now()} {node} {severity}: {msg}")
    insert_message(severity, node, msg)


def message_info(node, payload):
    _msg("Info", node, payload)


def message_warning(node, payload):
    _msg("Warning", node, payload)


def message_error(node, payload):
    _msg("Error", node, payload)


def _task(state, node, payload):
    percent = int(payload[0])
    remaining = uint_16_packer.unpack(payload[1:3])[0]
    description = str(payload[3:], encoding='utf-8')
    if(state == 'completed'):
        # the task completed successfull if we reach 100 percent, else it's consodered failed
        insert_task_status(state, node, description, remaining, percent, percent < 100)
    else:
        insert_task_status(state, node, description, remaining, percent, False)


def task_started(node, payload):
    _task('started', node, payload)


def task_progress(node, payload):
    _task('progressing', node, payload)


def task_completed(node, payload):
    _task('completed', node, payload)


def temperature_value(node, payload):
    termometer = str(payload[:16], encoding='utf-8')
    temperature = int_16_packer.unpack(payload[16:])[0]
    temperature = temperature * 0.0625
    insert_temperature_value(node, termometer, temperature)


protocols = {
    0x00: request_address,
    0x01: request_time,
    0x1F: radio_state,
    0x20: log_key_value,
    0x21: message_info,
    0x22: message_warning,
    0x23: message_error,
    0x26: task_started,
    0x37: task_progress,
    0x38: task_completed,
    0x3A: temperature_value
}

node_last_receive_time = None


# doesn't return
start_listen("LOGR", protocols)
