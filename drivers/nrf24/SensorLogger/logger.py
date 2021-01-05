from logger_base import start_listen
from datetime import datetime
from mongo import insert_message, insert_key_value, insert_task_status, insert_temperature_value, update_radio_state
import struct

int_16_packer = struct.Struct(">h")
uint_16_packer = struct.Struct(">H")
#uint_32_packer = struct.Struct(">I")

def request_address(node, payload):
    pass


def request_time(node, payload):
    pass


def _unpack_key_value(payload):
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
                    # if value is None, this int was the final token of the stream
                    value = int(remainder)
                    msg = ""
            else:
                # it's a string, split at "End of Text"
                value, msg = remainder.split('\3', 1)
            values[key] = value
    except Exception as e:
        #print(e)
        pass
    return values


def radio_state(node, payload):
    values = _unpack_key_value(payload)
    success = values['s']
    failure = values['f']
    total = success + failure
    percent = 0.00
    if total > 0:
        percent = float(success) / float(total)
    update_radio_state(node, success, failure, percent)


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
    thermometer = str(payload[:16], encoding='utf-8')
    temperature = int_16_packer.unpack(payload[16:])[0]
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
start_listen("LG01", protocols)
