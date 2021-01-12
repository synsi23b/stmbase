from datetime import datetime
import pymongo
import json
import pathlib

# global DB object
_db = None
#MONGO_USER = config('MONGO_USER', default="", cast=str)
#MONGO_PASS = config('MONGO_PASS', default="", cast=str)
#MONGO_URI = config('MONGO_URI', default="", cast=str)
#MONGO_NAME = config('MONGO_NAME', default="", cast=str)


def get_db():
    """populates the global DB if not yet initialized

    Returns:
      MongoDB -- the connected DB
    """
    global _db
    if _db == None:
        this_folder = pathlib.Path(__file__).parent.absolute()
        with open(this_folder / '.mongo_creds.json') as f:
            creds = json.load(f)
            MONGO_USER = creds['user']
            MONGO_PASS = creds['pass']
            MONGO_URL = creds['url']
            DB_URI = f"mongodb+srv://{MONGO_USER}:{MONGO_PASS}@{MONGO_URL}?retryWrites=true&w=majority"
        _db = pymongo.MongoClient(DB_URI)["RF24Logger"]
    return _db


def update_radio_state(node, success, failure, percent, out_overflow):
    mfilter = {
        'node': node
    }
    mupdate = {
        '$currentDate': {'radio_status_stamp': True},
        '$set': {'percent': percent},
        '$inc': {
            'packet_send_success': success,
            'packet_send_failure': failure,
            'outbox_overflow': out_overflow
        }
    }
    db = get_db()
    res = db.radios.update_one(mfilter, mupdate, upsert=True)
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing radio state update!")
        print(mupdate)


def insert_message(severity, node, message):
    db = get_db()
    data = {
        'stamp': datetime.utcnow(),
        'severity': severity,
        'node': node,
        'message': message
    }
    res = db.messages.insert_one(data)
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing message insert!")
        print(data)


def insert_key_value(node, values):
    values['node'] = node
    values['stamp'] = datetime.utcnow()
    db = get_db()
    res = db.logs.insert_one(values)
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing log insert!")
        print(values)


def insert_task_status(status, node, description, time_remaining, percent, failed):
    db = get_db()
    data = {
        'stamp': datetime.utcnow(),
        'state': status,
        'node': node,
        'description': description,
        'seconds': time_remaining,
        'percent': percent,
        'failed': failed
    }
    res = db.tasks.insert_one(data)
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing task insert!")
        print(data)


def insert_temperature_value(node, thermometer, temperature):
    db = get_db()
    data = {
        'stamp': datetime.utcnow(),
        'node': node,
        'addr': thermometer,
        'temp': temperature
    }
    res = db.temperatures.insert_one(data)
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing temperature insert!")
        print(data)
