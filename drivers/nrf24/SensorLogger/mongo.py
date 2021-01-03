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


def insert_radio_state(node, success, failure, percent):
    db = get_db()
    res = db.radio_state.insert_one(
        {
            'stamp': datetime.utcnow(),
            'node': node,
            'success': success,
            'failure': failure,
            'percent': percent
        })
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing insert!")


def insert_message(severity, node, message):
    db = get_db()
    res = db.message.insert_one(
        {
            'stamp': datetime.utcnow(),
            'severity': severity,
            'node': node,
            'message': message
        })
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing insert!")


def insert_key_value(node, values):
    values['node'] = node
    values['stamp'] = datetime.utcnow()
    db = get_db()
    res = db.log.insert_one(values)
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing insert!")
    

def insert_task_status(status, node, description, time_remaining, percent, failed):
    db = get_db()
    res = db.task.insert_one(
        {
            'stamp': datetime.utcnow(),
            'state': status,
            'node': node,
            'description': description,
            'seconds': time_remaining,
            'percent': percent,
            'failed': failed
        })
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing insert!")


def insert_temperature_value(node, thermometer, temperature):
    db = get_db()
    res = db.temperature.insert_one(
        {
            'stamp': datetime.utcnow(),
            'node': node,
            'addr': thermometer,
            'temp': temperature
        })
    if not res.acknowledged:
        print("Error: Mongo DB not acknowledgeing insert!")