import struct, serial, threading
from time import sleep

# transmitting over usb happens in packets of up to 64 byte
# if we send exactly 64 bytes, the host is not
# sure wether or not the transmission is finished.
# it will than ask the again and we need to transmit a zero length packet
# so by defining the max size at 122 and adding the packet descriptor of 5 bytes
# we can finish any transmission in 2 steps and can optimize the device buffer easily
SYNRPC_MAX_MSGSIZE = 122

class _Array(object):
    def __init__(self, parentlist, start, end):
        self._parent = parentlist
        self._start = start
        self._end = end

    def __len__(self):
        return self._end - self._start

    def __getitem__(self, key):
        return self._parent[self._getParentIdx(key)]
        
    def __setitem__(self, key, val):
        self._parent[self._getParentIdx(key)] = val

    def _getParentIdx(self, key):
        if key >= 0:
            key = self._start + key
            if key >= self._end:
                raise IndexError()
        else:
            key = self._end + key # key is negativ, iterate reverse
            if key < self._start:
                raise IndexError()
        return key

    def __iter__(self):
        return _Array._Iter(self._parent, self._start, self._end)

    def __reversed__(self):
        return _Array._Iter(self._parent, self._start, self._end)

    class _Iter:
        def __init__(self, parent, start, stop):
            self._parent = parent
            self._current = start
            self._stop = stop

        def __next__(self):
            if self._current < self._stop:
                val = self._current
                self._current += 1
                return self._parent[val]
            else:
                raise StopIteration()

    class _RevIter:
        def __init__(self, parent, start, stop):
            self._parent = parent
            self._current = stop - 1
            self._stop = start

        def __next__(self):
            if self._current >= self._stop:
                val = self._current
                self._current -= 1
                return self._parent[val]
            else:
                raise StopIteration()

class PacketHandler:
    """Receive and send SynRPCMessages using this handler.
example usage ->
mymsg = BarFooMsg()
mymsg.bla = int('7AAAAAAA', 16)
mymsg.nignog = int('BBBB', 16)
mymsg.data = [int('CC', 16)] * 10

def barfoohhandler(msg):
    print hex(msg.bla), hex(msg.nignog), st

ph = PacketHandler("COM6")
ph.addMessageHandler(BarFooMsg, barfoohandler)
ph.start()

while True:
    ph.sendMessage(mymsg)
    sleep(1)
"""
    def __init__(self, comport):
        self._comport = comport
        self._serial = serial.Serial(comport)
        self._handlers = { 0 : PacketHandler._errmsgHandler }
        self._running = False
        self._recthread = threading.Thread(target=self._receiver)

    def start(self):
        self._running = True
        self._recthread.start()

    def shutdown(self):
        self._running = False
        self._recthread.join(2.0)
        self._serial = None

    def addMessageHandler(self, MsgClass, handler):
        self._handlers[MsgClass._type] = handler

    def sendMessage(self, message):
        self._serial.write(message._serialize())

    def _receiver(self):
        buffer = []
        while self._running:
            inavail = self._serial.in_waiting
            if not inavail:
                if not buffer: # empty
                    sleep(0.001)
                    continue
            else:
                buffer += self._serial.read(inavail)
            # check buffer 0 for expected packet size and if the buffer is big enough
            plsize = buffer[0]
            if len(buffer) >= plsize + 5:
                # check if end of packet matches plsize
                plsrev = buffer[plsize + 4]
                if (SYNRPC_MAX_MSGSIZE - plsrev) == plsize:
                    try: # valid packet sizes, check content
                        #msgs = ''.join(buffer[:plsize + 5])
                        msg = _generateMessage(buffer[1], buffer)
                        if msg:
                            try:
                                buffer = buffer[msg._size + 5:]
                                if msg._type in self._handlers:
                                    self._handlers[msg._type](msg)
                                else:
                                    print(f"Warning -> Missing handler for message: {type(msg)}")
                            except Exception as e:
                                print(e)
                    except ValueError as e:
                        print(e)
                        buffer.pop(0)
                    except struct.error as e:
                        print(e, f" - buffer actual length: {len(buffer)}")
                else: # data missmatch, purge first byte and try again
                    buffer.pop(0)
            else: # not enough data in yet, sleep a bit
                # probably never happens because of lots and lots of queing in the os driver and all that
                sleep(0.01)
                pass

    @staticmethod
    def _errmsgHandler(synrpcerror):
        print(synrpcerror)

def _generateMessage(msgtype, buffer):
    if msgtype < len(_msgGenerators):
        return _msgGenerators[msgtype](bytes(buffer))
    return None

class SynRpcError(object):
    _type = 0
    _size = 27
    _header = int('ffff', 16) << 16 | 0 << 8 | 27
    _footer = SYNRPC_MAX_MSGSIZE - 27
    _packer = struct.Struct("<LB26sB")

    def __init__(self, buffer):
        data = SynRpcError._packer.unpack_from(buffer)
        if data[0] != SynRpcError._header:
            raise ValueError("header missmatch after parsing of msg type SynRpcError")
        self.errtype = data[1]
        self.errmsg = data[2]

    def __str__(self):
        return "Type: {} Error: {}".format(self.errtype, self.errmsg)

    @staticmethod
    def _unserialize(buffer):
        return SynRpcError(buffer)

class PulseReportMsg(object):
    """
Message fields:
uint16_t width[8]
uint16_t t_on[8]

"""
    _size = 32
    _type = 1
    _sha1 = int('c5e4', 16)
    _header = int('c5e4', 16) << 16 | 1 << 8 | 32
    _footer = SYNRPC_MAX_MSGSIZE - 32
    _packer = struct.Struct("<L8H8HB")

    def __init__(self, buffer=None):
        if buffer:
            self._data = PulseReportMsg._packer.unpack_from(buffer)
            if self._data[0] != PulseReportMsg._header:
                raise ValueError("header missmatch after parsing of msg type PulseReportMsg")
        else:
            self._data = [PulseReportMsg._header]  + [0] * 8 + [0] * 8 + [PulseReportMsg._footer]

    def _serialize(self):
        return PulseReportMsg._packer.pack(*self._data)
    
    @staticmethod
    def _unserialize(buffer):
        return PulseReportMsg(buffer)

    @property
    def width(self):
        return _Array(self._data, 1, 9)

    @width.setter
    def width(self, value):
        if len(value) == 8:
            idx = 1
            for v in value:
                self._data[idx] = v
                idx += 1
        else:
            raise ValueError("Attempt to overwrite array with wrong length: {} Expected length: 8".format(len(value)))

    @property
    def t_on(self):
        return _Array(self._data, 9, 17)

    @t_on.setter
    def t_on(self, value):
        if len(value) == 8:
            idx = 9
            for v in value:
                self._data[idx] = v
                idx += 1
        else:
            raise ValueError("Attempt to overwrite array with wrong length: {} Expected length: 8".format(len(value)))

class SetPotentiometerMsg(object):
    """
Message fields:
uint8_t values[122]

"""
    _size = 122
    _type = 2
    _sha1 = int('baa1', 16)
    _header = int('baa1', 16) << 16 | 2 << 8 | 122
    _footer = SYNRPC_MAX_MSGSIZE - 122
    _packer = struct.Struct("<L122BB")

    def __init__(self, buffer=None):
        if buffer:
            self._data = SetPotentiometerMsg._packer.unpack_from(buffer)
            if self._data[0] != SetPotentiometerMsg._header:
                raise ValueError("header missmatch after parsing of msg type SetPotentiometerMsg")
        else:
            self._data = [SetPotentiometerMsg._header]  + [0] * 122 + [SetPotentiometerMsg._footer]

    def _serialize(self):
        return SetPotentiometerMsg._packer.pack(*self._data)
    
    @staticmethod
    def _unserialize(buffer):
        return SetPotentiometerMsg(buffer)

    @property
    def values(self):
        return _Array(self._data, 1, 123)

    @values.setter
    def values(self, value):
        if len(value) == 122:
            idx = 1
            for v in value:
                self._data[idx] = v
                idx += 1
        else:
            raise ValueError("Attempt to overwrite array with wrong length: {} Expected length: 122".format(len(value)))

class SetTestTimerMsg(object):
    """
Message fields:
uint16_t frequencies[2]
uint16_t values[8]

"""
    _size = 20
    _type = 3
    _sha1 = int('674d', 16)
    _header = int('674d', 16) << 16 | 3 << 8 | 20
    _footer = SYNRPC_MAX_MSGSIZE - 20
    _packer = struct.Struct("<L2H8HB")

    def __init__(self, buffer=None):
        if buffer:
            self._data = SetTestTimerMsg._packer.unpack_from(buffer)
            if self._data[0] != SetTestTimerMsg._header:
                raise ValueError("header missmatch after parsing of msg type SetTestTimerMsg")
        else:
            self._data = [SetTestTimerMsg._header]  + [0] * 2 + [0] * 8 + [SetTestTimerMsg._footer]

    def _serialize(self):
        return SetTestTimerMsg._packer.pack(*self._data)
    
    @staticmethod
    def _unserialize(buffer):
        return SetTestTimerMsg(buffer)

    @property
    def frequencies(self):
        return _Array(self._data, 1, 3)

    @frequencies.setter
    def frequencies(self, value):
        if len(value) == 2:
            idx = 1
            for v in value:
                self._data[idx] = v
                idx += 1
        else:
            raise ValueError("Attempt to overwrite array with wrong length: {} Expected length: 2".format(len(value)))

    @property
    def values(self):
        return _Array(self._data, 3, 11)

    @values.setter
    def values(self, value):
        if len(value) == 8:
            idx = 3
            for v in value:
                self._data[idx] = v
                idx += 1
        else:
            raise ValueError("Attempt to overwrite array with wrong length: {} Expected length: 8".format(len(value)))

class VoltageReportMsg(object):
    """
Message fields:
uint16_t voltages[10]

"""
    _size = 20
    _type = 4
    _sha1 = int('54a7', 16)
    _header = int('54a7', 16) << 16 | 4 << 8 | 20
    _footer = SYNRPC_MAX_MSGSIZE - 20
    _packer = struct.Struct("<L10HB")

    def __init__(self, buffer=None):
        if buffer:
            self._data = VoltageReportMsg._packer.unpack_from(buffer)
            if self._data[0] != VoltageReportMsg._header:
                raise ValueError("header missmatch after parsing of msg type VoltageReportMsg")
        else:
            self._data = [VoltageReportMsg._header]  + [0] * 10 + [VoltageReportMsg._footer]

    def _serialize(self):
        return VoltageReportMsg._packer.pack(*self._data)
    
    @staticmethod
    def _unserialize(buffer):
        return VoltageReportMsg(buffer)

    @property
    def voltages(self):
        return _Array(self._data, 1, 11)

    @voltages.setter
    def voltages(self, value):
        if len(value) == 10:
            idx = 1
            for v in value:
                self._data[idx] = v
                idx += 1
        else:
            raise ValueError("Attempt to overwrite array with wrong length: {} Expected length: 10".format(len(value)))

_msgGenerators = [
    SynRpcError._unserialize,
    PulseReportMsg._unserialize,
    SetPotentiometerMsg._unserialize,
    SetTestTimerMsg._unserialize,
    VoltageReportMsg._unserialize,
]
