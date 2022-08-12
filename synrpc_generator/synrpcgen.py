import re, os, hashlib, shutil
import argparse
from collections import OrderedDict

# transmitting over usb happens in packets of up to 64 byte
# if we send exactly 64 bytes, the host is not
# sure wether or not the transmission is finished.
# it will than ask the again and we need to transmit a zero length packet
# so by defining the max size at 122 and adding the packet descriptor of 5 bytes
# we can finish any transmission in 2 steps and can optimize the device buffer easily
SYNRPC_MAX_MSGSIZE = 127

# check the maximum message size actually needed, minimum is error message (4 byte header + 27 byte data + footer) == 32 byte message
SYNRPC_GEN_MAX = 32

def main():
    parser = argparse.ArgumentParser(description='Generate synrpc messages')
    parser.add_argument('bits', type=str, help="STM8 or STM32")
    args = parser.parse_args()

    if args.bits == "STM32":
        basepath = os.getcwd()
        srcpath = basepath + os.sep + 'src'
        msgpath = basepath + os.sep + 'msg'
        handlerpath = srcpath + os.sep + "synrpc_handlers.cpp"
        headerpath =  srcpath + os.sep + "synrpc_usbcon.h"
        evokerpath =  srcpath + os.sep + "synrpc_usbcon.cpp"
        pybridgepath = basepath + os.sep + "synrpc_usbcon.py"
        files = next(os.walk(msgpath))[2]
        messages = OrderedDict()
        for f in files:
            print(f"Generating Message: {f}")
            m = Message(msgpath, f)
            messages[m.msgname] = m
        if os.path.isfile(handlerpath):
            shutil.copy(handlerpath, handlerpath + ".back")
        print("Writing new source files")
        writeHandlerCpp(handlerpath, messages)
        writeHeaderCpp(headerpath, messages)
        writeEvokerCpp(evokerpath, messages)
        #writePythonBridge(pybridgehere, messages)
        writePythonBridge(pybridgepath, messages)

def writeHandlerCpp(handlerpath, messages):
    userincludes = "/// USER INCLUDES ///\n/// USER INCLUDES END ///\n"
    if os.path.isfile(handlerpath):
        with open(handlerpath, 'r') as f:
            lines = f.read().splitlines()
            userincludes = getUserIncludes(lines)
            getUserCode(lines, messages)
    with open(handlerpath, 'w') as f:
        f.write("""
#include "synrpc_usbcon.h"\n
/* Here come the handler definitions. They will be evoked with the message in an internal buffer
 * and on the PacketHandler Thread, thus any RTOS function calls are allowed.
 * The packet reception continues in the background to another buffer. If you stall too long, it
 * will start to stall the Host.
 * Since the programmer is responsible for keeping align requirements, please be careful!
 * Also, there will be a check of the packets checksum before it gets here, An error will be send automatically
 * in case of mismatched messages.
 * Furthermore, returning anything else other than 0 will be regarded as an error and interpreted as
 * a pointer to a zero terminated C-String. This string will be copied into the internal Error-Message buffer.
 * It allows only a maximum size of 26 characters. Anything more will be discarded. You have been warned.
 */
""")
        f.write(userincludes)
        for v in messages.values():
            f.write(v.genCppHandlerStub())

def getUserIncludes(lines):
    # find the beginning of user includes
    inclstart = 0
    inclend = 0
    lineidx = 0
    for line in lines:
        if line == "/// USER INCLUDES ///":
            inclstart = lineidx
        if line == "/// USER INCLUDES END ///":
            inclend = lineidx + 2
        lineidx += 1
    res = '\n'.join(lines[inclstart:inclend])
    print(f"\nFound {inclend - inclstart - 3} user includes: \n{res}")
    return res

def getUserCode(lines, messages):
    startpattern = re.compile(r'^\/\/\/ USER CODE (\w+) \/\/\/$')
    endpattern = re.compile(r'^\/\/\/ END USER CODE \/\/\/$')
    currentMsg = None
    currcode = ""
    for line in lines:
        if currentMsg is None:
            startmatch = re.match(startpattern, line)
            if startmatch:
                msgname = startmatch.group(1)
                if msgname in messages:
                    print(f"Found user code for message: {msgname}")
                    currentMsg = messages[msgname]
                else:
                    print(f"Warning! Found user code for unregistered message: {msgname}")
        else:
            if re.match(endpattern, line):
                currentMsg.addUserCodeCpp(currcode)
                currcode = ""
                currentMsg = None
            elif re.match(startpattern, line):
                raise RuntimeError(f"Found unterminated user code section for message: {msgname}")
            else:
                currcode = currcode + line + "\n"

def writeHeaderCpp(headerpath, messages):
    with open("stmbase/synrpc_generator/synrpc_usbcon_template.h", 'r') as f:
        header = f.read().format(SYNRPC_USBCON_MAX_GEN=SYNRPC_GEN_MAX)
    header += "\nconst uint8_t MAX_HANDLER_TYPE = {};\n".format(len(messages))
    for m in messages.values():
        header += m.genCppHeader()
    header += "\n}\n\n"
    with open(headerpath, "w") as f:
        f.write(header)

def writeEvokerCpp(evokerpath, messages):
    with open("stmbase/synrpc_generator/synrpc_usbcon_template.cpp", 'r') as f:
        code = f.read()
    for m in messages.values():
        code += m.genCppConverter()
        code += m.genCppChecker()
    code += "\nconverter_t packetconverters[MAX_HANDLER_TYPE] = {\n"
    for m in messages.values():
        code += m.genCppConverterEntry()
    code += "};\n"
    code += "\nchecker_t packetcheckers[MAX_HANDLER_TYPE] = {\n"
    for m in messages.values():
        code += m.genCppCheckerEntry()
    code += "};\n"
    code +="""
const char* handlePacket(const UsbRpc::Packet& p){
  uint8_t type = p.type();
  if(type > MAX_HANDLER_TYPE){
    return "msg type to big";
  }
  else if(type == 0){
    return "msg type is ignored";
  }
  type -= 1;
  return packetconverters[type](p);
}

uint16_t UsbRpc::Handler::plausible(const UsbRpc::Packet &p)
{
  uint8_t type = p.type();
  if (type > MAX_HANDLER_TYPE)
  {
    return 0;
  }
  else if (type == 0)
  {
    return 0;
  }
  type -= 1;
  return packetcheckers[type](p);
}
"""
    with open(evokerpath, "w") as f:
        f.write(code)

def writePythonBridge(path, messages):
    with open("stmbase/synrpc_generator/synrpc_usbcon_template.py", 'r') as f:
        content = f.read()
    for m in messages.values():
        content += m.genPyClass()
    content += "\n_msgGenerators = [\n    SynRpcError._unserialize,\n"
    for m in messages.values():
        content += m.genPyMsgGen()
    content += "]\n\n_msgNames = [\n    'SynRpcError',\n"
    for m in messages.values():
        content += m.genPyMsgName()
    content += "]\n"
    with open(path, 'w') as f:
        f.write(content)

class Variable:
    typepattern = re.compile(r'\b((u?int(8|(16)|(32)|(64)))|(bool)|(float)|(double)|(char))\b')
    arraypattern = re.compile(r'(\w+)\[(\d+)\]')

    # this dictionary translates variable types from the message definiton to cpp and python struct
    # it also gives the size and thus, the allignment requirements
    # TODO actually do alignment checks, currently this code relies on the sanity of the developer
    typedict = {'int8' : ('int8_t', 1, 'b'), 'uint8' : ('uint8_t', 1, 'B'),
                'int16' : ('int16_t', 2, 'h'), 'uint16' : ('uint16_t', 2, 'H'),
                'int32' : ('int32_t', 4, 'l'), 'uint32' : ('uint32_t', 4, 'L'),
                'int64' : ('int64_t', 4, 'q'), 'uint64' : ('uint64_t', 4, 'Q'),
                # For the 'f' and 'd' conversion codes, the packed representation uses
                # the IEEE 754 binary32 (for 'f') or binary64 (for 'd') format,
                # regardless of the floating-point format used by the platform.
                'float' : ('float', 4, 'f'), 'double' : ('double', 8, 'd'),
                'char' : ('char', 1, 's'), 'bool' : ('bool', 1, '?')
            }

    def __init__(self, typename, varname, comments):
        self.origname = varname
        self.comments = comments
        self.arraycount = 0
        self.name = ""
        if not re.match(Variable.typepattern, typename):
            raise ValueError("Could not determine type of Variable: {} {}".format(typename, varname))
        self.typecpp = Variable.typedict[typename][0]
        self.sizecpp = Variable.typedict[typename][1]
        self.pypacks = Variable.typedict[typename][2]
        if re.match(Variable.arraypattern, varname):
            strsplit = varname.split('[')
            self.name = strsplit[0].lower()
            self.arraycount = int((strsplit[1]).split(']')[0])
            if self.arraycount == 0:
                raise ValueError("Arrays with length 0 are forbidden!")
        else:
            self.name = varname.lower()

    def size(self):
        return self.sizecpp * self.arraycount if self.arraycount else self.sizecpp

    def pysize(self):
        if self.typecpp == 'char': # save strings as python strings not python arrays. struct module does it all for us
            return 1
        return self.arraycount if self.arraycount else 1 
    
    def genCppVardef(self):
        if self.arraycount:
            cppstr = f"  static const uint8_t {self.name.upper()}_SIZE = {self.arraycount};\n"
            for c in self.comments:
                cppstr += f"  //{c}\n"
            cppstr += f"  {self.typecpp} {self.name}[{self.name.upper()}_SIZE];\n"
            if self.typecpp == 'char':
                cppstr += f"  void write_{self.name}(const char* str) {{\n"
                cppstr += f"    strncpy({self.name}, str, {self.name.upper()}_SIZE - 1);\n"
                cppstr += f"    {self.name}[{self.name.upper()}_SIZE - 1] = 0;\n  }}"
        else:
            cppstr = ""
            for c in self.comments:
                cppstr += f"  //{c}\n"
            cppstr += f"  {self.typecpp} {self.name};\n"
        return cppstr

    def genPyPackstr(self):
        return str(self.arraycount) + self.pypacks if self.arraycount else self.pypacks

    def genPyAccess(self, start):
        if self.typecpp == 'char':
            fndef = """
    @property
    def {name}(self):
        return self._data[{start}]

    @{name}.setter
    def {name}(self, value):
        self._data[{start}] = value[:{size}]
"""
            return fndef.format(name=self.name, start=start, size=self.arraycount - 1)
        if self.arraycount:
            end = start + self.arraycount
            fndef = """
    @property
    def {name}(self):
        return _Array(self._data, {start}, {end})

    @{name}.setter
    def {name}(self, value):
        if len(value) == {size}:
            idx = {start}
            for v in value:
                self._data[idx] = v
                idx += 1
        else:
            raise ValueError("Attempt to overwrite array with wrong length: {{}} Expected length: {size}".format(len(value)))
"""
            return fndef.format(name=self.name, start=start, end=end, size=self.arraycount)
        fndef = """
    @property
    def {name}(self):
        return self._data[{start}]

    @{name}.setter
    def {name}(self, value):
        self._data[{start}] = value
"""
        return fndef.format(name=self.name, start=start)
    
    def genPyInit(self):
        if self.typecpp == 'char':
            return " + ['']"
        if self.arraycount:
            return " + [0] * {}".format(self.arraycount)
        return " + [0]"

    def genPyFieldDsc(self):
        return '\n'.join(self.comments) + f"\n{self.typecpp} {self.origname}\n"

class Message:
    typecounter = 1 # type 0 reserved for error messages

    def __init__(self, path, filename):
        global SYNRPC_GEN_MAX
        self.name = filename.split('.')[0]
        self.msgname = self.name + "Msg"
        self.handlername = self.name + "Handler"
        self.convertername = self.name + "Converter"
        self.checkername = self.name + "Checker"
        self.mesagedefinition = ""
        self.usercode = ""
        self.vars = []
        shafunc = hashlib.sha1()
        with open(path + os.sep + filename, 'r') as f:
            comments = []
            for index, line in enumerate(f.readlines()):
                self.mesagedefinition += f"* {line}"
                if line[0] == '#':
                    comments.append(line.strip()[1:])
                    continue
                line = line.strip()
                # we calculate the sha-sum only over the varables, not the comments
                shafunc.update(line.encode("utf-8"))
                line = line.split()
                try:
                    self.vars.append(Variable(line[0], line[1], comments))
                    comments = []
                except Exception as e:
                    print("Error parsing Message {} on line {}".format(filename, index + 1))
                    print(e)
        self.sha1 = shafunc.hexdigest()[-4:]
        self.size = 0
        self.pysize = 0
        for v in self.vars:
            self.size += v.size()
            self.pysize += v.pysize()
        self.size += 5 # add meta data
        if self.size > SYNRPC_MAX_MSGSIZE:
            raise RuntimeError(f"Msg: {filename} -> is bigger than {SYNRPC_MAX_MSGSIZE} bytes.")
        if self.size < 6:
            raise RuntimeError(f"Msg: {filename} -> Contains no Variables?")
        SYNRPC_GEN_MAX = max(self.size, SYNRPC_GEN_MAX)
        self.type = Message.typecounter
        Message.typecounter += 1

    def addUserCodeCpp(self, code):
        self.usercode = code
        
    def genCppHeader(self):
        msghead = """
struct {}{{
  static const uint8_t _size = {};
  static const uint8_t _type = {};
  static const uint16_t _sha1 = 0x{};
  uint32_t _packetstart;
"""
        msghead = msghead.format(self.msgname, self.size, self.type, self.sha1)
        msgfoot = """  uint8_t _packetend;
}};
const char* {}(const {}& msg);
"""
        msgfoot = msgfoot.format(self.handlername, self.msgname)
        msgbody = ""
        for v in self.vars:
            msgbody += v.genCppVardef()
        return msghead + msgbody + msgfoot

    def genCppConverter(self):
        conv = """
const char* {}(const UsbRpc::Packet& p){{
  return Converter<{}>(&{}, p);
}}
"""
        return conv.format(self.convertername, self.msgname, self.handlername)

    def genCppConverterEntry(self):
        return "&{},\n".format(self.convertername)

    def genCppChecker(self):
        check = """
uint16_t {}(const UsbRpc::Packet& p){{
  return Checker<{}>(p);
}}
"""
        return check.format(self.checkername, self.msgname)

    def genCppCheckerEntry(self):
        return "&{},\n".format(self.checkername)

    def genCppHandlerStub(self):
        stubhead = """
/* {msgname} message definition
{msgdefin}
*/
const char* syn::{handlername}(const syn::{msgname}& msg){{
  (void)msg; // avoid "unused variable" compiler warning
/// USER CODE {msgname} ///
"""
        stubhead = stubhead.format(msgname=self.msgname, handlername=self.handlername, msgdefin=self.mesagedefinition)
        stubfoot = "/// END USER CODE ///\n  return 0;\n}\n\n"
        return stubhead + self.usercode + stubfoot

    def genPyClass(self):
        position = 1
        access = ""
        packstr = ""
        pyinit = ""
        msgfields = ""
        for v in self.vars:
            access += v.genPyAccess(position)
            packstr += v.genPyPackstr()
            pyinit += v.genPyInit()
            position += v.pysize()
            msgfields += v.genPyFieldDsc()
        cldef = """\n\n
class {msgname}(object):
    \"\"\"
Message fields:

{msgfields}
\"\"\"
    _size = {msgsize}
    _type = {msgtype}
    _sha1 = int('{sha1}', 16)
    _header = int('{sha1}', 16) << 16 | {msgtype} << 8 | {msgsize}
    _footer = SYNRPC_MAX_MSGSIZE - {msgsize}
    _packer = struct.Struct("<L{packstr}B")

    def __init__(self, buffer=None):
        if buffer:
            self._data = {msgname}._packer.unpack_from(buffer)
            if self._data[0] != {msgname}._header:
                raise ValueError("header missmatch for {msgname}")
        else:
            self._data = [{msgname}._header] {pyinit} + [{msgname}._footer]

    def _serialize(self):
        return {msgname}._packer.pack(*self._data)
    
    @staticmethod
    def _unserialize(buffer):
        return {msgname}(buffer)
"""
        cldef = cldef.format(msgname=self.msgname, msgsize=self.size, msgtype=self.type, 
                            sha1=self.sha1, packstr=packstr, pyinit=pyinit, msgfields=msgfields)
        cldef += access
        return cldef

    def genPyMsgGen(self):
        return "    {}._unserialize,\n".format(self.msgname)

    def genPyMsgName(self):
        return "    '{}',\n".format(self.msgname)


main()