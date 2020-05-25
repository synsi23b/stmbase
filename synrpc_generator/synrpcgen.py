import re, os, hashlib, shutil
import argparse
from collections import OrderedDict

SYNRPC_MAX_MSGSIZE = 255

def main():
    parser = argparse.ArgumentParser(description='Generate synrpc messages')
    parser.add_argument('bits', type=str, help="STM8 or STM32")
    args = parser.parse_args()

    if args.bits == "STM32":
        wdir = os.getcwd()
        basepath = wdir + os.sep + ".."
        srcpath = basepath + os.sep + 'src'
        msgpath = basepath + os.sep + 'msg'
        handlerpath = srcpath + os.sep + "synrpc_handlers.cpp"
        headerpath =  srcpath + os.sep + "synrpc_usbcon.h"
        evokerpath =  srcpath + os.sep + "synrpc_usbcon.cpp"
        pybridgehere = wdir + os.sep + "synrpc_usbcon.py"
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
        writePythonBridge(pybridgehere, messages)
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
 * Also, there will be check of the packets checksum before it gets here, An error will be send automatically
 * in case of mismatched messages.
 * Furthermore, returning anything else other than 0 will be regarded as an error and interpreted as
 * a pointer to a zero terminated C-String. This string will be copied into the internal Error-Message buffer.
 * It allows only a maximum size of 57 characters. Anything more will be discarded. You have been warned.
 */
""")
        f.write(userincludes)
        for k, v in messages.items():
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
    with open("synrpc_generator/synrpc_usbcon_template.h", 'r') as f:
        header = f.read()
    header += "\nconst uint16_t MAX_HANDLER_TYPE = {};\n".format(len(messages))
    for k, m in messages.items():
        header += m.genCppHeader()
    header += "\n}\n\n"
    with open(headerpath, "w") as f:
        f.write(header)

def writeEvokerCpp(evokerpath, messages):
    with open("synrpc_generator/synrpc_usbcon_template.cpp", 'r') as f:
        code = f.read()
    for k, m in messages.items():
        code += m.genCppConverter()
    code += "\nconverter_t packetconverters[MAX_HANDLER_TYPE] = {\n"
    for k, m in messages.items():
        code += m.genCppConverterEntry()
    code += """};

const char* handlePacket(const Packet& p){
  uint8_t type = p.type();
  if(type > MAX_HANDLER_TYPE){
    return "Message type unknown, handler type to big";
  }
  else if(type == 0){
    return "Error Message type is ignored";
  }
  type -= 1;
  return packetconverters[type](p);
}
"""
    with open(evokerpath, "w") as f:
        f.write(code)

def writePythonBridge(path, messages):
    with open("synrpc_generator/synrpc_usbcon_template.py", 'r') as f:
        content = f.read()
    for k, m in messages.items():
        content += m.genPyClass()
    content += "\n_msgGenerators = [\n    SynRpcError._unserialize,\n"
    for k, m in messages.items():
        content += m.genPyMsgGen()
    content += "]\n"
    with open(path, 'w') as f:
        f.write(content)

class Variable:
    typepattern = re.compile(r'\b((u?int(8|(16)|(32)))|(bool)|(char))\b')
    arraypattern = re.compile(r'(\w+)\[(\d+)\]')

    # this dictionary translates variable types from the message definiton to cpp and python struct
    # it also gives the size and thus, the allignment requirements
    # TODO actually do alignment checks, currently this code relies on the sanity of the developer
    typedict = {'int8' : ('int8_t', 1, 'b'), 'uint8' : ('uint8_t', 1, 'B'),
                'int16' : ('int16_t', 2, 'h'), 'uint16' : ('uint16_t', 2, 'H'),
                'int32' : ('int32_t', 4, 'l'), 'uint32' : ('uint32_t', 4, 'L'),
                'char' : ('char', 1, 's'), 'bool' : ('bool', 1, '?')
            }

    def __init__(self, typename, varname):
        self.origname = varname
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
            if self.typecpp != 'char':
                return "  static const uint8_t {}_SIZE = {};\n  {} {}[{}_SIZE];\n".format(
                    self.name.upper(), self.arraycount, self.typecpp, self.name, self.name.upper())
            else:
                var = """  static const uint8_t {uname}_SIZE = {size};
  char {name}[{uname}_SIZE];
  void write_{name}(const char* str){{
    strncpy({name}, str, {uname}_SIZE - 1);
    {name}[{uname}_SIZE - 1] = 0;
  }}
"""
                return var.format(name=self.name, size=self.arraycount, uname=self.name.upper())
        return "  {} {};\n".format(self.typecpp, self.name)

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
        return f"{self.typecpp} {self.origname}\n"

class Message:
    typecounter = 1 # type 0 reserved for error messages

    def __init__(self, path, filename):
        self.name = filename.split('.')[0]
        self.msgname = self.name + "Msg"
        self.handlername = self.name + "Handler"
        self.convertername = self.name + "Converter"
        self.mesagedefinition = ""
        self.usercode = ""
        self.vars = []
        shafunc = hashlib.sha1()
        with open(path + os.sep + filename, 'r') as f:
            for index, line in enumerate(f.readlines()):
                self.mesagedefinition += f"* {line}"
                if line[0] == '#':
                    continue
                line = line.strip()
                shafunc.update(line.encode("utf-8"))
                line = line.split()
                try:
                    self.vars.append(Variable(line[0], line[1]))
                except Exception as e:
                    print("Error parsing Message {} on line {}".format(filename, index + 1))
                    print(e)
        self.sha1 = shafunc.hexdigest()[-4:]
        self.size = 0
        self.pysize = 0
        for v in self.vars:
            self.size += v.size()
            self.pysize += v.pysize()
        if self.size > SYNRPC_MAX_MSGSIZE:
            raise RuntimeError(f"Msg: {filename} -> is bigger than 255 bytes.")
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
const char* {}(const Packet& p){{
  return Converter<{}>(&{}, p);
}}
"""
        return conv.format(self.convertername, self.msgname, self.handlername)

    def genCppConverterEntry(self):
        return "&{},\n".format(self.convertername)

    def genCppHandlerStub(self):
        stubhead = """
/* {msgname} message definition
{msgdefin}
*/
const char* syn::{handlername}(const syn::{msgname}& msg){{
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
        cldef = """
class {msgname}(object):
    \"\"\"
Message fields:
{msgfields}
\"\"\"
    _size = {msgsize}
    _type = {msgtype}
    _sha1 = int('{sha1}', 16)
    _header = int('{sha1}', 16) << 16 | {msgtype} << 8 | {msgsize}
    _footer = 255 - {msgsize}
    _packer = struct.Struct("<L{packstr}B")

    def __init__(self, buffer=None):
        if buffer:
            self._data = {msgname}._packer.unpack_from(buffer)
            if self._data[0] != {msgname}._header:
                raise ValueError("header missmatch after parsing of msg type {msgname}")
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


main()