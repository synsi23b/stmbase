import pathlib
#from Cryptodome.Cipher import AES
from AES import KeyExpansion


mode = 128
while mode != 128 and mode != 196 and mode != 256:
  try:
    mode = int(input("Enter AES mode (128, 196, 256): "))
  except Exception as e:
    print(e)

if mode == 128:
  exp_key = 16
  exp_rk = 176
elif mode == 196:
  exp_key = 24
  exp_rk = 208
elif mode == 256:
  exp_key = 32
  exp_rk = 240

key = ""
while len(key) != exp_key:
  key = input(f"Please enter a {exp_key} character key: ")


#cypher = AES.new(bytes(key, encoding='utf-8'), AES.MODE_CBC, bytes([0]*16))

print(f"Got key >> {key} <<")
ordkey = [ord(c) for c in key]
prockey = []
for i in range(int(exp_key / 4)):
  prockey.append([])
  prockey[-1].append(ordkey.pop(0))
  prockey[-1].append(ordkey.pop(0))
  prockey[-1].append(ordkey.pop(0))
  prockey[-1].append(ordkey.pop(0))

expanded = KeyExpansion(prockey)

current_folder = pathlib.Path(__file__).parent.absolute()


def make_keyline(keyblock):
  res = ""
  for x in keyblock:
    res += f"{hex(x)}, "
  return res + '\n'


roundkeys = []
for block in expanded:
  roundkeys.append(make_keyline(block))

lastline = roundkeys[-1]
roundkeys[-1] = lastline[:-3]

with open(current_folder / "roundkey.h", "w") as of:
  of.write(f"// Key: {key}\n")
  of.write(f"static const unsigned char static_roundkey[{exp_rk}] = {{\n")
  of.writelines(roundkeys)
  of.write("\n};")
