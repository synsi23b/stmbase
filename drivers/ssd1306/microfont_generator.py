from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from PIL import ImageOps
from pathlib import Path

fonts = {} 
base = Path(__file__).absolute().parent.parent
folder = base / "fonts"
for f in folder.glob("*.ttf"):
    name = f.name[:-4]
    size = int(name.split("_")[-1])
    save_name = "_".join(name.split("_")[:-1])
    font = ImageFont.truetype(str(f), size)
    fonts[save_name] = (font, size)
    print(f"Added >>{save_name}<< with size {size}")


img = Image.new('1', (64, 64))
draw = ImageDraw.Draw(img)
clrang = (0, 0, 64, 64)

def make_char_arr(ordinal, font, target):
    char = chr(ordinal)
    draw.rectangle(clrang, 0, 0)
    ex, ey = draw.textsize(char, font)
    draw.text((0, int((target - ey) / 2)), char, 255, font)

    #draw.line([0, target, ex, target, ex, 0], 255, 1)
    #img.save("pixl.png")

    outdata = bytearray()
    curbyte = 0
    mask = 0x01
    for px in range(ex):
        for py in range(target):
            if img.getpixel((px, py)):
                curbyte |= mask
            mask <<= 1
            if mask > 0x80:
                mask = 0x01
                outdata.append(curbyte)
                curbyte = 0
    return outdata


def get_target(font):
    size = font[1]
    for sz in [8, 16, 24, 32]:
        if size <= sz:
            return sz
    raise ValueError("Font to big, need implement!")


def make_arr_val(char):
    outstr = ""
    for b in char:
        outstr += f" {hex(b)},"
    return outstr


def make_font_file(name, height, pages, chars):
    with open(f"{name}_{height}_ssd1306font.cpp", "w") as of:
        of.write(f"#include \"ssd1306.h\"\n")
        of.write(f"// microfont {name} with height {height}\n")
        of.write(f"const uint8_t font_{name}_{height}_bmp[] = {{\n")
        offset = 0
        offsets = []
        for ch in chars:
            of.write(f"  // '{ch[0]}' @ offset {offset}\n")
            of.write(f" {make_arr_val(ch[1])}\n")
            offsets.append((ch[0], offset))
            offset += len(ch[1])
        offsets.append((' ', offset)) # append final length as well
        of.write("};\n\n")
        of.write(f"const uint16_t font_{name}_{height}_off[] = {{\n")
        for v in offsets:
            of.write(f"  {v[1]}, // '{v[0]}'\n")
        of.write("};\n\n")
        of.write(f"const SSD1306::Fontinfo_t font_{name}_{height} = {{\n")
        of.write("  // uint8_t character_height; // in pages\n")
        of.write(f"  {int(pages)},\n")
        of.write("  // table containing the offset into the bitmap for the character\n")
        of.write(f"  font_{name}_{height}_off,\n")
        of.write("  // pixel values directly writable in vertical mode\n")
        of.write("  // setup the characters start and end (offset_table[char + 1])\n")
        of.write(f"  font_{name}_{height}_bmp\n")
        of.write("};\n\n")

for name, font in fonts.items():
    target = get_target(font)
    chars = []
    for o in range(ord(' '), 127):
        dat = make_char_arr(o, font[0], target)
        chars.append((chr(o), dat))
    make_font_file(name, font[1], target / 8, chars)
    