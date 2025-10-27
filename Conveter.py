from PIL import Image

def image_to_oled_array(image_path, output_name="myImage", width=128, height=64):
    # Open and convert image
    img = Image.open(image_path).convert('1')  # 1-bit mode
    img = img.resize((width, height), Image.LANCZOS)

    # Get image data as bytes
    pixels = img.tobytes()
    bytes_per_row = width // 8
    data = []

    # Convert pixels into bytes (8 pixels = 1 byte)
    for y in range(height):
        for x_byte in range(bytes_per_row):
            byte = 0
            for bit in range(8):
                x = x_byte * 8 + bit
                pixel = img.getpixel((x, y))
                if pixel == 0:  # black pixel = ON for OLED
                    byte |= (1 << (7 - bit))
            data.append(byte)

    # Format as C array
    hex_data = ", ".join(f"0x{b:02X}" for b in data)
    array_str = f"static const unsigned char PROGMEM {output_name}[] = {{\n  {hex_data}\n}};\n"

    with open(f"{output_name}.h", "w") as f:
        f.write(array_str)

    print(f"✅ Conversion complete! Saved to {output_name}.h")
    print(f"Array size: {len(data)} bytes ({width}x{height})")


# Example usage:
image_to_oled_array("download1.jpg", "zoroFace", 128, 64)
