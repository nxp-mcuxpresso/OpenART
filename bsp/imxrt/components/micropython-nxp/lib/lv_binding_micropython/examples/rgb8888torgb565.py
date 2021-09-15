import numpy as np 

with open('./blue_flower_32.bin','rb') as f:
    color = f.read()
    f.close()

# only need the LSB:24
color_np = np.frombuffer(color, dtype='uint8')
color_np = color_np.reshape(-1,4)[:,:3]

with open('./blue_flower_24.txt', 'wb') as f:
    f.write(color_np.tobytes())
    f.close()

color_16 = []
swap = True
for b,g,r in color_np:
    b5 = (b >> 3) & ((1<<5) - 1)
    g6 = (g >> 2) & ((1<<6) - 1)
    r5 = (r >> 3) & ((1<<5) - 1)
    color_16_pix = (r5 << 11) | (g6 << 5) | b5
    if(swap):
        color_16_pix = ((color_16_pix & 0xff) << 8) | (color_16_pix >> 8) 
    color_16 += [color_16_pix]

color_16_b = np.asarray(color_16, dtype='uint16').tobytes()
with open('./blue_flower_16.txt', 'wb') as f:
    f.write(color_16_b)
    f.close()

