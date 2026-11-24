#!/usr/bin/env python3
"""Generate a simple factory-corridor test map (pgm + yaml) for the simulator and map_server.

Layout (metres): 24 x 8 hall, a long east-west corridor 2.5 m wide at the bottom, two rooms above
joined by door gaps, and a pillar — enough to exercise straight / side / front / line / detour.
Run once:  python3 make_test_map.py <out_dir>
"""
import os
import sys

import numpy as np

try:
    from PIL import Image
except ImportError:
    sys.exit('pip install pillow')

RES = 0.05
W, H = 24.0, 8.0
w, h = int(W / RES), int(H / RES)
img = np.full((h, w), 254, dtype=np.uint8)  # free = 254 (white), occupied = 0


def rect(x0, y0, x1, y1, val=0):
    c0, c1 = int(x0 / RES), int(x1 / RES)
    r0, r1 = int(y0 / RES), int(y1 / RES)
    img[r0:r1, c0:c1] = val


# outer walls (0.2 m thick)
rect(0, 0, W, 0.2)
rect(0, H - 0.2, W, H)
rect(0, 0, 0.2, H)
rect(W - 0.2, 0, W, H)
# corridor ceiling wall at y = 2.7 .. 2.9 with door gaps
rect(0.2, 2.7, W - 0.2, 2.9)
rect(4.0, 2.7, 5.2, 2.9, 254)     # door 1
rect(13.0, 2.7, 14.4, 2.9, 254)   # door 2
rect(20.0, 2.7, 21.6, 2.9, 254)   # door 3
# divider between the two upper rooms
rect(11.9, 2.9, 12.1, H - 0.2)
rect(11.9, 5.0, 12.1, 6.4, 254)   # opening in the divider
# pillar in the corridor and racks in the rooms
rect(8.9, 1.3, 9.3, 1.7)
rect(2.5, 4.5, 6.5, 5.0)
rect(15.0, 4.0, 15.5, 7.0)
rect(17.5, 4.0, 18.0, 7.0)

out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
os.makedirs(out_dir, exist_ok=True)
pgm = os.path.join(out_dir, 'test_hall.pgm')
# pgm row 0 is the top -> flip so map y increases upwards
Image.fromarray(np.flipud(img)).save(pgm)
with open(os.path.join(out_dir, 'test_hall.yaml'), 'w') as f:
    f.write(f"""image: test_hall.pgm
mode: trinary
resolution: {RES}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")
print('wrote', pgm)
