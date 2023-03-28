#!/usr/bin/env python3
import sys
import glob
import imageio

if len(sys.argv) != 3:
    print("Usage: python3 create_gif.py /path/to/extracted/images /path/to/output/gif.gif")
    exit(1)

input_path = sys.argv[1]
output_file = sys.argv[2]

images = sorted(glob.glob(f"{input_path}/*.png"))[:30]
frames = [imageio.imread(image) for image in images]

imageio.mimsave(output_file, frames, format='GIF', duration=0.1)
