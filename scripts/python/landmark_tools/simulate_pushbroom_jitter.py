
# #
#  \file   `simulate_pushbroom_jitter.py`
#  \author Cecilia Mauceri
#  \brief  Warp image as to simulated disrupted scanlines
#   
#  \copyright Copyright 2024 California Institute of Technology
#
#  Portions of this code were generated with assistance from ChatGPT, a language model provided by OpenAI
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#  
#  \section updates Update History
#  - Created: 2025-03-26
#  

import argparse
import numpy as np
from skimage import io
from skimage.transform import warp

def simulate_pushbroom_jitter(image, angle_deg, distance, num_disrupted):
    """
    image         : Numpy array (H x W x C) or (H x W)
    angle_deg     : Angle of the disrupted scanline relative to the x-axis (in degrees)
    distance      : Nearest distance from the origin to the center scanline
                    (i.e., the line is offset from (0,0) by 'distance' in the normal direction)
    num_disrupted : Width (in pixels) of the disrupted region (measured perpendicular to that center line)

    Returns a warped (jittered) image as a Numpy array of the same shape.
    """

    angle_rad = np.deg2rad(angle_deg)

    # Direction vector of the scanline
    d_x = np.cos(angle_rad)
    d_y = np.sin(angle_rad)

    # Normal vector (n) to that line (offset from origin by 'distance')
    n_x = -np.sin(angle_rad)
    n_y =  np.cos(angle_rad)

    half_width = num_disrupted / 2.0

    def transform(coords):
        """
        coords: Nx2 array of (row, col) coordinate pairs
        """

        rr = coords[:, 0]  # row -> y
        cc = coords[:, 1]  # col -> x

        # Distance between scanline and coordinate
        dist_perp = (cc * n_x + rr * n_y) - distance

        # How far along the scanline the coordinate lies
        dist_line = (cc * d_x + rr * d_y)

        # Scale factor in the disrupted region
        scale = np.ones_like(rr)
        inside_mask = np.abs(dist_perp) < half_width
        scale[inside_mask] = 0.5 + 0.5 * (np.abs(dist_perp[inside_mask]) / half_width)

        # Reconstruct final normal coordinate => distance + compressed offset
        final_n = scale * dist_perp + distance

        # Convert back to (x, y)
        new_x = final_n * n_x + dist_line * d_x
        new_y = final_n * n_y + dist_line * d_y

        return np.vstack([new_y, new_x]).T

    jittered = warp(
        image,
        inverse_map=transform,
        mode='edge',
        preserve_range=True
    )

    return jittered.astype(image.dtype)

def main():
    parser = argparse.ArgumentParser(
        description="Simulate pushbroom image jitter for a given angle, distance, and number of disrupted scanlines."
    )
    parser.add_argument("input_image", help="Path to the input image file.")
    parser.add_argument("output_image", help="Path to save the output jittered image.")
    parser.add_argument("--angle", type=float, default=0.0, 
                        help="Angle of the disrupted scanline relative to the x-axis (degrees).")
    parser.add_argument("--distance", type=float, default=50.0,
                        help="Nearest distance from origin to the center scanline (pixels).")
    parser.add_argument("--num_disrupted", type=int, default=50,
                        help="Number of disrupted scanlines (width of region).")
    args = parser.parse_args()

    image = io.imread(args.input_image)

    jittered_image = simulate_pushbroom_jitter(
        image,
        angle_deg=args.angle,
        distance=args.distance,
        num_disrupted=args.num_disrupted
    )

    io.imsave(args.output_image, jittered_image)

if __name__ == "__main__":
    main()
