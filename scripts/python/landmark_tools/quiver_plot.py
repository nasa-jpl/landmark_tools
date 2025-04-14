
# #
#  \file   `quiver_plot.py`
#  \author Cecilia Mauceri
#  \brief  Visualize DEM patch displacement as a quiver plot
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
#  - Created: 2025-04-11
#  

import numpy as np
import matplotlib.pyplot as plt
from skimage.measure import block_reduce
from visualize_corr import loadAllDisplacementMatrices

def quiver_plot(dx, dy, dz, grid_spacing_m=10, min_displacement_m = 10, patch_size=(10,10), arrow_scale=1):
    # --- Pooling (Mean over patches) ---
    dx_pool = block_reduce(dx, patch_size, np.nanmean)
    dy_pool = block_reduce(dy, patch_size, np.nanmean)
    dz_pool = block_reduce(dz, patch_size, np.nanmean)

    # --- Coordinates of patch centers (in meters) ---
    x_pool, y_pool = np.meshgrid(
        np.arange(dx_pool.shape[1]) * grid_spacing_m * patch_size[1] + grid_spacing_m * patch_size[1] / 2,
        np.arange(dx_pool.shape[0]) * grid_spacing_m * patch_size[0] + grid_spacing_m * patch_size[0] / 2
    )

    # --- Mask small vectors AFTER pooling ---
    magnitude_pool = np.sqrt(dx_pool**2 + dy_pool**2)
    mask = np.logical_and((magnitude_pool >= min_displacement_m), np.logical_not(np.isnan(magnitude_pool)))

    x_masked = x_pool[mask]
    y_masked = y_pool[mask]
    dx_masked = dx_pool[mask]
    dy_masked = dy_pool[mask]
    dz_masked = dz_pool[mask]

    # --- Plot ---
    fig = plt.figure(figsize=(10, 8))

    q = plt.quiver(
        x_masked, y_masked,        # Position in meters
        dx_masked, dy_masked,      # Displacement in meters
        dz_masked,                 # Color mapped to dz
        cmap='viridis',
        scale=arrow_scale,                   # True scale (1 m displacement = 1 m in plot)
        scale_units='xy',
        angles='xy'
    )

    plt.colorbar(q, label='Displacement Z (m)')
    plt.gca().set_aspect('equal')
    plt.gca().invert_yaxis()
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Displacement Field (Mean Pooled over {}x{} pixel patches, \n displacement > {}m only)'.format(patch_size[0], patch_size[1], min_displacement_m))
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    return fig

if __name__=='__main__':
    import argparse
    import glob

    parser = argparse.ArgumentParser(
                    prog='DEMCorrelationQuiverPlot',
                    description='Visualize the output of landmark_comparison using a quiver plot')
    parser.add_argument('prefix')
    parser.add_argument('width', type=int)
    parser.add_argument('height', type=int)
   
    # --- Optional Parameters ---
    parser.add_argument('--grid_spacing_m', type=float, default=10, help="Grid cell spacing in meters")
    parser.add_argument('--min_displacement_m', type=float, default=10, help="Threshold for masking small vectors")
    parser.add_argument('--patch_size', type=float, default=5, help="Patch size for pooling in pixels")
    parser.add_argument('--save_as', type=str, default=None, help="Filepath")
    parser.add_argument('--arrow_scale', type=float, default=1, help="Inverse scale factor for length of arrows")

    args = parser.parse_args()

    displacement_maps = loadAllDisplacementMatrices(args.prefix, args.width, args.height)
    fig = quiver_plot(displacement_maps['dx'], displacement_maps['dy'], displacement_maps['dz'], 
                args.grid_spacing_m, args.min_displacement_m, (args.patch_size, args.patch_size), args.arrow_scale)
    if args.save_as is not None:
        fig.savefig(args.save_as)

    print("All figures have been generated.")
