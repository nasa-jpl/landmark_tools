#!/usr/bin/env python3

##
#  \file   `render_ply.py`
#  \author Cecilia Mauceri
#  \brief  Render a ply file using the Blender api
#  usage: render_ply.py file sun_elevation sun_azimuth output -height HEIGHT -width WIDTH [--save-blend-file] [-h]
#  you need to install bpy `pip install bpy`
#  
#  \copyright Copyright 2024 California Institute of Technology
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

import argparse
import bpy
import math
import numpy as np

def configure_scene(
    ply_file: str,
    width: int = 1000,
    height: int = 1000,
    resolution: int = 10,
    save_blend_file: bool = False
):
    """Configure the scene for rendering a ply file. 
    
    Args:
        ply_file (str): Path to the ply file.
        width (int, optional): Width of the render file in pixels. Defaults to 1000.
        height (int, optional): Height of the render file in pixels. Defaults to 1000.
        res (int, optional): Resolution of the render file. Defaults to 10.
        save_blend_file (bool, optional): Save a new blend file for additional renders. Defaults to False.
    """
    # Remove default cube
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.select_by_type(type="MESH")
    bpy.ops.object.delete()

    # Load ply mesh
    bpy.ops.wm.ply_import(filepath=ply_file)
    ob = bpy.context.active_object

    # Change albedo to 14%
    mat = bpy.data.materials.new(name="DEM")
    mat.diffuse_color = (0.14, 0.14, 0.14, 1)
    ob.data.materials.append(mat)

    #Find the lowest vertex
    glob_vertex_coordinates = [ ob.matrix_world @ v.co for v in ob.data.vertices ] # Global coordinates of vertices
    minZ = min( [ co.z for co in glob_vertex_coordinates ] ) 

    # Set camera
    bpy.context.view_layer.objects.active = ob
    camera = bpy.context.scene.camera
    camera.rotation_mode = 'YXZ'
    camera.rotation_euler = (0, 0, 0)
    camera.location = (0, 0, 3000)
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = width*resolution
    camera.data.sensor_fit = "HORIZONTAL"
    bpy.context.scene.render.resolution_x = width
    bpy.context.scene.render.resolution_y = height
    
    clip_margin = 1000
    bpy.context.scene.camera.data.clip_end = camera.location.z - minZ + clip_margin

    # set renderer settings
    bpy.context.scene.render.engine = "CYCLES"
    bpy.context.scene.cycles.device = "GPU"
    bpy.context.scene.cycles.samples = 2048

    # Set output resolution
    scene = bpy.context.scene
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100
    scene.render.use_file_extension = True
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'BW'
    scene.render.image_settings.color_depth = '8'

    # remove all lights
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.select_by_type(type="LIGHT")
    bpy.ops.object.delete()

    # Add sun
    bpy.ops.object.light_add(type="SUN", location=(0, 0, 0))
    sun = bpy.data.objects["SUN".capitalize()]
    sun.data.energy = 10
    sun.data.angle = 0.5*np.pi/180
    sun.data.use_shadow = True

    if save_blend_file:
        bpy.ops.wm.save_as_mainfile(
                filepath=ply_file.replace(".ply", ".blend")
            )

def render_sun_angle(
    sun_elevation: float,
    sun_azimuth: float,
    output_file: str
):
    """Render the scene with the given sun angle.

    Args:
        sun_elevation (float): Sun elevation in degrees.
        sun_azimuth (float): Sun azimuth in degrees.
        output_file (str): Path to the output file.
    """
    # Landmark elevation array has top left origin, which requires the sun azimuth to be mirrored over the horizontal axis for to match
    sun_azimuth = 180 - sun_azimuth
    sun = bpy.data.objects["SUN".capitalize()]
    sun.rotation_euler = (
        math.pi / 2 - math.radians(sun_elevation),
        0,
        math.radians(sun_azimuth),
    )
    bpy.context.scene.render.filepath = (output_file)
    bpy.ops.render.render(write_still=True)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Render given file with blender")
    parser.add_argument("file", help=".ply file defining a DEM mesh in a local reference frame OR a .blend file from a previous run.")
    parser.add_argument("sun_elevation", type=float, help="sun elevation in degrees")
    parser.add_argument("sun_azimuth", type=float, help="sun azimuth in degrees")
    parser.add_argument("output", type=str, help="path to output file of rendered image")
    parser.add_argument("-height", type=int, required=True, help="height of output image in pixels. ")
    parser.add_argument("-width", type=int, required=True, help="width of output image in pixels")
    parser.add_argument("-resolution", type=int, required=True, help="resolution in meters/pixel of output image")
    parser.add_argument(
        "--save-blend-file",
        action="store_true",
        help="Save a new blend file for additional renders. This option is recommended to accelerate additional renders.",
    )
    args = parser.parse_args()
    if(args.file.endswith(".ply")):
        configure_scene(
            args.file,
            width=args.width,
            height=args.height,
            resolution=args.resolution,
            save_blend_file=args.save_blend_file
        )
    elif(args.file.endswith("blend")):
        bpy.ops.wm.open_mainfile(filepath=args.file)
    else:
        print(f"File type not supported: {args.file}, must be a .ply or .blend file")
        exit(1)

    render_sun_angle(
        sun_elevation=args.sun_elevation,
        sun_azimuth=args.sun_azimuth,
        output_file=args.output
    )