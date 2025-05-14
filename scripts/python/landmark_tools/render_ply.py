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
import os
from enum import Enum

class ShaderType(Enum):
    LUNAR_LAMBERT = "lunar_lambert"
    DEFAULT = "default"


def configure_scene(
    ply_file: str,
    width: int = 1000,
    height: int = 1000,
    resolution: float = 10,
    albedo: float = 0.14,
    save_blend_file: bool = False,
    shader_type: ShaderType = ShaderType.DEFAULT
):
    """Configure the scene for rendering a ply file. 
    
    Args:
        ply_file (str): Path to the ply file.
        width (int, optional): Width of the render file in pixels. Defaults to 1000.
        height (int, optional): Height of the render file in pixels. Defaults to 1000.
        res (float, optional): Resolution of the render file. Defaults to 10.
        albedo (float): Surface reflectivity as decimal. Defaults to 0.14
        save_blend_file (bool, optional): Save a new blend file for additional renders. Defaults to False.
        shader_type (ShaderType, optional): Type of shader to use. Defaults to ShaderType.DEFAULT.
    """
    # Remove default cube
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.select_by_type(type="MESH")
    bpy.ops.object.delete()

    # Load ply mesh
    bpy.ops.wm.ply_import(filepath=ply_file)
    ob = bpy.context.active_object

    # Create material
    mat = bpy.data.materials.new(name="DEM")
    
    if shader_type != ShaderType.DEFAULT:
        # Enable OSL and set renderer settings
        bpy.context.scene.render.engine = 'CYCLES'
        bpy.context.scene.cycles.shading_system = True
        # OSL shaders don't support GPU rendering
        bpy.context.scene.cycles.device = "CPU"
        bpy.context.scene.cycles.samples = 2048
        
        # Set color management to preserve raw values
        bpy.context.scene.view_settings.view_transform = 'Raw'
        bpy.context.scene.view_settings.look = 'None'
        bpy.context.scene.display_settings.display_device = 'sRGB'
        
        # Create OSL node
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        nodes.clear()
        
        # Add OSL script
        script_path = os.path.join(os.path.dirname(__file__), f"{shader_type.value}.osl")
        if not os.path.exists(script_path):
            raise FileNotFoundError(f"OSL shader not found at {script_path}")
            
        # Create text datablock
        if shader_type.value not in bpy.data.texts:
            with open(script_path, 'r') as f:
                text = bpy.data.texts.new(name=shader_type.value)
                text.write(f.read())
        
        # Add OSL node and set script
        osl_node = nodes.new('ShaderNodeScript')
        osl_node.mode = 'INTERNAL'
        osl_node.script = bpy.data.texts[shader_type.value]
        
        # Set input parameters after script is loaded
        if 'albedo' in osl_node.inputs:
            osl_node.inputs['albedo'].default_value = albedo

        # Create Output node
        output_node = nodes.new("ShaderNodeOutputMaterial")

        # Connect nodes
        if 'Cout' in osl_node.outputs:
            # Connect shader output directly to Output node
            mat.node_tree.links.new(osl_node.outputs['Cout'], output_node.inputs['Surface'])

    else:
        # Simple diffuse material with GPU rendering
        bpy.context.scene.render.engine = 'CYCLES'
        bpy.context.scene.cycles.device = "GPU"
        bpy.context.scene.cycles.samples = 2048
        mat.diffuse_color = (albedo, albedo, albedo, 1)
    
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

    # Only add sun light for default shader
    if shader_type == ShaderType.DEFAULT:
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
    output_file: str,
    shader_type: ShaderType = ShaderType.DEFAULT
):
    """Render the scene with the given sun angle.

    Args:
        sun_elevation (float): Sun elevation in degrees.
        sun_azimuth (float): Sun azimuth in degrees.
        output_file (str): Path to the output file.
    """
    # Landmark elevation array has top left origin, which requires the sun azimuth to be mirrored over the horizontal axis for to match
    sun_azimuth = 180 - sun_azimuth
    
    if shader_type != ShaderType.DEFAULT:
        # Calculate sun direction vector
        sun_elev_rad = math.radians(sun_elevation)
        sun_azim_rad = math.radians(sun_azimuth)
        # In Blender's coordinate system:
        # -X is east, Y is north, -Z is up
        # The vector points TOWARD the surface
        sun_dir = (
            -math.sin(sun_azim_rad) * math.cos(sun_elev_rad),  # east
            math.cos(sun_azim_rad) * math.cos(sun_elev_rad),   # north
            -math.sin(sun_elev_rad)                            # up
        )
        
        # Update shader if it exists
        for mat in bpy.data.materials:
            if mat.use_nodes:
                for node in mat.node_tree.nodes:
                    if node.type == 'SCRIPT' and node.script:
                        if 'sun_dir' in node.inputs:
                            node.inputs['sun_dir'].default_value = sun_dir
    

    else:
        # Update sun light
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
    parser.add_argument("-resolution", type=float, required=True, help="resolution in meters/pixel of output image")
    parser.add_argument("-albedo", type=float, default=0.14, help="reflectivity of moon (default 14%)")
    parser.add_argument(
        "--save-blend-file",
        action="store_true",
        help="Save a new blend file for additional renders. This option is recommended to accelerate additional renders.",
    )
    parser.add_argument(
        "--use-lunar-lambert",
        action="store_true",
        help="Use lunar Lambert shader instead of default. CPU only.",
    )

    args = parser.parse_args()

    if(args.use_lunar_lambert):
        shader_type = ShaderType.LUNAR_LAMBERT
    else:
        shader_type = ShaderType.DEFAULT

    if(args.file.endswith(".ply")):
        configure_scene(
            args.file,
            width=args.width,
            height=args.height,
            resolution=args.resolution,
            albedo=args.albedo,
            save_blend_file=args.save_blend_file,
            shader_type=shader_type
        )
    elif(args.file.endswith("blend")):
        bpy.ops.wm.open_mainfile(filepath=args.file)
    else:
        print(f"File type not supported: {args.file}, must be a .ply or .blend file")
        exit(1)

    render_sun_angle(
        sun_elevation=args.sun_elevation,
        sun_azimuth=args.sun_azimuth,
        output_file=args.output,
        shader_type=shader_type
    )
