#!/usr/bin/env python3
 
# #
#  \file   `batch_render.py`
#  \author Cecilia Mauceri
#  \brief  This script is used to render a batch of different sun angles with blender
#  Usage: python3 batch_render.py file angle_csv -height HEIGHT -width WIDTH [--save-blend-file]
#  
#  Copyright 2024 California Institute of Technology
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
import csv
import render_ply

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Render a batch of different sun angles with blender")
    parser.add_argument("file", type=str, help="plyfile defining a DEM mesh in a local reference frame OR a blend file from a previous run")
    parser.add_argument("angle_csv", type=str, help="CSV file containing one line per desired sun angle. Line format is <timestamp, elevation_in_degrees, azimuth_in_degrees>")
    parser.add_argument("-height", type=int, required=True, help="height of output file in pixels")
    parser.add_argument("-width", type=int, required=True, help="width of output file in pixels")
    parser.add_argument(
        "--save-blend-file",
        action="store_true",
        help="Save a new blend file for additional renders. This option is recommended to accelerate additional renders.",
    )

    args = parser.parse_args()
    if(args.file.endswith(".ply")):
        render_ply.configure_scene(
            args.file,
            width=args.width,
            height=args.height,
            save_blend_file=args.save_blend_file
        )
    elif(args.file.endswith("blend")):
        bpy.ops.wm.open_mainfile(filepath=args.file)
    else:
        print(f"File type not supported: {args.file}, must be a .ply or .blend file")
        exit(1)

    with open(args.angle_csv, newline='') as csvfile:
        angle_reader = csv.reader(csvfile, delimiter=',')
        for row in angle_reader:
            sun_elevation = float(row[1])
            sun_azimuth = float(row[2])
            output_file = "{}_{:0.3f}e_{:0.3f}a.png".format(args.file.replace(".blend", ""), sun_elevation, sun_azimuth)

            render_ply.render_sun_angle(
                sun_elevation,
                sun_azimuth,
                output_file
            )