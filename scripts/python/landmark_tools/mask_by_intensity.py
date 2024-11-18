#!/usr/bin/env python3

##
#  \file   `mask_by_intensity.py`
#  \author Cecilia Mauceri
#  \brief  This script lets you load an image in whatever format support by OpenCV and build a corresponding mask image based on pixel intensity.
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
import os
import cv2
import numpy as np


def get_args():
    parser = argparse.ArgumentParser(
        description="Convert an image and / or build a mask by intensity"
    )
    parser.add_argument(
        "--input_image", type=str, required=True, help="Image to convert / mask"
    )
    parser.add_argument(
        "--output_intensity", default=None, type=str, help="Output intensity image"
    )
    parser.add_argument(
        "--output_mask", default=None, type=str, help="Output mask image"
    )
    parser.add_argument(
        "--mask_out_intensity_less_than",
        default=None,
        type=int,
        help="Mask out intensity less than this",
    )
    parser.add_argument(
        "--mask_out_intensity_greater_than",
        default=None,
        type=int,
        help="Mask out intensity greater than this",
    )
    parser.add_argument(
        "--mask_out_value",
        default=None,
        type=int,
        help="Set masked-out pixels to this value",
    )
    parser.add_argument(
        "--mask_in_value",
        default=None,
        type=int,
        help="Set masked-in pixels to this value",
    )
    args = parser.parse_args()

    # Check arguments
    args.do_output_intensity = args.output_intensity is not None
    # Output mask if either of the mask arguments is provided then check
    args.do_output_mask = any(
        [
            getattr(args, arg_name) is not None
            for arg_name in [
                "output_mask",
                "mask_out_intensity_less_than",
                "mask_out_intensity_greater_than",
                "mask_out_value",
                "mask_in_value",
            ]
        ]
    )
    if args.do_output_mask:
        assert (
            args.output_mask is not None
        ), f"Attempting to output mask but --output_mask path not provided"
        assert (
            args.mask_out_value is not None
        ), f"Attempting to output mask but --mask_out_value path not provided"
        assert (
            args.mask_in_value is not None
        ), f"Attempting to output mask but --mask_in_value path not provided"
        # Check that only one of the less or greater than arguments was provided
        args.do_mask_out_intensity_less_than = (
            args.mask_out_intensity_less_than is not None
        )
        args.do_mask_out_intensity_greater_than = (
            args.mask_out_intensity_greater_than is not None
        )
        assert not (
            args.do_mask_out_intensity_greater_than
            and args.do_mask_out_intensity_less_than
        ), f"Cannot have both arguments --mask_out_intensity_less_than and --mask_out_intensity_greater_than"
        assert (
            args.do_mask_out_intensity_greater_than
            or args.do_mask_out_intensity_less_than
        ), f"Attempting to output mask but neither argument --mask_out_intensity_less_than nor --mask_out_intensity_greater_than was provided"

    return args


def main():
    args = get_args()
    print(f"Load image: {args.input_image}")
    assert os.path.isfile(
        args.input_image
    ), f"--input_image does not exist: {args.input_image}"
    image = cv2.imread(args.input_image, cv2.IMREAD_GRAYSCALE)

    if args.do_output_intensity:
        output_intensity_dir = os.path.dirname(args.output_intensity)
        os.makedirs(output_intensity_dir, exist_ok=True)
        print(f"Write intensity: {args.output_intensity}")
        cv2.imwrite(args.output_intensity, image)
    else:
        print(f"--output_intensity not provided: skip")

    if args.do_output_mask:
        output_mask_dir = os.path.dirname(args.output_mask)
        os.makedirs(output_mask_dir, exist_ok=True)
        # Build mask
        if args.do_mask_out_intensity_less_than:
            print(f"Mask out pixels less than {args.mask_out_intensity_less_than}")
            mask_out_matrix = image < args.mask_out_intensity_less_than
        elif args.do_mask_out_intensity_greater_than:
            print(
                f"Mask out pixels greater than {args.mask_out_intensity_greater_than}"
            )
            mask_out_matrix = image > args.mask_out_intensity_greater_than
        else:
            raise ValueError("This should have been caught during argument parsing")
        print(
            f"  Set masked-in pixels to --mask_in_value={args.mask_in_value} and masked-out pixels to --mask_out_value={args.mask_out_value}"
        )
        mask = np.zeros(image.shape, dtype=image.dtype)
        mask += args.mask_in_value
        mask[mask_out_matrix] = args.mask_out_value
        print(f"  Write mask: {args.output_mask}")
        cv2.imwrite(args.output_mask, mask)
    else:
        print(f"--output_mask not provided: skip")


if __name__ == "__main__":
    main()
