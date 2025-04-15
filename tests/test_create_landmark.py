from pathlib import Path
import sys
sys.path.insert(1, 'scripts/python/landmark_tools')
import landmark
from test_helpers import run_cmd

import numpy as np

def test_legacy_configuration():
    """Compare the landmark file created using a configuration file and the current code to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 60 cm
    """
    # Run create_landmark
    run_cmd(["../build/create_landmark", 
              "-projection", "GEOGRAPHIC",
              "-planet", "Earth",
              "-nodata_value", "-999999",
              "-config_file", "make_equal_rectangular_lmk.txt"], cwd="tests/")

    assert (Path("tests/output/equal_rectangular_WY.lmk").exists())

    # Check changes
    L1 = landmark.Landmark("tests/output/equal_rectangular_WY.lmk")
    gt = landmark.Landmark("tests/gold_standard_data/equal_rectangular_WY.lmk")

    # Elevation difference less than 60 cm
    assert np.all(np.less(np.abs(L1.ele-gt.ele), 0.6))

def test_geotiff_configuration():
    """Compare the landmark file created by automatically reading geotiff metadata to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    # Run create_landmark
    run_cmd(["../build/create_landmark", " -projection", "GEOGRAPHIC",
              "-geotif_file", "gold_standard_data/equal_rectangular_WY.tif",
                "-lmk_file", "output/equal_rectangular_WY_geotif.lmk",
                "-lmk_width_meters", "5000",
                "-lmk_height_meters", "5000",
                "-lmk_res", "10",
                "-lmk_center_lat", "44.55",
                "-lmk_center_long", "-107.44",
                "-planet", "Earth"], cwd="tests/")

    assert (Path("tests/output/equal_rectangular_WY_geotif.lmk").exists())

    # Check changes
    L1 = landmark.Landmark("tests/output/equal_rectangular_WY_geotif.lmk")
    gt = landmark.Landmark("tests/gold_standard_data/equal_rectangular_WY.lmk")

    # Elevation difference less than 1 m
    assert np.all(np.less(np.abs(L1.ele-gt.ele), 1))