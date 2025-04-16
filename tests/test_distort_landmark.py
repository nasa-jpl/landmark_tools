from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.landmark as visualize_corr
import landmark_tools.landmark as landmark

def test_translate():
    """Apply (x, y, z) translation to a DEM, then apply the inverse (-x, -y, -z); result should match the original DEM.
    """
    x = 20 
    y = 30 
    z = 50
    # Run executables
    run_cmd([ TOP_DIR / "build/distort_landmark",
            "-input", "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk",
            "-output", "output/translate_test1.lmk",
            "-translate", str(x), str(y), str(z)], 
            cwd= TEST_DIR)

    run_cmd([ TOP_DIR / "build/distort_landmark",
            "-input", "output/translate_test1.lmk",
            "-output", "output/translate_test2.lmk",
            "-translate", str(-x), str(-y), str(-z)], 
            cwd= TEST_DIR)

    L1 = landmark.Landmark(TEST_DIR / "output/translate_test1.lmk")
    L2 = landmark.Landmark(TEST_DIR / "output/translate_test2.lmk")
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")

    # Check changes
    assert not L1 == L2
    assert L2 == gt

def test_rotate():
    """Apply rotation to a DEM, then apply the inverse rotation; result should match the original DEM.
    """
    rotate = 15
    # Run executables
    run_cmd([ TOP_DIR / "build/distort_landmark",
            "-input", "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk",
            "-output", "output/rotate_test1.lmk",
            "-rotate", str(rotate)], 
            cwd= TEST_DIR)

    run_cmd([ TOP_DIR / "build/distort_landmark",
            "-input", "output/rotate_test1.lmk",
            "-output", "output/rotate_test2.lmk",
            "-rotate", str(-rotate)], 
            cwd= TEST_DIR)

    L1 = landmark.Landmark(TEST_DIR / "output/rotate_test1.lmk")
    L2 = landmark.Landmark(TEST_DIR / "output/rotate_test2.lmk")
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")

    # Check changes
    assert not L1 == L2
    assert L2 == gt

## TODO Test for sine distortion and random distortion?