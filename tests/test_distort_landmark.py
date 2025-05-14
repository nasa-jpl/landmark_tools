from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.landmark as landmark

def test_translate(tmp_path):
    """Apply (x, y, z) translation to a DEM, then apply the inverse (-x, -y, -z); result should match the original DEM.
    """
    x = 20 
    y = 30 
    z = 50
    test1_path = tmp_path / "translate_test1.lmk"
    test2_path = tmp_path / "translate_test2.lmk"
    # Run executables
    run_cmd([ TOP_DIR / "build/distort_landmark",
            "-input", TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk",
            "-output", test1_path,
            "-translate", str(x), str(y), str(z)], 
            cwd= TEST_DIR)

    run_cmd([ TOP_DIR / "build/distort_landmark",
            "-input", test1_path,
            "-output", test2_path,
            "-translate", str(-x), str(-y), str(-z)], 
            cwd= TEST_DIR)

    L1 = landmark.Landmark(test1_path)
    L2 = landmark.Landmark(test2_path)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")

    # Check changes
    assert not L1 == L2
    assert L2 == gt

def test_rotate(tmp_path):
    """Apply rotation to a DEM, then apply the inverse rotation; result should match the original DEM.
    """
    rotate = 15
    test1_path = tmp_path / "rotate_test1.lmk"
    test2_path = tmp_path / "rotate_test2.lmk"
    # Run executables
    run_cmd([ TOP_DIR / "build/distort_landmark",
            "-input", TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk",
            "-output", test1_path,
            "-rotate", str(rotate)], 
            cwd= TEST_DIR)

    run_cmd([ TOP_DIR / "build/distort_landmark",
            "-input", test1_path,
            "-output", test2_path,
            "-rotate", str(-rotate)], 
            cwd= TEST_DIR)

    L1 = landmark.Landmark(test1_path)
    L2 = landmark.Landmark(test2_path)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")

    # Check changes
    assert not L1 == L2
    assert L2 == gt

## TODO Test for sine distortion and random distortion?