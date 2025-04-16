from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
PYTHON_SCRIPT_DIR = TOP_DIR / 'scripts/python/landmark_tools'
TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(1, str(PYTHON_SCRIPT_DIR))
import landmark

def test_landmark_registration():
    """Apply a translation to the DEM, then reregister it; the result should match the original DEM if registration is correct.
 
    """
    x = 20 
    y = 30 
    z = 50
    run_cmd([ TOP_DIR / "build/distort_landmark",
            "-input", "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk",
            "-output", "output/translate_test1.lmk",
            "-translate", str(x), str(y), str(z)], 
            cwd= TEST_DIR)
    
    # Run executables
    run_cmd([ TOP_DIR / "build/landmark_registration",
        "-child", "output/translate_test1.lmk",
        "-base", "output/Haworth_final_adj_5mpp_surf_tif_rendered.lmk"], 
        cwd= TEST_DIR)

    L1 = landmark.Landmark(TEST_DIR / "output/translate_test1.lmk")
    L2 = landmark.Landmark(TEST_DIR / "output/translate_test1.lmk_registered.lmk")
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")

    assert L1 != gt
    assert gt == L2
