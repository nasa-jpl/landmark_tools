from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
PYTHON_SCRIPT_DIR = TOP_DIR / 'scripts/python/landmark_tools'
import landmark_tools.landmark as visualize_corr
import landmark_tools.landmark as landmark

def normalized_cross_correlation(img1, img2):
    img1 = (img1 - np.mean(img1)) / np.std(img1)
    img2 = (img2 - np.mean(img2)) / np.std(img2)
    return np.mean(img1 * img2)

def test_render_ply_regression():
    """Compare the shadow rendering current code to an archival copy
    
    Completely different renderer (C vs Blender API), so some small differences in value are expected

    I'm using the gold standard landmark files to render with so that we are only comparing differences in the rendering code, 
    not accumulated differences from the landmark creation and rendering code. 
    """
    # Run executables
    run_cmd([ TOP_DIR / "build/landmark_2_point",
            "-landmark", "gold_standard_data/UTM_WY.lmk",
            "-ply", "output/UTM_WY.ply",
            "-frame", "LOCAL"], 
            cwd= TOP_DIR / "tests/")

    assert Path( TOP_DIR / "tests/output/UTM_WY.ply").exists()

    run_cmd([ "python", PYTHON_SCRIPT_DIR / "render_ply.py",
            "output/UTM_WY.ply",
            "30", "120",
            "output/UTM_WY.png",
            "-height", "500",
            "-width", "500",
            "-resolution", "10"], 
            cwd= TOP_DIR / "tests/")

    assert Path( TOP_DIR / "tests/output/UTM_WY.png").exists()

    run_cmd([ TOP_DIR / "build/add_srm",
            "-input", "gold_standard_data/UTM_WY.lmk",
            "-output", "output/UTM_WY_rendered.lmk",
            "-srm", "output/UTM_WY.png"], 
            cwd= TOP_DIR / "tests/")

    assert Path( TOP_DIR / "tests/output/UTM_WY_rendered.lmk").exists()

    # Check changes
    L1 = landmark.Landmark( TOP_DIR / "tests/output/UTM_WY_rendered.lmk")
    gt = landmark.Landmark( TOP_DIR / "tests/gold_standard_data/UTM_WY.lmk_demo.lmk")

    # NCC is strong
    assert normalized_cross_correlation(L1.srm, gt.srm) > 0.7