from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
PYTHON_SCRIPT_DIR = TOP_DIR / 'scripts/python/landmark_tools'
import landmark_tools.landmark as landmark

def normalized_cross_correlation(img1, img2):
    img1 = (img1 - np.mean(img1)) / np.std(img1)
    img2 = (img2 - np.mean(img2)) / np.std(img2)
    return np.mean(img1 * img2)

def test_render_ply_regression(tmp_path):
    """Compare the shadow rendering current code to an archival copy
    
    Completely different renderer (C vs Blender API), so some small differences in value are expected

    I'm using the gold standard landmark files to render with so that we are only comparing differences in the rendering code, 
    not accumulated differences from the landmark creation and rendering code. 
    """
    ply_path = tmp_path / "UTM_WY.ply"
    png_path = tmp_path / "UTM_WY.png"
    output_path = tmp_path / "UTM_WY_rendered.lmk"
    # Run executables
    run_cmd([ TOP_DIR / "build/landmark_2_point",
            "-landmark", TEST_DIR / "gold_standard_data/UTM_WY.lmk",
            "-ply", ply_path,
            "-frame", "LOCAL"], 
            cwd= TEST_DIR)

    assert Path(ply_path).exists()

    run_cmd([ "python", PYTHON_SCRIPT_DIR / "render_ply.py",
            ply_path,
            "30", "120",
            png_path,
            "-height", "500",
            "-width", "500",
            "-resolution", "10"], 
            cwd= TEST_DIR)

    assert Path( png_path).exists()

    run_cmd([ TOP_DIR / "build/add_srm",
            "-input", TEST_DIR / "gold_standard_data/UTM_WY.lmk",
            "-output", output_path,
            "-srm", png_path], 
            cwd= TEST_DIR)

    assert Path(output_path).exists()

    # Check changes
    L1 = landmark.Landmark( output_path)
    gt = landmark.Landmark( TEST_DIR / "gold_standard_data/UTM_WY.lmk_demo.lmk")

    # NCC is strong
    assert normalized_cross_correlation(L1.srm, gt.srm) > 0.7