from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.landmark as landmark

def test_crop(tmp_path):
    """Crop the DEM directly to the target size, and compare with a two-step crop sequence that yields the same final size; results should be identical.

    Cropping a landmark will change the elevation values because the anchor point changes, but the structure should remain the same.
    """
    # Run executables
    crop1_path = tmp_path / "Haworth_final_adj_5mpp_surf_tif_rendered_cropped1.lmk"
    crop2_path = tmp_path / "Haworth_final_adj_5mpp_surf_tif_rendered_cropped2.lmk"
    crop_rough_path = tmp_path / "Haworth_final_adj_5mpp_surf_tif_rendered_cropped_rough.lmk"
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk", 
             "-output", crop1_path, 
             "-operation", "CROP", 
             "-roi", "150", "150", "200", "200"], 
            cwd= TEST_DIR)
    
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk", 
             "-output", crop_rough_path, 
             "-operation", "CROP", 
             "-roi", "50", "50", "400", "400"], 
            cwd= TEST_DIR)
    
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", crop_rough_path, 
             "-output", crop2_path, 
             "-operation", "CROP", 
             "-roi", "100", "100", "200", "200"], 
            cwd= TEST_DIR)

    L_crop1 = landmark.Landmark( crop1_path)
    L_crop2 = landmark.Landmark( crop2_path)
    
    assert L_crop1 == L_crop2

def test_subset(tmp_path):
    """New DEM is a submatrix of the original raster. Elevation values should not change.
    """
    # Run executables
    width = 200
    height = 300
    x1 = 50
    y1 = 25
    subset_path = tmp_path / "Haworth_final_adj_5mpp_surf_tif_rendered_subset.lmk"
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk", 
             "-output", subset_path, 
             "-operation", "SUBSET", 
             "-roi", str(x1), str(y1), str(width), str(height)], 
            cwd= TEST_DIR)

    L_org = landmark.Landmark( TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")
    L_subset = landmark.Landmark( subset_path)
    
    np.testing.assert_allclose(L_org.ele[y1:y1+height, x1:x1+width], L_subset.ele)
    np.testing.assert_allclose(L_org.srm[y1:y1+height, x1:x1+width], L_subset.srm)

    assert L_org.BODY == L_subset.BODY
    assert L_org.lmk_id == L_subset.lmk_id
    assert L_subset.num_cols == width
    assert L_subset.num_rows == height
    assert L_subset.anchor_col == width/2
    assert L_subset.anchor_row== height/2
    assert L_org.resolution == L_subset.resolution
    np.allclose(L_org.mapRworld, L_subset.mapRworld, rtol=0, atol=1e-4)

    ##TODO? Anchor point also changes but by how much would require a calculation

def test_rescale(tmp_path):
    """By upscaling, then downscaling, we should return to the same map.
    """
    # Run executables
    scale = 0.5
    upscale_path = tmp_path / "Haworth_final_adj_5mpp_surf_tif_rendered_upsample.lmk"
    downscale_path = tmp_path / "Haworth_final_adj_5mpp_surf_tif_rendered_downsample.lmk"
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk", 
             "-output", upscale_path, 
             "-operation", "RESCALE", 
             "-scale", str(scale)], 
            cwd= TEST_DIR)
    
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", upscale_path, 
             "-output", downscale_path, 
             "-operation", "RESCALE", 
             "-scale", str(1/scale)], 
            cwd= TEST_DIR)

    L_org = landmark.Landmark( TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")
    L_scale = landmark.Landmark( downscale_path)
    
    assert L_org == L_scale