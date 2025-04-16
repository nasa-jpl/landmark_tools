from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.landmark as visualize_corr
import landmark_tools.landmark as landmark

def test_legacy_configuration_geographic_regression():
    """Compare the landmark file created using a configuration file and the current code to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    # Run create_landmark
    run_cmd([ TOP_DIR / "build/create_landmark", 
              "-projection", "GEOGRAPHIC",
              "-planet", "Earth",
              "-nodata_value", "-999999",
              "-config_file", "make_equal_rectangular_lmk.txt"], 
              cwd= TOP_DIR / "tests/")

    assert Path(TOP_DIR / "tests/output/equal_rectangular_WY.lmk").exists()

    # Check changes
    L1 = landmark.Landmark( TOP_DIR / "tests/output/equal_rectangular_WY.lmk")
    gt = landmark.Landmark( TOP_DIR / "tests/gold_standard_data/equal_rectangular_WY.lmk")

    # Elevation difference less than 1m
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def test_geotiff_configuration_geographic_regression():
    """Compare the landmark file created by automatically reading geotiff metadata to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    # Run create_landmark
    run_cmd([TOP_DIR / "build/create_landmark",
              "-geotif_file", "gold_standard_data/equal_rectangular_WY.tif",
                "-lmk_file", "output/equal_rectangular_WY_geotif.lmk",
                "-lmk_width_meters", "5000",
                "-lmk_height_meters", "5000",
                "-lmk_res", "10",
                "-lmk_center_lat", "44.55",
                "-lmk_center_long", "-107.44",
                "-planet", "Earth"], 
                cwd= TOP_DIR / "tests/")

    assert Path(TOP_DIR / "tests/output/equal_rectangular_WY_geotif.lmk").exists()

    # Check changes
    L1 = landmark.Landmark(TOP_DIR / "tests/output/equal_rectangular_WY_geotif.lmk")
    gt = landmark.Landmark(TOP_DIR / "tests/gold_standard_data/equal_rectangular_WY.lmk")

    # Elevation difference less than 1 m
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def test_legacy_configuration_utm_regression():
    """Compare the landmark file created using a configuration file and the current code to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    # Run create_landmark
    run_cmd([TOP_DIR / "build/create_landmark", 
              "-projection", "UTM",
              "-planet", "Earth",
              "-nodata_value", "-999999",
              "-config_file", "make_UTM_lmk.txt"], 
              cwd= TOP_DIR / "tests/")

    assert Path(TOP_DIR / "tests/output/UTM_WY.lmk").exists()

    # Check changes
    L1 = landmark.Landmark(TOP_DIR / "tests/output/UTM_WY.lmk")
    gt = landmark.Landmark(TOP_DIR / "tests/gold_standard_data/UTM_WY.lmk")

    # Elevation difference less than 1 m
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def test_geotiff_configuration_utm_regression():
    """Compare the landmark file created by automatically reading geotiff metadata to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1m
    """
    # Run create_landmark
    run_cmd([TOP_DIR / "build/create_landmark",
              "-geotif_file", "gold_standard_data/UTM_WY.tif",
                "-lmk_file", "output/UTM_WY_geotif.lmk",
                "-lmk_width_meters", "5000",
                "-lmk_height_meters", "5000",
                "-lmk_res", "10",
                "-lmk_center_lat", "44.55",
                "-lmk_center_long", "-107.44",
                "-planet", "Earth"], 
                cwd= TOP_DIR / "tests/")

    assert Path(TOP_DIR / "tests/output/UTM_WY_geotif.lmk").exists()

    # Check changes
    L1 = landmark.Landmark(TOP_DIR / "tests/output/UTM_WY_geotif.lmk")
    gt = landmark.Landmark(TOP_DIR / "tests/gold_standard_data/UTM_WY.lmk")

    # Elevation difference less than 1m
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def test_legacy_configuration_stereo_regression():
    """Compare the landmark file created using a configuration file and the current code to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    # Run create_landmark
    run_cmd([TOP_DIR / "build/create_landmark", 
              "-projection", "STEREO",
              "-planet", "Moon",
              "-config_file", "make_polar_stereo_lmk.txt"], 
              cwd="tests/")

    assert Path(TOP_DIR / "tests/output/polarstereo_moon.lmk").exists()

    # Check changes
    L1 = landmark.Landmark(TOP_DIR / "tests/output/polarstereo_moon.lmk")
    gt = landmark.Landmark(TOP_DIR / "tests/gold_standard_data/polarstereo_moon.lmk")

    # Elevation difference less than 1 m
    diff = np.abs(L1.ele-gt.ele)
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def test_geotiff_configuration_stereo_regression():
    """Compare the landmark file created by automatically reading geotiff metadata to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    # Run create_landmark
    run_cmd([TOP_DIR / "build/create_landmark",
              "-geotif_file", "gold_standard_data/polarstereo_moon.tif",
                "-lmk_file", "output/polarstereo_moon_geotif.lmk",
                "-lmk_width_meters", "10000",
                "-lmk_height_meters", "10000",
                "-lmk_res", "20",
                "-lmk_center_lat", "-89.480003",
                "-lmk_center_long", "215.199997",
                "-planet", "Moon"], 
                cwd= TOP_DIR / "tests/")

    assert Path(TOP_DIR / "tests/output/polarstereo_moon_geotif.lmk").exists()

    # Check changes
    L1 = landmark.Landmark(TOP_DIR / "tests/output/polarstereo_moon_geotif.lmk")
    gt = landmark.Landmark(TOP_DIR / "tests/gold_standard_data/polarstereo_moon.lmk")

    # Elevation difference less than 1 m
    diff = np.abs(L1.ele-gt.ele)
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)