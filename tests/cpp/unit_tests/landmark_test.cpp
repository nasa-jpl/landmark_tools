#include <gtest/gtest.h>
#include "landmark_tools/landmark_util/landmark.h"
#include "landmark_tools/map_projection/datum_conversion.h"

// Test fixture for landmark tests
class LandmarkTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize test landmark
        lmk = new LMK;
        memset(lmk, 0, sizeof(LMK));
        
        // Set up representative landmark data
        strncpy(lmk->filename, "test_landmark.lmk", LMK_FILENAME_SIZE - 1);
        strncpy(lmk->lmk_id, "TEST001", LMK_ID_SIZE - 1);
        
        // Set basic landmark properties
        lmk->BODY = Earth;  // Using Earth as test planet
        lmk->num_cols = 100;
        lmk->num_rows = 100;
        lmk->resolution = 1.0;  // 1 meter per pixel
        lmk->anchor_col = 50.0;  // Center of the landmark
        lmk->anchor_row = 50.0;
        
        // Set anchor point (example coordinates in meters)
        lmk->anchor_point[0] = 1000.0;  // x
        lmk->anchor_point[1] = 2000.0;  // y
        lmk->anchor_point[2] = 100.0;   // z
        
        // Set up rotation matrix (identity matrix for simplicity)
        lmk->mapRworld[0][0] = 1.0; lmk->mapRworld[0][1] = 0.0; lmk->mapRworld[0][2] = 0.0;
        lmk->mapRworld[1][0] = 0.0; lmk->mapRworld[1][1] = 1.0; lmk->mapRworld[1][2] = 0.0;
        lmk->mapRworld[2][0] = 0.0; lmk->mapRworld[2][1] = 0.0; lmk->mapRworld[2][2] = 1.0;
        
        // Allocate arrays
        EXPECT_TRUE(allocate_lmk_arrays(lmk, lmk->num_cols, lmk->num_rows));
        
        // Fill elevation data with a simple pattern (ramp)
        for (int i = 0; i < lmk->num_rows; i++) {
            for (int j = 0; j < lmk->num_cols; j++) {
                lmk->ele[i * lmk->num_cols + j] = 0;  // Default elevation
                lmk->srm[i * lmk->num_cols + j] = 100;  // Default surface reflectance
            }
        }
        
        // Calculate derived values
        calculateDerivedValuesVectors(lmk);
    }

    void TearDown() override {
        free_lmk(lmk);
        delete lmk;
    }

    LMK* lmk;
};



// Test landmark copy
TEST_F(LandmarkTest, CopyTest) {
    // Set up source landmark
    LMK* source = new LMK;
    memset(source, 0, sizeof(LMK));
    
    // Test copy
    EXPECT_TRUE(Copy_LMK(source, lmk));
    EXPECT_EQ(lmk->num_cols, source->num_cols);
    EXPECT_EQ(lmk->num_rows, source->num_rows);
    EXPECT_EQ(lmk->resolution, source->resolution);
    EXPECT_EQ(lmk->anchor_col, source->anchor_col);
    EXPECT_EQ(lmk->anchor_row, source->anchor_row);
    
    free_lmk(source);
    delete source;
}

// Test coordinate transformations
TEST_F(LandmarkTest, LMK_Col_Row2World_Test) {
    double world_point[3];
    
    // Test world point calculation
    LMK_Col_Row2World(lmk, lmk->anchor_col, lmk->anchor_row, world_point);
    
    // Verify the world point is close to the anchor point
    EXPECT_NEAR(world_point[0], lmk->anchor_point[0], 1e-6);
    EXPECT_NEAR(world_point[1], lmk->anchor_point[1], 1e-6);
    EXPECT_NEAR(world_point[2], lmk->anchor_point[2], 1e-6);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 