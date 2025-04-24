/**
 * \file homography_estimation.h
 * \brief Functions for estimating homography between images using feature matching
 * 
 * This file provides functions for estimating homography transformations between
 * images using various feature matching methods.
 */

#ifndef HOMOGRAPHY_ESTIMATION_H
#define HOMOGRAPHY_ESTIMATION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * \brief Estimate homography between two images using feature matching
 * 
 * This function estimates a homography transformation between a base image and a
 * child image using feature matching. It tries multiple feature matching methods
 * and selects the best one based on the number of inliers.
 * 
 * \param[in] base_image Base image data
 * \param[in] base_nan_mask Mask for NaN pixels in base image
 * \param[in] base_image_num_rows Number of rows in base image
 * \param[in] base_image_num_cols Number of columns in base image
 * \param[in] child_image Child image data
 * \param[in] child_nan_mask Mask for NaN pixels in child image
 * \param[in] child_image_num_rows Number of rows in child image
 * \param[in] child_image_num_cols Number of columns in child image
 * \param[out] base2child Estimated homography matrix (3x3)
 * \param[in] output_dir Directory for saving debug outputs
 * \param[in] homography_max_dist_between_matching_keypoints Maximum allowed distance between matching keypoints
 * \return true if homography estimation was successful, false otherwise
 */
bool estimateHomographyFromFeatureMatching(
    uint8_t *base_image,
    uint8_t *base_nan_mask,
    int base_image_num_rows,
    int base_image_num_cols,
    uint8_t *child_image,
    uint8_t *child_nan_mask,
    int child_image_num_rows,
    int child_image_num_cols,
    double base2child[3][3],
    char *output_dir,
    double homography_max_dist_between_matching_keypoints
);

#endif /* HOMOGRAPHY_ESTIMATION_H */ 