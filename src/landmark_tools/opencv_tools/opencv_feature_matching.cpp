/** 
 * \copyright Copyright 2024 California Institute of Technology
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

#include "opencv2/calib3d/calib3d.hpp" // for findHomography
#include <opencv2/features2d.hpp> // for drawMatches

#include "opencv_feature_matching.h"

char* HomographyMatchMethodToStr(HomographyMatchMethod method){
    switch (method)
    {
        case SIFT:
            return "SIFT";
            break;
        case ORB:
            return "ORB";
            break;
        default:
            fprintf(stderr, "Unknown HomographyMatchMethod. This should never happen\n");
            return "";
    }
}

// This returns the matrix such that
// homography_arr * base_point is the corresponding point on the child image
// modulo normalizations
bool calc_homography_from_feature_matching(
    uint8_t *base_image,
    uint8_t *base_nan_mask,
    int base_image_num_rows,
    int base_image_num_cols,
    uint8_t *child_image,
    uint8_t *child_nan_mask,
    int child_image_num_rows,
    int child_image_num_cols,
    double homography_arr[3][3],
    HomographyMatchMethod homography_match_method,
    uint32_t homography_min_inlier_count,
    uint32_t *p_homography_found_inlier_count,
    bool do_draw_homography_image,
    char *path_draw_match_image,
    char *path_draw_inlier_image,
    double homography_max_dist_between_matching_keypoints
)
{
    // Build cv::Mat objects
    cv::Mat base_image_mat = cv::Mat(base_image_num_rows, base_image_num_cols, CV_8UC1, base_image);
    cv::Mat child_image_mat = cv::Mat(child_image_num_rows, child_image_num_cols, CV_8UC1, child_image);
    cv::Mat base_mask_mat = cv::Mat(base_image_num_rows, base_image_num_cols, CV_8UC1);
    cv::Mat child_mask_mat = cv::Mat(child_image_num_rows, child_image_num_cols, CV_8UC1);
    // Feature detection masks are such that non-zero is region of interest
    int base_pixel = 0;
    for (int row = 0; row < base_image_num_rows; ++row)
    {
        for (int col = 0; col < base_image_num_cols; ++col)
        {
            if (base_nan_mask[base_pixel] > 0)
            {
                base_mask_mat.at<uint8_t>(row, col) = 0;
            }
            else
            {
                base_mask_mat.at<uint8_t>(row, col) = 1;
            }
            base_pixel += 1;
        }
    }
    int child_pixel = 0;
    for (int row = 0; row < child_image_num_rows; ++row)
    {
        for (int col = 0; col < child_image_num_cols; ++col)
        {
            if (child_nan_mask[child_pixel] > 0)
            {
                child_mask_mat.at<uint8_t>(row, col) = 0;
            }
            else
            {
                child_mask_mat.at<uint8_t>(row, col) = 1;
            }
            child_pixel += 1;
        }
    }

    std::vector<cv::KeyPoint> base_keypoints, child_keypoints;
    std::vector<cv::DMatch> matches;

    cv::Mat base_descriptors, child_descriptors;
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (homography_match_method == ORB)
    {
        detector = cv::ORB::create();
        descriptor = cv::ORB::create();
        matcher = cv::BFMatcher::create(cv::NORM_HAMMING);

        // Detect keypoints
        detector->detect(base_image_mat, base_keypoints, base_mask_mat);
        detector->detect(child_image_mat, child_keypoints, child_mask_mat);

        // TODO: filter keypoints based on masks

        // Compute descriptors
        descriptor->compute(base_image_mat, base_keypoints, base_descriptors);
        descriptor->compute(child_image_mat, child_keypoints, child_descriptors);

        // Compute matches
        matcher->match(base_descriptors, child_descriptors, matches);
    }
    else if (homography_match_method == SIFT)
    {
        detector = cv::SIFT::create();
        descriptor = cv::SIFT::create();
        matcher = cv::BFMatcher::create(cv::NORM_L1);

        // Detect keypoints
        detector->detect(base_image_mat, base_keypoints, base_mask_mat);
        detector->detect(child_image_mat, child_keypoints, child_mask_mat);

        // TODO: filter keypoints based on masks

        // Compute descriptors
        descriptor->compute(base_image_mat, base_keypoints, base_descriptors);
        descriptor->compute(child_image_mat, child_keypoints, child_descriptors);

        // Compute matches
        matcher->match(base_descriptors, child_descriptors, matches);
    }
    else
    {
      printf("Unrecognized homography match method: %u\n", homography_match_method);
      return false;
    }

    // Filter matches
    if (homography_max_dist_between_matching_keypoints > 0)
    {
      std::vector<cv::DMatch> filtered_matches = {};
      for(int i = 0; i < matches.size(); i++ )
      {
        cv::Point2f base_point = base_keypoints[matches[i].queryIdx].pt;
        cv::Point2f child_point = child_keypoints[matches[i].trainIdx].pt;
        cv::Point2f dist = base_point - child_point;
        if (
          (std::abs(dist.x) <= homography_max_dist_between_matching_keypoints)
          && (std::abs(dist.y) <= homography_max_dist_between_matching_keypoints)
        )
        {
          filtered_matches.push_back(matches[i]);
        }
      }
      printf(
          "Filtered out matches further than %f pixels on any axis: kept %d matches out of %d\n",
          homography_max_dist_between_matching_keypoints,
          (uint32_t) filtered_matches.size(),
          (uint32_t) matches.size()
      );
      matches = filtered_matches;
    }

    // Check that we even have enough matches to start
    if (matches.size() < homography_min_inlier_count)
    {
      printf(
          "Homography failed: got %d matches but required %d\n",
          (uint32_t) matches.size(),
          homography_min_inlier_count
      );
      return false;
    }

    // Calculate homography
    std::vector<cv::Point2f> matching_base_points;
    std::vector<cv::Point2f> matching_child_points;
    for(int i = 0; i < matches.size(); i++ )
    {
      matching_base_points.push_back(base_keypoints[matches[i].queryIdx].pt);
      matching_child_points.push_back(child_keypoints[matches[i].trainIdx].pt);
    }
    double ransac_reprojection_threshold = 3.0;
    cv::Mat homography_inlier_mask;
    cv::Mat homography_mat = findHomography(
        matching_base_points,
        matching_child_points,
        cv::RANSAC,
        ransac_reprojection_threshold,
        homography_inlier_mask
    );

    // Convert inlier mask to CV_8UC1 since we're not sure about the mask type
    cv::Mat homography_inlier_mask_char;
    homography_inlier_mask.convertTo(homography_inlier_mask_char, CV_8UC1);
    std::vector<cv::DMatch> inlier_matches;
    for(int i = 0; i < matches.size(); i++ )
    {
      if (homography_inlier_mask_char.at<uint8_t>(i, 0) > 0)
      {
        inlier_matches.push_back(matches.at(i));
      }
    }
    *p_homography_found_inlier_count = (uint32_t) inlier_matches.size();

    // Fail early if we didn't get enough inliers
    if (*p_homography_found_inlier_count < homography_min_inlier_count)
    {
      printf(
          "Homography failed: got %d inliers out of %d matches but required %d\n",
          *p_homography_found_inlier_count,
          (uint32_t) matches.size(),
          homography_min_inlier_count
      );
      return false;
    }

    // Also convert homography mat to CV_32FC1 since we're not sure about the type
    cv::Mat homography_mat_double;
    homography_mat.convertTo(homography_mat_double, CV_64FC1);

    // Populate output matrix
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            homography_arr[row][col] = homography_mat_double.at<double>(row, col);
        }
    }

    // Draw matches
    if (do_draw_homography_image)
    {
      cv::Mat match_mat;
      cv::drawMatches(
          base_image_mat,
          base_keypoints,
          child_image_mat,
          child_keypoints,
          matches,
          match_mat,
          cv::Scalar::all(-1),
          cv::Scalar::all(-1),
          std::vector<char>(),
          cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
      );
      cv::imwrite(path_draw_match_image, match_mat);
      cv::Mat inlier_mat;
      cv::drawMatches(
          base_image_mat,
          base_keypoints,
          child_image_mat,
          child_keypoints,
          inlier_matches,
          inlier_mat,
          cv::Scalar::all(-1),
          cv::Scalar::all(-1),
          std::vector<char>(),
          cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
      );
      cv::imwrite(path_draw_inlier_image, inlier_mat);
    }

    return true;
}

