#ifndef _LANDMARK_TOOLS_POINT_LINE_PLANE_UTIL_H_
#define _LANDMARK_TOOLS_POINT_LINE_PLANE_UTIL_H_
//2D
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 \brief Calculate distance between 3D point `P` and `plane` 
 
 \param[in] P (x,y,z) point 
 \param[in] plane The coefficients of the plane equation Ax + By + Cz + D = 0.
 \return double distance 
*/
double Point2PlaneDist(double P[3], double plane[4]);

/**
 \brief Calculate distance between 2D point and 2D line
 
 \param[in] x x-coordinate of point
 \param[in] y y-coordinate of point
 \param[in] line = [a, b, c] where ax + by + c = 0 
 \return double distance
*/
double XY2LineDist2D(double x, double y, double line[3]);

/**
 \brief Calculate distance between 2D point and 2D line
 
 \param[in] pt = [x, y]
 \param[in] line = [a, b, c] where ax + by + c = 0 
 \return double distance 
*/
double Point2LineDist2D(double pt[2], double line[3]);

/**
 \brief Find coordinates of 2D point `pin` projected on 2D line
 
 \param[in] vec 2x1 direction vector for line
 \param[in] p0 [x,y] point on line
 \param[in] pin [x,y] point to project
 \param[out] pout [x,y] projected point
*/
void PointProject2Line2D(double vec[2], double p0[2], double pin[2], double pout[2]);

/**
 \brief Find coordinates of 3D point `pin` projected on 3D line
 
 \param[in] vec 3x1 direction vector for line
 \param[in] p0 [x,y,z] point on line
 \param[in] pin [x,y,z] point to project
 \param[out] pout [x,y,z] projected point
*/
int32_t PointProject2Line3D(double vec[3], double p0[3], double pin[3], double pout[3]);

/**
 \brief Calculate distance between two 3D points
 
 \param[in] p1 
 \param[in] p2 
 \return double distance 
*/
double PointsDist3D(double p1[3], double p2[3]);

/**
 * @brief Compute the intersection point between a ray and a plane in 3D space.
 *
 * This function calculates the intersection point between a ray, defined by a point 'p' and a direction 'ray',
 * and a plane specified by its equation in the form Ax + By + Cz + D = 0, where [A, B, C] are the plane's normal
 * coefficients and 'D' is the plane's offset from the origin.
 *
 * @param p [in] The origin point of the ray.
 * @param ray [in] The direction vector of the ray.
 * @param plane [in] The coefficients of the plane equation Ax + By + Cz + D = 0.
 * @param p_out [out] The calculated intersection point (x, y, z) in world coordinates.
 *
 * @return An integer value (1) indicating a successful intersection.
 */
int32_t PointRayIntersection2Plane(double p[3], double r[3], double plane[4], double p_out[3]);

/**
 \brief Given a normal of a plane and a point on the plane to get the equation of the plane
 
 \param[in] vec plane normal
 \param[in] p point on plane
 \param[out] plane The coefficients of the plane equation Ax + By + Cz + D = 0.
 \return int32_t 
*/
int32_t normalpoint2plane(double vec[3], double p[3], double plane[4]);

/**
 \brief TODO 
 
 \param[] ptsA 
 \param[] ptsB 
 \param[] num_pts 
 \param[] bRa 
 \param[] T 
 \return int32_t 
*/
int32_t  Point_Clouds_rot_T(double *ptsA, double *ptsB, int32_t num_pts, double bRa[3][3], double T[3]);

/**
 \brief TODO
 
 \param[] ptsA 
 \param[] ptsB 
 \param[] num_pts 
 \param[] bRa 
 \param[] T 
 \param[] tol 
 \return int32_t 
*/
int32_t  Point_Clouds_rot_T_RANSAC(double *ptsA, double *ptsB, int32_t num_pts, double bRa[3][3], double T[3], double tol);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //_LANDMARK_TOOLS_POINT_LINE_PLANE_UTIL_H_
