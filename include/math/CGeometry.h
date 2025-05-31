// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CGeometry_H
#define CGeometry_H

#include "CyC_TYPES.h"

namespace CGeometry
{
    /**
     * \brief       Calculates the euclidean distance between two vectors
     *
     * \param _v1   Input vector 1
     * \param _v2   Input vector 2
     * \return      Euclidean distance
     **/
    float euclidean_dist(const Eigen::Vector2f& _v1, const Eigen::Vector2f& _v2);
    float euclidean_dist(const Eigen::Vector3f& _v1, const Eigen::Vector3f& _v2);
    float euclidean_dist(const Eigen::Vector4f& _v1, const Eigen::Vector4f& _v2);

    /**
     * \brief       Calculates the absolute distance between two vectors
     *
     * \param _v1   Input vector 1
     * \param _v2   Input vector 2
     * \return      Absolute distance
     **/
    float abs_dist(const Eigen::Vector2f& _pt1, const Eigen::Vector2f& _pt2);
    float abs_dist(const Eigen::Vector3f& _pt1, const Eigen::Vector3f& _pt2);

    /**
     * \brief               Calculates the orthogonal distance between a point and a line
     *
     * \param _line_coeffs  Input line coefficients a, b, c, according to the line equation ax + by + c = 0
     * \param _pt           Input 2D point
     * \return              Orthogonal distance
     **/
    float orthogonal_dist(const Eigen::Vector3f& _line_coeffs, const Eigen::Vector2f& _pt);

    /**
     * \brief       Calculates the magnitude of a vector
     *
     * \param _vect Input vector
     * \return      Magnitude of the vector
     **/
    float mag(const Eigen::Vector2f& _vect);
    float mag(const Eigen::Vector3f& _vect);
    float mag(const Eigen::Vector4f& _vect);

    /**
     * \brief       Calculates the average magnitude of a vector of voxels
     *
     * \param _vect Input vector of voxels
     * \return      Avegare magnitude
     **/
    float mag(const CycVoxels& _vxs);

    /**
     * \brief               Calculates if a point lies on a line
     *
     * \param _line_coeffs  Input line coefficients a, b, c, according to the line equation ax + by + c = 0
     * \param _pt           Input 2D point
     * \return              0 if the point lies on a line
     **/
    float point_on_line(const Eigen::Vector3f& _line_coeffs, const Eigen::Vector2f& _pt);

    /**
     * \brief                   Calculates the intersection of two lines
     *
     * \param _line_coeffs_1    Input coefficients a, b, c for line 1, according to the line equation ax + by + c = 0
     * \param _line_coeffs_2    Input coefficients a, b, c for line 2, according to the line equation ax + by + c = 0
     * \return                  Intersection point
     **/
    Eigen::Vector2f intersection(const Eigen::Vector3f& _line_coeffs_1, const Eigen::Vector3f& _line_coeffs_2);

    /**
     * \brief       Calculates line coefficients from 2 given points
     *
     * \param _pt1  Input point 1
     * \param _pt2  Input point 1
     * \return      Line coefficients a, b, c, according to the line equation ax + by + c = 0
     **/
    Eigen::Vector3f points2line(const Eigen::Vector2f& _pt1, const Eigen::Vector2f& _pt2);

    /**
     * \brief               Calculates the orthogonal projection of a point onto a line
     *
     * \param _line_coeffs  Input line coefficients a, b, c, according to the line equation ax + by + c = 0
     * \param _pt           Input 2D point
     * \return              Output orthogonal projection as 2D point
     **/
    Eigen::Vector2f orthogonal_projection(const Eigen::Vector3f& _line_coeffs, const Eigen::Vector2f& _pt);
}

#endif // CGeometry_H
