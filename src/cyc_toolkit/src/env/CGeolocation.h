// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CGeolocation_H_
#define CGeolocation_H_

#include "CyC_TYPES.h"

class CGeolocation
{
public:
    CGeolocation();
    virtual ~CGeolocation();

    /**
     * \brief Transforms GPS latitude and longitude coordinates to cartesian coordinates in meters
     *
     * \param _lat   Latitude
     * \param _lon   Longitude
     **/
    static Eigen::Vector2f gps2cartesian(float _lat, float _lon, float _alt);

    static Eigen::Vector2f gps2utm(float _lat, float _lon, float _alt);

    /**
     * \brief Transforms cartesian coordinates in meters to GPS latitude and longitude coordinates
     *
     * \param _lat   Latitude
     * \param _lon   Longitude
     **/
    static Eigen::Vector2f cartesian2gps(float _x, float _y, float _z);

    static Eigen::Vector2f utm2gps(float _x, float _y, float _z);

    static float nmea2Deg(float coord);
};

#endif /* CGeolocation_H_ */
