// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CGeolocation.h"
#include <iostream>

CGeolocation::CGeolocation()
{
}

CGeolocation::~CGeolocation()
{
}

Eigen::Vector2f CGeolocation::gps2cartesian(float _lat, float _lon, float _alt)
{
    // WGS84
    const float a = 6378137.F;
    const float b = 6356752.3142F;
    const float f = (a - b) / a;

    const float e2 = f * (2.F - f);
    const float radius = a / sqrt(1.F - e2 * pow(sin(_lat * DEG2RAD), 2.F));

    //const float radius = b + ((a - b) * cosf(_lat * DEG2RAD)) + _alt;
    const float x = radius * cosf(_lat * DEG2RAD) * cosf(_lon * DEG2RAD);
    const float y = radius * cosf(_lat * DEG2RAD) * sinf(_lon * DEG2RAD);

    return { x, y };
}

CyC_INT latlon_to_zone_number(float lat, float lon)
{
    if (56 <= lat && lat < 64 && 3 <= lon && lon < 12)
        return 32;

    if (72 <= lat && lat <= 84 && lon >= 0)
    {
        if (lon < 9)
            return 31;
        else if (lon < 21)
            return 33;
        else if (lon < 33)
            return 35;
        else if (lon < 42)
            return 37;
    }

    return int((lon + 180.F) / 6.F) + 1;
}

float zone_number_to_central_longitude(CyC_INT zone_number)
{
    return float((zone_number - 1) * 6 - 180 + 3);
}

float mod_angle(float value)
{
    return fmod((value + PI), (2.F * PI)) - PI;
}

Eigen::Vector2f CGeolocation::gps2utm(float _lat, float _lon, float _alt)
{
    const float _lat_rad = _lat * DEG2RAD;
    const float _lon_rad = _lon * DEG2RAD;

    const auto _a = 6378137.F;
    const auto b = 6356752.3142F;
    const auto f = (_a - b) / _a;
    const auto K0 = 0.9996F;
    const auto e2 = f * (2.F - f);
    const auto R = _a;// / sqrt(1.F - e2 * pow(sin(_lat * DEG2RAD), 2.F));

    const auto E = 0.00669438F;
    const auto E2 = E * E;
    const auto E3 = E2 * E;
    const auto E_P2 = E / (1.F - E);

    const auto SQRT_E = sqrt(1.F - E);
    const auto _E = (1.F - SQRT_E) / (1.F + SQRT_E);
    const auto _E2 = _E * _E;
    const auto _E3 = _E2 * _E;
    const auto _E4 = _E3 * _E;
    const auto _E5 = _E4 * _E;

    const auto M1 = (1.F - E / 4.F - 3.F * E2 / 64.F - 5 * E3 / 256.F);
    const auto M2 = (3.F * E / 8.F + 3 * E2 / 32.F + 45.F * E3 / 1024.F);
    const auto M3 = (15.F * E2 / 256.F + 45.F * E3 / 1024.F);
    const auto M4 = (35.F * E3 / 3072.F);

    const auto P2 = (3.F / 2.F * _E - 27.F / 32.F * _E3 + 269.F / 512.F * _E5);
    const auto P3 = (21.F / 16.F * _E2 - 55.F / 32.F * _E4);
    const auto P4 = (151.F / 96.F * _E3 - 417.F / 128.F * _E5);
    const auto P5 = (1097.F / 512.F * _E4);

    const auto lat_sin = sin(_lat_rad);
    const auto lat_cos = cos(_lat_rad);

    const auto lat_tan = lat_sin / lat_cos;
    const auto lat_tan2 = lat_tan * lat_tan;
    const auto lat_tan4 = lat_tan2 * lat_tan2;

    const auto zone_number = latlon_to_zone_number(_lat, _lon);

    const auto central_lon = zone_number_to_central_longitude(zone_number);
    const auto central_lon_rad = central_lon * DEG2RAD;

    const auto n = R / sqrt(1.F - E * lat_sin * lat_sin);
    const auto c = E_P2 * lat_cos * lat_cos;

    const auto a = lat_cos * mod_angle(_lon_rad - central_lon_rad);
    const auto a2 = a * a;
    const auto a3 = a2 * a;
    const auto a4 = a3 * a;
    const auto a5 = a4 * a;
    const auto a6 = a5 * a;

    const auto m = R * (M1 * _lat_rad -
        M2 * sin(2.F * _lat_rad) +
        M3 * sin(4.F * _lat_rad) -
        M4 * sin(6.F * _lat_rad));

    const auto easting = K0 * n * (a +
        a3 / 6.F * (1.F - lat_tan2 + c) +
        a5 / 120.F * (5.F - 18.F * lat_tan2 + lat_tan4 + 72.F * c - 58.F * E_P2)) + 500000.F;

    auto northing = K0 * (m + n * lat_tan * (a2 / 2.F +
        a4 / 24.F * (5.F - lat_tan2 + 9.F * c + 4.F * c * c) +
        a6 / 720.F * (61.F - 58.F * lat_tan2 + lat_tan4 + 600.F * c - 330.F * E_P2)));

    if (_lat < 0.F)
        northing += 10000000.F;

    return { -northing, easting };
}

Eigen::Vector2f CGeolocation::cartesian2gps(float _x, float _y, float _z)
{
    // WGS84
    const float a = 6378137.F;
    const float b = 6356752.3142F;
    const float f = (a - b) / a;

    const float e2 = f * (2.F - f);
    const float ep2 = e2 / (1 - e2);
    const float p = sqrt(_x*_x + _y*_y);

    // Bowring's Method
    // Only one iteration is done as the method has an error of 0.000000030"
    // IF needed (probably not), this can be increased to 2 or 3 iterations
    CyC_INT iters = 1;
    float psi_0 = atan2(a * _z, b * p);

    float lat = 0.F;
    float lng = 0.F;
    while (iters--)
    {
        lat = (_z + b * ep2 * pow(sin(psi_0), 3.F)) / (p - a * e2 * pow(cos(psi_0), 3.F));
        lng = atan2(_y, _x);

        psi_0 = lat;
    }

    return { lat, lng };
}

Eigen::Vector2f CGeolocation::utm2gps(float x, float y, float z)
{
    enum { SOUTH, NORTH };
    const CyC_INT zone_number = 38; // tailored for Brasov, Romania
    const CyC_INT hemisphere = NORTH; // ...

    const auto _a = 6378137.F;
    const auto b = 6356752.3142F;
    const auto f = (_a - b) / _a;
    const auto K0 = 0.9996F;
    const auto e2 = f * (2.F - f);
    const auto R = _a;

    const auto E = 0.00669438F;
    const auto E2 = E * E;
    const auto E3 = E2 * E;
    const auto E_P2 = E / (1.F - E);

    const auto M1 = (1.F - E / 4.F - 3.F * E2 / 64.F - 5 * E3 / 256.F);

    const auto SQRT_E = sqrt(1.F - E);
    const auto _E = (1.F - SQRT_E) / (1.F + SQRT_E);
    const auto _E2 = _E * _E;
    const auto _E3 = _E2 * _E;
    const auto _E4 = _E3 * _E;
    const auto _E5 = _E4 * _E;

    const auto P2 = (3.F / 2.F * _E - 27.F / 32.F * _E3 + 269.F / 512.F * _E5);
    const auto P3 = (21.F / 16.F * _E2 - 55.F / 32.F * _E4);
    const auto P4 = (151.F / 96.F * _E3 - 417.F / 128.F * _E5);
    const auto P5 = (1097.F / 512.F * _E4);

    //{x, y} = { -northing, easting };
    //{x, y} = { easting, -northing }
    // esting -= 500000

    std::swap(x, y);
    x = x - 500000.F;
    y = -y;

    if (hemisphere != NORTH)
        y -= 10000000.F;

    const float m = y / K0;
    const float mu = m / (R * M1);

    const float p_rad = (mu +
        P2 * sin(2.F * mu) +
        P3 * sin(4.F * mu) +
        P4 * sin(6.F * mu) +
        P5 * sin(8.F * mu));

    const float p_sin = sin(p_rad);
    const float p_sin2 = p_sin * p_sin;

    const float p_cos = cos(p_rad);

    const float p_tan = p_sin / p_cos;
    const float p_tan2 = p_tan * p_tan;
    const float p_tan4 = p_tan2 * p_tan2;

    const float ep_sin = 1.F - E * p_sin2;
    const float ep_sin_sqrt = sqrt(1.F - E * p_sin2);

    const float n = R / ep_sin_sqrt;
    const float r = (1 - E) / ep_sin;

    const float c = E_P2 * powf(p_cos, 2);
    const float c2 = c * c;

    const float d = x / (n * K0);
    const float d2 = d * d;
    const float d3 = d2 * d;
    const float d4 = d3 * d;
    const float d5 = d4 * d;
    const float d6 = d5 * d;

    const float latitude = (p_rad - (p_tan / r) *
        (d2 / 2.F - d4 / 24.F * (5.F + 3.F * p_tan2 + 10.F * c - 4.F * c2 - 9.F * E_P2)) +
        d6 / 720.F * (61.F + 90.F * p_tan2 + 298.F * c + 45.F * p_tan4 - 252.F * E_P2 - 3.F * c2));

    float longitude = (d -
        d3 / 6.F * (1.F + 2.F * p_tan2 + c) +
        d5 / 120.F * (5.F - 2.F * c + 28.F * p_tan2 - 3.F * c2 + 8.F * E_P2 + 24.F * p_tan4)) / p_cos;

    longitude = mod_angle(longitude + zone_number_to_central_longitude(zone_number) * DEG2RAD);

    return { latitude * RAD2DEG, longitude * RAD2DEG };
}

float CGeolocation::nmea2Deg(float coord) //  convert ddmm.mmmm to dd.dddd
{
    float degrees = coord / 100.f;
    float minutes = degrees - floor(degrees);
    degrees = floorf(degrees);
    float digits = minutes * 10.f / 6.f;
    return degrees + digits;
}
