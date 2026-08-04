// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#pragma once

// The objects.conf scene description, shared by everything that reads it: the environment
// map filter rasterises these into an occupancy grid, the MuJoCo filters build bodies out
// of them. The file format has one definition, here, so the two cannot disagree about where
// an object is at a given moment.
//
// Nothing here knows about MuJoCo or octomap -- it parses the file and answers "where is
// object i at time t", and each consumer decides what to do with that.
//
// The parsing itself lives in CObjects.cpp, so libconfig stays out of every translation
// unit that only needs to know what an Object is.

#include <CyC_TYPES.h>
#include <optional>
#include <string>
#include <vector>

namespace objects
{

enum class EasingType
{
    linear,
    quadratic,
    cubic,
    sine,
    exponential,
    circular
};

enum class ShapeKind
{
    rectangle,
    sphere
};

// Maps x in [0, 1] to an eased position along the same interval. Applied once across a
// whole waypoint leg: easing each half of a leg separately (which is what interpolating
// through its midpoint amounted to) turns any non-linear curve into two of them back to
// back, and the object stalls in the middle of every leg.
double ease(double x, EasingType type);

// Falls back to linear, with a warning, for anything unrecognised.
EasingType easingFromString(std::string const& value);

struct Waypoint
{
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    double          duration = 1.;
    EasingType      easing = EasingType::linear;
};

// A prescribed path through space. An object of infinite mass walks it exactly; one with a
// finite mass is pulled toward it by its gains, or ignores it entirely when they are zero,
// because physics has the final say on where a body actually ends up.
struct Behavior
{
    bool                  stationary = true;
    bool                  cyclic = true;
    std::vector<Waypoint> waypoints;    // empty when stationary
    Eigen::Vector3f       position = Eigen::Vector3f::Zero();  // stationary only

    // Progress along the path, advanced by step().
    std::size_t           leg = 0;
    double                elapsed = 0.;

    Eigen::Vector3f start() const;

    // Advances by dt and returns where the object should be. For a one-shot path that has
    // run out this is the final waypoint, rather than wherever the last partial
    // interpolation happened to leave it.
    Eigen::Vector3f step(double dt);
};

struct Object
{
    std::string name;

    ShapeKind kind = ShapeKind::rectangle;
    double    length = 0., width = 0., height = 0.;   // rectangle
    double    radius = 0.;                            // sphere

    // Unset means infinite: the object walks its trajectory exactly and nothing that hits
    // it can move it. A value makes it a physical body instead, subject to gravity and to
    // contact, which only a consumer that runs physics can honour.
    std::optional<double> mass;

    // How hard a finite-mass object is pulled back onto its trajectory. Both zero leaves it
    // a free body that ignores its waypoints and stays where physics puts it.
    double kp = 0.;
    double kd = 0.;

    Behavior behavior;

    bool hasFiniteMass() const { return mass.has_value(); }

    // Half-extents, which is what MuJoCo's box geoms take and what a bounding box around
    // the sphere works out to.
    Eigen::Vector3f halfExtents() const;
};

// Reads a scene description. nullopt if the file cannot be read or defines no usable
// object -- a caller that asked for a scene should stop rather than run an empty one.
// Individual malformed objects are skipped with an error rather than failing the file.
std::optional<std::vector<Object>> load(std::string const& filename);

} // namespace objects
