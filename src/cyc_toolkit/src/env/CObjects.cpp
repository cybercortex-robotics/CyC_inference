// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CObjects.h"
#include "env/CObjectClasses.h"

#include <cmath>
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4275)
#endif
#include <libconfig.h++>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace objects
{

double ease(double x, EasingType type)
{
    switch (type)
    {
    default:
    case EasingType::linear:
        return x;
    case EasingType::quadratic:
        return (x >= 0.5) ? (1. - 2. * pow(1. - x, 2.)) : (2. * x * x);
    case EasingType::cubic:
        return 3. * (x * x) - 2. * (x * x * x);
    case EasingType::sine:
        return 0.5 * (1. - cos(PI * x));
    case EasingType::exponential:
        return (x >= 0.5) ? (1. - 0.5 * pow(2., -20. * x + 10.)) : (0.5 * pow(2., 20. * x - 10.));
    case EasingType::circular:
        return (x >= 0.5) ? (0.5 * (1. + sqrt(1. - pow(2. * x - 2., 2.)))) : (0.5 * (1. - sqrt(1. - pow(2. * x, 2.))));
    }
}

EasingType easingFromString(std::string const& value)
{
    if (value == "linear") return EasingType::linear;
    if (value == "quadratic") return EasingType::quadratic;
    if (value == "cubic") return EasingType::cubic;
    if (value == "sine") return EasingType::sine;
    if (value == "exponential") return EasingType::exponential;
    if (value == "circular") return EasingType::circular;

    spdlog::warn("Objects: unknown easing type '{}'. Defaulting to linear.", value);
    return EasingType::linear;
}

Eigen::Vector3f Behavior::start() const
{
    return stationary ? position : waypoints.front().position;
}

Eigen::Vector3f Behavior::step(double dt)
{
    if (stationary || waypoints.empty())
    {
        return position;
    }

    elapsed += dt;

    // Whole legs are subtracted rather than the clock reset, so the remainder of a step
    // that crosses a waypoint is spent on the next leg instead of dropped -- a frame per
    // waypoint otherwise, every lap of a cyclic path. Terminates because every duration is
    // validated positive at load.
    while ((leg < waypoints.size()) && (elapsed >= waypoints[leg].duration))
    {
        elapsed -= waypoints[leg].duration;
        ++leg;

        if (leg >= waypoints.size())
        {
            if (!cyclic)
            {
                return waypoints.back().position;
            }

            leg = 0;
        }
    }

    if (leg >= waypoints.size())
    {
        return waypoints.back().position;
    }

    Waypoint const& from = waypoints[leg];
    Waypoint const& to = waypoints[(leg + 1) % waypoints.size()];

    const auto e = static_cast<float>(ease(elapsed / from.duration, from.easing));
    return from.position + e * (to.position - from.position);
}

Eigen::Vector3f Object::halfExtents() const
{
    if (kind == ShapeKind::sphere)
    {
        const auto r = static_cast<float>(radius);
        return { r, r, r };
    }

    return { static_cast<float>(0.5 * length),
             static_cast<float>(0.5 * width),
             static_cast<float>(0.5 * height) };
}

namespace
{

bool isArrayLike(libconfig::Setting const& config)
{
    return config.isArray() || config.isGroup() || config.isList();
}

bool readVector3(libconfig::Setting const& config, char const* name, Eigen::Vector3f& out)
{
    if (!config.exists(name) || !isArrayLike(config[name]) || (config[name].getLength() != 3))
    {
        return false;
    }

    for (int j = 0; j < 3; j++)
    {
        try
        {
            out[j] = config[name][j];
        }
        catch (std::exception const& ex)
        {
            spdlog::error("Objects: failed to read '{}' element {}: {}", name, j, ex.what());
            return false;
        }
    }

    return true;
}

bool readBehavior(libconfig::Setting const& config, Behavior& out)
{
    std::string type;
    if (!config.lookupValue("type", type))
    {
        spdlog::error("Objects: behavior type was not specified");
        return false;
    }

    if (type == "stationary")
    {
        out.stationary = true;

        if (!readVector3(config, "position", out.position))
        {
            spdlog::error("Objects: stationary behavior has no valid position");
            return false;
        }

        return true;
    }

    if (type != "straight_line")
    {
        spdlog::error("Objects: unknown behavior type '{}'", type);
        return false;
    }

    out.stationary = false;

    EasingType defaultEasing = EasingType::linear;
    {
        std::string easing;
        if (config.lookupValue("easing", easing))
        {
            defaultEasing = easingFromString(easing);
        }
    }

    if (!config.lookupValue("cyclic", out.cyclic))
    {
        spdlog::warn("Objects: cyclic is enabled by default because it was not specified");
        out.cyclic = true;
    }

    if (!config.exists("waypoints") || !isArrayLike(config["waypoints"]))
    {
        spdlog::error("Objects: straight_line behavior has no waypoints");
        return false;
    }

    std::size_t i = 0;
    for (auto const& waypointConfig : config["waypoints"])
    {
        i++;

        Waypoint waypoint;

        if (!waypointConfig.lookupValue("duration", waypoint.duration))
        {
            spdlog::error("Objects: waypoint #{} does not have a duration set", i);
            return false;
        }

        // A leg that takes no time to walk is not merely degenerate, it hangs the caller:
        // step() would consume it without ever reducing the elapsed time, and a cyclic path
        // would keep handing it back forever.
        if (waypoint.duration <= 0.)
        {
            spdlog::error("Objects: waypoint #{} has a non-positive duration ({})", i, waypoint.duration);
            return false;
        }

        if (!readVector3(waypointConfig, "position", waypoint.position))
        {
            spdlog::error("Objects: waypoint #{} has no valid position", i);
            return false;
        }

        std::string easing;
        waypoint.easing = waypointConfig.lookupValue("easing", easing) ? easingFromString(easing) : defaultEasing;

        out.waypoints.emplace_back(std::move(waypoint));
    }

    if (out.waypoints.empty())
    {
        spdlog::error("Objects: the waypoint list is empty");
        return false;
    }

    return true;
}

bool readObject(libconfig::Setting const& config, std::size_t index, Object& out)
{
    std::string type;
    if (!config.lookupValue("type", type))
    {
        spdlog::error("Objects: object #{} has no type", index);
        return false;
    }

    if (type != "shape")
    {
        spdlog::error("Objects: unknown object type '{}'", type);
        return false;
    }

    out.name = "object_" + std::to_string(index);

    std::string kind;
    if (!config.lookupValue("kind", kind))
    {
        spdlog::error("Objects: object #{} has no shape kind", index);
        return false;
    }

    if (kind == "sphere")
    {
        out.kind = ShapeKind::sphere;
        if (!config.lookupValue("radius", out.radius) || (out.radius <= 0.))
        {
            spdlog::error("Objects: object #{} has no valid radius", index);
            return false;
        }
    }
    else if (kind == "rectangle")
    {
        out.kind = ShapeKind::rectangle;
        if (!config.lookupValue("length", out.length) ||
            !config.lookupValue("width", out.width) ||
            !config.lookupValue("height", out.height))
        {
            spdlog::error("Objects: object #{} is missing length, width and/or height", index);
            return false;
        }
    }
    else
    {
        spdlog::error("Objects: object #{} has unknown shape kind '{}'", index, kind);
        return false;
    }

    // A colour is named, not spelled out in channels, so that a scene file and the rest of
    // the toolkit cannot end up drawing the same thing in two different shades of the same
    // idea. An unknown name is worth a warning but not a rejected object: it costs the
    // object its colour, nothing else.
    std::string colorName;
    if (config.lookupValue("color", colorName))
    {
        if (color::fromName(colorName, out.color))
        {
            // Most of the palette leaves alpha at 0, which reads as fully transparent to a
            // renderer that honours it -- an object that is there but cannot be seen.
            out.color[3] = 255.;
        }
        else
        {
            spdlog::warn("Objects: object #{} has unknown color '{}'; see the 'color' class "
                         "in CObjectClasses.h for the names", index, colorName);
        }
    }

    // Absence is how the file says "infinite": libconfig's float grammar has no spelling of
    // infinity, and it leaves the common case -- an object that walks its trajectory
    // regardless of what runs into it -- as the one needing no configuration.
    if (config.exists("mass"))
    {
        double mass = 0.;
        if (!config.lookupValue("mass", mass))
        {
            spdlog::error("Objects: object #{} has a 'mass' that is not a floating point value (5.0, not 5)", index);
            return false;
        }

        if (mass <= 0.)
        {
            spdlog::error("Objects: object #{} has a non-positive mass ({}); omit it for infinite mass", index, mass);
            return false;
        }

        out.mass = mass;
    }

    if (out.mass.has_value())
    {
        config.lookupValue("kp", out.kp);
        config.lookupValue("kd", out.kd);
    }
    else if (config.exists("kp") || config.exists("kd"))
    {
        spdlog::warn("Objects: object #{} has kp/kd, which are ignored for an object of "
                     "infinite mass -- it already tracks its trajectory exactly", index);
    }

    if (!config.exists("behavior"))
    {
        spdlog::error("Objects: object #{} has no behavior", index);
        return false;
    }

    return readBehavior(config["behavior"], out.behavior);
}

} // namespace

std::optional<std::vector<Object>> load(std::string const& filename)
{
    // Named 'scene' rather than 'objects': inside namespace objects, a local of that name
    // hides the namespace for the rest of the function.
    std::vector<Object> scene;

    try
    {
        libconfig::Config config;
        config.readFile(filename.c_str());

        auto const& root = config.getRoot();
        if (!root.exists("objects") || !isArrayLike(root["objects"]))
        {
            spdlog::error("Objects: no objects found in '{}'", filename);
            return std::nullopt;
        }

        std::size_t i = 0;
        for (auto const& objectConfig : root["objects"])
        {
            i++;

            Object object;
            if (!readObject(objectConfig, i, object))
            {
                spdlog::error("Objects: skipping object #{} of '{}'", i, filename);
                continue;
            }

            scene.emplace_back(std::move(object));
        }
    }
    catch (std::exception const& ex)
    {
        spdlog::error("Objects: failed to load '{}': {}", filename, ex.what());
        return std::nullopt;
    }

    if (scene.empty())
    {
        spdlog::error("Objects: '{}' defines no usable object", filename);
        return std::nullopt;
    }

    return scene;
}

} // namespace objects
