#include "CLinearFirstOrderSystem.h"

void CLinearFirstOrderSystem::setConstants(float T, float initialY)
{
    if (T > 1e-6)
    {
        m_T = T;
        m_lastY = initialY;
    }
    else
    {
        spdlog::error("CLinearFirstOrderSystem: invalid time constant T={}. MUST be non-null and positive.", T);
    }
}

float CLinearFirstOrderSystem::step(float u, float dt)
{
    const auto a = dt / (dt + m_T);
    const auto y = a * u + (1.F - a) * m_lastY;

    m_lastY = y;

    return y;
}

