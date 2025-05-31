#ifndef CyC_LINEARFIRSTORDERSYSTEM_H_
#define CyC_LINEARFIRSTORDERSYSTEM_H_

#include <CyC_TYPES.h>

// https://val-sagrario.github.io/Dynamics%20of%20First%20Order%20Systems%20for%20game%20devs%20-%20Jan%202020.pdf
// Simulates an approximated 1st order system:
// 
//           1
// G(s) = --------
//        (sT + 1)
// 
// sampled at "dt" seconds
class CLinearFirstOrderSystem
{
public:
    CLinearFirstOrderSystem() = default;
    CLinearFirstOrderSystem(const CLinearFirstOrderSystem&) = default;
    CLinearFirstOrderSystem(CLinearFirstOrderSystem&&) = default;
    CLinearFirstOrderSystem& operator=(const CLinearFirstOrderSystem&) = default;
    CLinearFirstOrderSystem& operator=(CLinearFirstOrderSystem&&) = default;
    ~CLinearFirstOrderSystem() = default;

    void setConstants(float T, float initialY);
    float step(float u, float dt);

private:
    float m_T = 1.F;
    float m_lastY = 0.F;
};

#endif // CyC_LINEARFIRSTORDERSYSTEM_H_
