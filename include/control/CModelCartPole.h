// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/*
 * https://github.com/tomstewart89/StateSpaceControl
 * 
 * x = [cart_position, cart_velocity, stick_angle, stick_angular_velocity] ^ T
 * y = [cart_position, stick_angular_velocity] ^ T
 * u = force applied to the cart
 */

#ifndef CModelCartPole_H_
#define CModelCartPole_H_

#include <CCR_TYPES.h>
#include "CBaseStateSpaceModel.h"

class CModelCartPole : public CBaseStateSpaceModel
{
public:
    CModelCartPole(const float _M, const float _m, const float _b, const float _l, const float _I, const float _g = GRAVITY);
    CModelCartPole(const CModelCartPole&) = default;
    CModelCartPole(CModelCartPole&&) = default;
    CModelCartPole& operator=(const CModelCartPole&) = default;
    CModelCartPole& operator=(CModelCartPole&&) = default;
    ~CModelCartPole() = default;

    // State variables mapping
    const float getPosition() const { return this->m_x(0); };
    const float getVelocity() const { return this->m_x(1); };
    const float getPoleAngle() const { return this->m_x(2); };
    const float getPoleAngularVelocity() const { return this->m_x(3); };

private:
    static const CCR_INT m_NumStates = 4;
    static const CCR_INT m_NumInputs = 1;
    static const CCR_INT m_NumOutputs = 2;
};

#endif // CModelCartPole_H_
