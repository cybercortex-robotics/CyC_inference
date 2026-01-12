// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CyC_TYPES.h"
#include "CRandom.h"

CRandom::CRandom()
{
    m_Generator = std::make_unique<std::mt19937>(m_Rand_dev());
}

CRandom::~CRandom()
{}
