// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CRandom_H
#define CRandom_H

#include "CyC_TYPES.h"
#include <random>
#include <vector>

class CRandom
{
public:
    CRandom();
    ~CRandom();

    template<typename T>
    std::vector<T> uniform(T range_from, T range_to, CyC_INT num_samples)
    {
        std::uniform_int_distribution<T> distr(range_from, range_to);
        std::vector<T> samples;

        for (size_t i = 0; i < num_samples; ++i)
            samples.emplace_back(distr(*m_Generator));

        return samples;
    }

    template<typename T>
    std::vector<T> uniform_unique(T range_from, T range_to, CyC_INT num_samples)
    {
        std::uniform_int_distribution<T> distr(range_from, range_to);
        std::vector<T> samples;

        for (size_t i = 0; i < num_samples; ++i)
        {
            T sample = distr(*m_Generator);

            bool bFound = false;
            for (const auto& n : samples)
                if (n == sample)
                    bFound = true;

            if (!bFound)
                samples.emplace_back(sample);
        }

        return samples;
    }

private:
    std::random_device              m_Rand_dev;
    std::unique_ptr<std::mt19937>   m_Generator;
};

#endif // CRandom_H