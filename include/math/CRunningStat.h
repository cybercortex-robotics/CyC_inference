// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CyC_RUNNING_STAT_H_
#define CyC_RUNNING_STAT_H_

class CRunningStat
{
public:
    CRunningStat()
        : m_n(0)
    {
    }

    void clear()
    {
        m_n = 0;
    }

    void push(float x)
    {
        m_n++;

        // See Knuth TAOCP vol 2, 3rd edition, page 232
        if (m_n == 1)
        {
            m_oldM = m_newM = x;
            m_oldS = 0.0;
        }
        else
        {
            m_newM = m_oldM + (x - m_oldM) / m_n;
            m_newS = m_oldS + (x - m_oldM) * (x - m_newM);

            // set up for next iteration
            m_oldM = m_newM;
            m_oldS = m_newS;
        }
    }

    int count() const
    {
        return m_n;
    }

    float mean() const
    {
        return (m_n > 0) ? m_newM : 0.F;
    }

    float var() const
    {
        return ((m_n > 1) ? m_newS / (m_n - 1) : 0.F);
    }

    float stddev() const
    {
        return sqrtf(var());
    }

private:
    int m_n;
    float m_oldM, m_newM, m_oldS, m_newS;
};

#endif // CyC_RUNNING_STAT_H_
