// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CSensorFusion.h"

bool CSensorFusion::accumulateOctree()
{
    const float occupied_probability = 0.85F; // to be replaced by the sensor value?
    const float free_probability = 0.4F;
    /*
    if (m_bAccumulate && (m_lastState.x_hat.size() > 0))
    {
        octomap::Pointcloud pcl;
        std::vector<float> logodds;
        std::vector<CyC_INT> classes;

        const auto potential_size = m_accumulatedOctree->pOccupancyModel->getNumLeafNodes();
        pcl.reserve(potential_size);
        logodds.reserve(potential_size);
        classes.reserve(potential_size);

        for (auto leaf = m_accumulatedOctree->pOccupancyModel->begin_leafs(); leaf != m_accumulatedOctree->pOccupancyModel->end_leafs(); ++leaf)
        {
            if (leaf->getOccupancy() > 0.3)
            {
                pcl.push_back(leaf.getX(), leaf.getY(), leaf.getZ());
                logodds.push_back(leaf->getValue());
                classes.push_back(leaf->getObjectClass());
            }
        }

        const auto dx_hat = currentState.x_hat - m_lastState.x_hat;
        const CPose absolute_rotation{ 0.F, 0.F, 0.F, 0.F, 0.F, -currentState.x_hat[3] };
        const CPose relative_rotation{ 0.F, 0.F, 0.F, 0.F, 0.F, -dx_hat[3] };
        const Eigen::Vector4f relative_translation{ -dx_hat[0], -dx_hat[1], 0.F, 1.F };

        m_accumulatedOctree->pOccupancyModel->clear();
        for (size_t i = 0; i < pcl.size(); ++i)
        {
            const Eigen::Vector4f original_point{ pcl[i].x(), pcl[i].y(), pcl[i].z(), 1.F };
            const Eigen::Vector4f point = relative_rotation.transform() *
                (original_point + (absolute_rotation.transform() * relative_translation));

            auto* node = m_accumulatedOctree->pOccupancyModel->updateNode(point.x(), point.y(), point.z(), true, true);
            node->setValue(logodds[i]);
            node->setObjectClass(classes[i]);
        }
    }
    else
    {
        m_accumulatedOctree->pOccupancyModel->clear();
    }

    m_lastState = currentState;
    */

    /*
    // Accumulate for lidar
    if (m_bAccumulate)
    {
        for (auto& leaf : *m_accumulatedOctree->pOccupancyModel)
        {
            if (leaf.getObjectClass() == CObjectClasses::LIDAR)
                leaf.setValue(leaf.getValue() + octomap::logodds(free_probability));
        }
    }
    */

    /*
    // Accumulate for semseg
    if (m_bAccumulate)
    {
        for (auto& leaf : *m_accumulatedOctree->pOccupancyModel)
        {
            if (leaf.getObjectClass() == 18)
                leaf.setValue(leaf.getValue() + octomap::logodds(free_probability));
        }
    }
    */

    return false;
}