// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CyC_TYPES.h"
#include "os/CComputeInspect.h"

const CyC_ULONG BYTES2MB = 1024 * 1024;

void showUsage()
{
    printf("\nUsage:\n"
        "tu_ComputeInspector \n\n"
        "eg: tu_ComputeInspector \n");
    exit(1);
}

int main(int argc, char** argv)
{
    // Do not use scientific notation
    std::cout << std::fixed;
    std::cout << std::setprecision(6);

    // Compute inspector object
    CComputeInspector m_ComputeInspect;

    std::cout << "CPUs:" << std::endl;
    CyC_UINT processor_architecture, num_processors;
    m_ComputeInspect.getCpuInfo(processor_architecture, num_processors);
    std::cout << "  CPU architecture:\t\t" << m_ComputeInspect.processorarch2str(processor_architecture) << std::endl;
    std::cout << "  Number of CPU processors:\t" << num_processors << std::endl;
    std::cout << std::endl;

    std::cout << "GPUs:" << std::endl;
    std::vector<std::string> gpu_names;
    std::vector<CyC_ULONG> gpu_total_mem, gpu_free_mem, gpu_used_mem;
    if (m_ComputeInspect.getGpuInfo(gpu_names, gpu_total_mem, gpu_free_mem, gpu_used_mem))
    {
        std::vector<float> gpus_compute, gpus_memory;
        m_ComputeInspect.getGpuUtilization(gpus_compute, gpus_memory);

        for (size_t i = 0; i < gpu_names.size(); ++i)
        {
            std::cout << "  " << i << ": " << gpu_names[i] << std::endl;

            std::cout << "    Utilization:\t" << gpus_compute[i] << " %" << std::endl;
            std::cout << "    Total Memory:\t" << gpu_total_mem[i] / BYTES2MB << " MB" << std::endl;
            std::cout << "    Used Memory:\t" << gpu_used_mem[i] / BYTES2MB << " MB" << std::endl;
            std::cout << "    Free Memory:\t" << gpu_free_mem[i] / BYTES2MB << " MB" << std::endl;
        }
    }
    else
    {
        std::cout << "  No available GPU found." << std::endl;
        return 0;
    }

    return 0;
}
