// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

// https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
// https://developer.nvidia.com/blog/how-implement-performance-metrics-cuda-cc/
// https://medium.com/devoops-and-universe/monitoring-nvidia-gpus-cd174bf89311

#ifndef CComputeInspect_H_
#define CComputeInspect_H_

#include "CCR_TYPES.h"
#ifdef WIN32
#include "windows.h"
#include "psapi.h"
#include "TCHAR.h"
#include "pdh.h"
#else
// TBD
#endif

class CComputeInspector
{
public:
	CComputeInspector();
	virtual ~CComputeInspector();

	void getMemUtilization(CCR_ULONG& _out_virtual_total, CCR_ULONG& _out_virtual_total_used, CCR_ULONG& _out_virtual_used_by_current_process,
		CCR_ULONG& _out_physical_total, CCR_ULONG& _out_physical_total_used, CCR_ULONG& _out_physical_used_by_current_process);

	void getCpuInfo(CCR_UINT& _out_processor_arch,
		CCR_UINT& _out_num_processors);

	void getCpuUtilization(float& _out_used, float& _out_used_by_current_process);

	bool getGpuInfo(std::vector<std::string>& _out_gpus_names,
		std::vector<CCR_ULONG>& _out_total_mem,
		std::vector<CCR_ULONG>& _out_free_mem,
		std::vector<CCR_ULONG>& _out_used_mem);

	bool getGpuUtilization(std::vector<float>& _out_gpus_compute, std::vector<float>& _out_gpus_memory);

	std::string processorarch2str(const CCR_UINT& _processor_arch);

private:
	bool init();

private:
	bool		m_bInitialized = false;
	std::thread	m_InitThread;
};
#endif /* CComputeInspect_H_ */
