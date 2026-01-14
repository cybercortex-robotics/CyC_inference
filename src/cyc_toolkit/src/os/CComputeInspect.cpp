// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CComputeInspect.h"
#ifdef CUDART_VERSION
#include "nvml.h" // NVIDIA management library
#endif

static CyC_INT          numCPUs;
static CyC_UINT		    gpuDeviceCount;
#ifdef WIN32
static PDH_HQUERY       cpuQuery;
static PDH_HCOUNTER     cpuTotal;
static ULARGE_INTEGER   lastCPU, lastSysCPU, lastUserCPU;
static HANDLE           self;
#endif
#ifdef CUDART_VERSION
static nvmlReturn_t	    gpuResult;
#endif

CComputeInspector::CComputeInspector()
{
    m_InitThread = std::thread(&CComputeInspector::init, this);
}

CComputeInspector::~CComputeInspector()
{
#ifdef CUDART_VERSION
    nvmlShutdown();
#endif

    if (m_InitThread.joinable())
    {
        m_InitThread.join();
    }
}

bool CComputeInspector::init()
{
    m_bInitialized = false;

#ifdef CUDART_VERSION
    // Init GPU info (uses NVIDIA management library)
    gpuResult = nvmlInit();
    if (gpuResult != NVML_SUCCESS)
        return m_bInitialized;

    gpuResult = nvmlDeviceGetCount(&gpuDeviceCount);
    if (gpuResult != NVML_SUCCESS)
        return m_bInitialized;

    for (CyC_INT i = 0; i < gpuDeviceCount; ++i)
    {
        nvmlDevice_t device;
        gpuResult = nvmlDeviceGetHandleByIndex(i, &device);

        if (gpuResult != NVML_SUCCESS)
            return m_bInitialized;
    }
#endif

#ifdef WIN32
    // Init memory info
    PdhOpenQuery(NULL, NULL, &cpuQuery);
    // You can also use L"\\Processor(*)\\% Processor Time" and get individual CPU values with PdhGetFormattedCounterArray()
    PdhAddEnglishCounter(cpuQuery, (LPCSTR)"\\Processor(_Total)\\% Processor Time", NULL, &cpuTotal);
    PdhCollectQueryData(cpuQuery);

    // Init CPU info
    SYSTEM_INFO sysInfo;
    FILETIME ftime, fsys, fuser;

    GetSystemInfo(&sysInfo);
    numCPUs = sysInfo.dwNumberOfProcessors;

    GetSystemTimeAsFileTime(&ftime);
    memcpy(&lastCPU, &ftime, sizeof(FILETIME));

    self = GetCurrentProcess();
    GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
    memcpy(&lastSysCPU, &fsys, sizeof(FILETIME));
    memcpy(&lastUserCPU, &fuser, sizeof(FILETIME));
#else
    // TBD
#endif

    m_bInitialized = true;

    return m_bInitialized;
}

void CComputeInspector::getMemUtilization(CyC_ULONG& _out_virtual_total, CyC_ULONG& _out_virtual_total_used, CyC_ULONG& _out_virtual_used_by_current_process,
    CyC_ULONG& _out_physical_total, CyC_ULONG& _out_physical_total_used, CyC_ULONG& _out_physical_used_by_current_process)
{
    if (!m_bInitialized)
        return;

#ifdef WIN32
    MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memInfo);

    PROCESS_MEMORY_COUNTERS_EX pmc;
    GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));

    _out_virtual_total = memInfo.ullTotalPageFile;
    _out_virtual_total_used = memInfo.ullTotalPageFile - memInfo.ullAvailPageFile;
    _out_virtual_used_by_current_process = pmc.PrivateUsage;

    _out_physical_total = memInfo.ullTotalPhys;
    _out_physical_total_used = memInfo.ullTotalPhys - memInfo.ullAvailPhys;
    _out_physical_used_by_current_process = pmc.WorkingSetSize;
#else
    // TBD
#endif
}

void CComputeInspector::getCpuInfo(CyC_UINT& _out_processor_arch, 
    CyC_UINT& _out_num_processors)
{
    if (!m_bInitialized)
        return;

#ifdef WIN32
    // Init memory info
    PdhOpenQuery(NULL, NULL, &cpuQuery);
    // You can also use L"\\Processor(*)\\% Processor Time" and get individual CPU values with PdhGetFormattedCounterArray()
    PdhAddEnglishCounter(cpuQuery, (LPCSTR)"\\Processor(_Total)\\% Processor Time", NULL, &cpuTotal);
    PdhCollectQueryData(cpuQuery);

    // Init CPU info
    SYSTEM_INFO sysInfo;
    GetSystemInfo(&sysInfo);
    _out_processor_arch = sysInfo.wProcessorArchitecture;
    _out_num_processors = sysInfo.dwNumberOfProcessors;
    sysInfo.dwProcessorType;

    //wProcessorArchitecture, wProcessorLevel, and wProcessorRevision

#else
    // TBD
#endif
}

void CComputeInspector::getCpuUtilization(float& _out_used, float& _out_used_by_current_process)
{
    if (!m_bInitialized)
        return;

#ifdef WIN32
    // Get total CPU used
    PDH_FMT_COUNTERVALUE counterVal;
    PdhCollectQueryData(cpuQuery);
    PdhGetFormattedCounterValue(cpuTotal, PDH_FMT_DOUBLE, NULL, &counterVal);
    _out_used = static_cast<float>(counterVal.doubleValue);

    // Get CPU used by current process
    FILETIME ftime, fsys, fuser;
    ULARGE_INTEGER now, sys, user;
    
    GetSystemTimeAsFileTime(&ftime);
    memcpy(&now, &ftime, sizeof(FILETIME));

    GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
    memcpy(&sys, &fsys, sizeof(FILETIME));
    memcpy(&user, &fuser, sizeof(FILETIME));
    _out_used_by_current_process = (float)((sys.QuadPart - lastSysCPU.QuadPart) + (user.QuadPart - lastUserCPU.QuadPart));
    _out_used_by_current_process /= (now.QuadPart - lastCPU.QuadPart);
    _out_used_by_current_process /= numCPUs;
    lastCPU = now;
    lastUserCPU = user;
    lastSysCPU = sys;
    _out_used_by_current_process *= 100.f;
#else
    // TBD
#endif
}

bool CComputeInspector::getGpuInfo(std::vector<std::string>& _out_gpus_names,
    std::vector<CyC_ULONG>& _out_total_mem,
    std::vector<CyC_ULONG>& _out_free_mem,
    std::vector<CyC_ULONG>& _out_used_mem)
{
    _out_gpus_names.clear();

    bool bReturn = false;

    if (!m_bInitialized)
        return false;

#ifdef CUDART_VERSION
    for (CyC_INT i = 0; i < gpuDeviceCount; ++i)
    {
        nvmlDevice_t device;
        gpuResult = nvmlDeviceGetHandleByIndex(i, &device);

        if (gpuResult != NVML_SUCCESS)
        {
            bReturn = false;
            break;
        }

        char device_name[NVML_DEVICE_NAME_BUFFER_SIZE];
        gpuResult = nvmlDeviceGetName(device, device_name, NVML_DEVICE_NAME_BUFFER_SIZE);

        if (gpuResult != NVML_SUCCESS)
        {
            bReturn = false;
            break;
        }

        nvmlMemory_t memInfo;
        gpuResult = nvmlDeviceGetMemoryInfo(device, &memInfo);

        if (gpuResult != NVML_SUCCESS)
        {
            bReturn = false;
            break;
        }

        _out_gpus_names.emplace_back(device_name);
        _out_total_mem.emplace_back(memInfo.total);
        _out_free_mem.emplace_back(memInfo.free);
        _out_used_mem.emplace_back(memInfo.used);

        bReturn = true;
    }
#endif

    return bReturn;
}

bool CComputeInspector::getGpuUtilization(std::vector<float>& _out_gpus_compute, std::vector<float>& _out_gpus_memory)
{
    _out_gpus_compute.clear();
    _out_gpus_memory.clear();

    if (!m_bInitialized)
        return false;

#ifdef CUDART_VERSION
    for (CyC_INT i = 0; i < gpuDeviceCount; ++i)
    {
        nvmlDevice_t device;
        gpuResult = nvmlDeviceGetHandleByIndex(i, &device);

        if (gpuResult != NVML_SUCCESS)
            return false;

        nvmlUtilization_st device_utilization;
        gpuResult = nvmlDeviceGetUtilizationRates(device, &device_utilization);

        if (gpuResult != NVML_SUCCESS)
            return false;

        _out_gpus_compute.emplace_back(device_utilization.gpu);
        _out_gpus_memory.emplace_back(device_utilization.memory);
    }
#endif

    return true;
}

std::string CComputeInspector::processorarch2str(const CyC_UINT& _processor_arch)
{
    std::string arch = "PROCESSOR_ARCHITECTURE_UNKNOWN";
#ifdef WIN32
    switch (_processor_arch)
    {
        case 0:
            arch = "PROCESSOR_ARCHITECTURE_INTEL";
            break;
        case 5:
            arch = "PROCESSOR_ARCHITECTURE_ARM";
            break;
        case 6:
            arch = "PROCESSOR_ARCHITECTURE_IA64";
            break;
        case 9:
            arch = "PROCESSOR_ARCHITECTURE_AMD64";
            break;
        case 12:
            arch = "PROCESSOR_ARCHITECTURE_ARM64";
            break;
        default:
            break;
    }
#else
    // TBD
#endif

    return arch;
}