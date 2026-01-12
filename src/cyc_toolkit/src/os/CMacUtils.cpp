// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CMacUtils.h"

#ifdef WIN32
#include <Windows.h>
#include <Iphlpapi.h>
#include <Assert.h>
#pragma comment(lib, "iphlpapi.lib")
#else // TBD
#endif

CyC_ULONG CMacUtils::getMAC()
{
    CyC_ULONG mac_addr = 0;
#ifdef WIN32
    mac_addr = getMAC_Windows();
#else
    // TBD
#endif
    return mac_addr;
}

CyC_ULONG CMacUtils::getMAC_Windows()
{
#ifdef WIN32
    std::string strMac("0");
    PIP_ADAPTER_INFO AdapterInfo;
    DWORD dwBufLen = sizeof(IP_ADAPTER_INFO);
    char* mac_addr = (char*)malloc(18);

    AdapterInfo = (IP_ADAPTER_INFO*)malloc(sizeof(IP_ADAPTER_INFO));
    if (AdapterInfo == NULL) {
        printf("Error allocating memory needed to call GetAdaptersinfo\n");
        free(mac_addr);
        return 0; // it is safe to call free(NULL)
    }

    // Make an initial call to GetAdaptersInfo to get the necessary size into the dwBufLen variable
    if (GetAdaptersInfo(AdapterInfo, &dwBufLen) == ERROR_BUFFER_OVERFLOW) {
        free(AdapterInfo);
        AdapterInfo = (IP_ADAPTER_INFO*)malloc(dwBufLen);
        if (AdapterInfo == NULL) {
            printf("Error allocating memory needed to call GetAdaptersinfo\n");
            free(mac_addr);
            return 0;
        }
    }

    if (GetAdaptersInfo(AdapterInfo, &dwBufLen) == NO_ERROR)
    {
        // Contains pointer to current adapter info
        PIP_ADAPTER_INFO pAdapterInfo = AdapterInfo;
        do {
            // technically should look at pAdapterInfo->AddressLength and not assume it is 6.
            sprintf(mac_addr, "%02X:%02X:%02X:%02X:%02X:%02X",
                pAdapterInfo->Address[0], pAdapterInfo->Address[1],
                pAdapterInfo->Address[2], pAdapterInfo->Address[3],
                pAdapterInfo->Address[4], pAdapterInfo->Address[5]);
            //printf("Address: %s, mac: %s\n", pAdapterInfo->IpAddressList.IpAddress.String, mac_addr);
            strMac = mac_addr;
            // print them all, return the last one.
            // return mac_addr;

            printf("\n");
            break;
            pAdapterInfo = pAdapterInfo->Next;
        } while (pAdapterInfo);
    }
    free(AdapterInfo);
    free(mac_addr);

    return convert_mac_2_uint(strMac);
#else
    return 0;
#endif
}

CyC_ULONG CMacUtils::getMAC_Unix()
{
    return 0;
}

CyC_ULONG CMacUtils::convert_mac_2_uint(std::string mac)
{
    // Remove colons
    mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());

    // Convert to uint64_t
    return strtoul(mac.c_str(), NULL, 16);
}
