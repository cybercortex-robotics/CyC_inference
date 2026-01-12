// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu
/*
 * Windows service for managing a CCR core
 * https://www.codeproject.com/Articles/499465/Simple-Windows-Service-in-Cplusplus
 * 
 * Install service (run cmd as admin):
 * sc create "CCR Service" binPath=C:\dev\src\CyberCortex.AI\core\bin\App_CycDaemon.exe
 * 
 * Uninstall service:
 * sc delete "CCR Service"
 */

#include <Windows.h>
#include <tchar.h>
#include <direct.h>
#include <filesystem>

SERVICE_STATUS        g_ServiceStatus = {0};
SERVICE_STATUS_HANDLE g_StatusHandle = NULL;
HANDLE                g_ServiceStopEvent = INVALID_HANDLE_VALUE;

VOID WINAPI ServiceMain (DWORD argc, LPTSTR *argv);
VOID WINAPI ServiceCtrlHandler (DWORD);
DWORD WINAPI ServiceWorkerThread (LPVOID lpParam);

#define SERVICE_NAME  _T("CCR Service")

bool service_started = false;

// additional information
STARTUPINFO si;
PROCESS_INFORMATION pi;


int _tmain (int argc, TCHAR *argv[])
{
    // set the size of the structures
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));

    OutputDebugString(_T("CCR Service: Main: Entry"));

    SERVICE_TABLE_ENTRY ServiceTable[] = 
    {
        {SERVICE_NAME, (LPSERVICE_MAIN_FUNCTION) ServiceMain},
        {NULL, NULL}
    };

    if (StartServiceCtrlDispatcher (ServiceTable) == FALSE)
    {
       OutputDebugString(_T("CCR Service: Main: StartServiceCtrlDispatcher returned error"));
       return GetLastError ();
    }

    OutputDebugString(_T("CCR Service: Main: Exit"));
    return 0;
}


VOID WINAPI ServiceMain (DWORD argc, LPTSTR *argv)
{
    DWORD Status = E_FAIL;

    OutputDebugString(_T("CCR Service: ServiceMain: Entry"));

    g_StatusHandle = RegisterServiceCtrlHandler (SERVICE_NAME, ServiceCtrlHandler);

    if (g_StatusHandle == NULL) 
    {
        OutputDebugString(_T("CCR Service: ServiceMain: RegisterServiceCtrlHandler returned error"));
        goto EXIT;
    }

    // Tell the service controller we are starting
    ZeroMemory (&g_ServiceStatus, sizeof (g_ServiceStatus));
    g_ServiceStatus.dwServiceType = SERVICE_WIN32_OWN_PROCESS;
    g_ServiceStatus.dwControlsAccepted = 0;
    g_ServiceStatus.dwCurrentState = SERVICE_START_PENDING;
    g_ServiceStatus.dwWin32ExitCode = 0;
    g_ServiceStatus.dwServiceSpecificExitCode = 0;
    g_ServiceStatus.dwCheckPoint = 0;

    if (SetServiceStatus (g_StatusHandle, &g_ServiceStatus) == FALSE) 
    {
        OutputDebugString(_T("CCR Service: ServiceMain: SetServiceStatus returned error"));
    }

    /* 
     * Perform tasks neccesary to start the service here
     */
    OutputDebugString(_T("CCR Service: ServiceMain: Performing Service Start Operations"));

    // Create stop event to wait on later.
    g_ServiceStopEvent = CreateEvent (NULL, TRUE, FALSE, NULL);
    if (g_ServiceStopEvent == NULL) 
    {
        OutputDebugString(_T("CCR Service: ServiceMain: CreateEvent(g_ServiceStopEvent) returned error"));

        g_ServiceStatus.dwControlsAccepted = 0;
        g_ServiceStatus.dwCurrentState = SERVICE_STOPPED;
        g_ServiceStatus.dwWin32ExitCode = GetLastError();
        g_ServiceStatus.dwCheckPoint = 1;

        if (SetServiceStatus (g_StatusHandle, &g_ServiceStatus) == FALSE)
	    {
		    OutputDebugString(_T("CCR Service: ServiceMain: SetServiceStatus returned error"));
	    }
        goto EXIT; 
    }    

    // Tell the service controller we are started
    g_ServiceStatus.dwControlsAccepted = SERVICE_ACCEPT_STOP;
    g_ServiceStatus.dwCurrentState = SERVICE_RUNNING;
    g_ServiceStatus.dwWin32ExitCode = 0;
    g_ServiceStatus.dwCheckPoint = 0;

    if (SetServiceStatus (g_StatusHandle, &g_ServiceStatus) == FALSE)
    {
	    OutputDebugString(_T("CCR Service: ServiceMain: SetServiceStatus returned error"));
    }

    // Start the thread that will perform the main task of the service
    HANDLE hThread = CreateThread (NULL, 0, ServiceWorkerThread, NULL, 0, NULL);

    OutputDebugString(_T("CCR Service: ServiceMain: Waiting for Worker Thread to complete"));

    // Wait until our worker thread exits effectively signaling that the service needs to stop
    WaitForSingleObject (hThread, INFINITE);
    
    OutputDebugString(_T("CCR Service: ServiceMain: Worker Thread Stop Event signaled"));
    
    
    /* 
     * Perform any cleanup tasks
     */
    OutputDebugString(_T("CCR Service: ServiceMain: Performing Cleanup Operations"));

    CloseHandle (g_ServiceStopEvent);

    g_ServiceStatus.dwControlsAccepted = 0;
    g_ServiceStatus.dwCurrentState = SERVICE_STOPPED;
    g_ServiceStatus.dwWin32ExitCode = 0;
    g_ServiceStatus.dwCheckPoint = 3;

    if (SetServiceStatus (g_StatusHandle, &g_ServiceStatus) == FALSE)
    {
	    OutputDebugString(_T("CCR Service: ServiceMain: SetServiceStatus returned error"));
    }
    
    EXIT:
    OutputDebugString(_T("CCR Service: ServiceMain: Exit"));

    return;
}


VOID WINAPI ServiceCtrlHandler (DWORD CtrlCode)
{
    OutputDebugString(_T("CCR Service: ServiceCtrlHandler: Entry"));

    switch (CtrlCode) 
	{
     case SERVICE_CONTROL_STOP :

        OutputDebugString(_T("CCR Service: ServiceCtrlHandler: SERVICE_CONTROL_STOP Request"));

        if (g_ServiceStatus.dwCurrentState != SERVICE_RUNNING)
           break;

        /* 
         * Perform tasks neccesary to stop the service here 
         */
        TerminateProcess(pi.hProcess, 0);

        // 500 ms timeout; use INFINITE for no timeout
        WaitForSingleObject(pi.hProcess, 500);

        // Close process and thread handles. 
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        


        g_ServiceStatus.dwControlsAccepted = 0;
        g_ServiceStatus.dwCurrentState = SERVICE_STOP_PENDING;
        g_ServiceStatus.dwWin32ExitCode = 0;
        g_ServiceStatus.dwCheckPoint = 4;

        if (SetServiceStatus (g_StatusHandle, &g_ServiceStatus) == FALSE)
		{
			OutputDebugString(_T("CCR Service: ServiceCtrlHandler: SetServiceStatus returned error"));
		}

        // This will signal the worker thread to start shutting down
        SetEvent (g_ServiceStopEvent);

        break;

     default:
         break;
    }

    OutputDebugString(_T("CCR Service: ServiceCtrlHandler: Exit"));
}


DWORD WINAPI ServiceWorkerThread (LPVOID lpParam)
{
    OutputDebugString(_T("CCR Service: ServiceWorkerThread: Entry"));

    //  Periodically check if the service has been requested to stop
    while (WaitForSingleObject(g_ServiceStopEvent, 0) != WAIT_OBJECT_0)
    {        
        /* 
         * Perform main service function here
         */
        if (!service_started)
        {
            chdir("C:\\dev\\src\\CyberCortex.AI\\core\\bin");
            //std::filesystem::path cwd = std::filesystem::current_path();
            //chdir(cwd.u8string().c_str());

            // start the program up
            char lpApplicationName[2048] = "App_CycCoreTwin.exe";
            char argv[2048] = "App_CycCoreTwin.exe ..\\etc\\pipelines\\control\\scout_sim.conf";
            
            service_started = CreateProcess(lpApplicationName,   // the path
                argv,           // Command line
                NULL,           // Process handle not inheritable
                NULL,           // Thread handle not inheritable
                FALSE,          // Set handle inheritance to FALSE
                0,              // No creation flags
                NULL,           // Use parent's environment block
                NULL,           // Use parent's starting directory 
                &si,            // Pointer to STARTUPINFO structure
                &pi             // Pointer to PROCESS_INFORMATION structure (removed extra parentheses)
            );
        }
    }

    OutputDebugString(_T("CCR Service: ServiceWorkerThread: Exit"));

    return ERROR_SUCCESS;
}