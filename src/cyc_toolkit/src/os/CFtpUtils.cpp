// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CFtpUtils.h"
#include <curl/curl.h>
#include <filesystem>

// Callback function to read from the file
size_t ftp_read_callback(void* ptr, size_t size, size_t nmemb, FILE* stream)
{
    return fread(ptr, size, nmemb, stream);
}

bool CFtpUtils::upload_file(const std::string& _url,
    const std::string& _local_file_path,
    const std::string& _credentials)
{
    // Command line example using curl
    // curl -v -T debug.txt ftp://cyber@cybercortex.ai/debug.txt --user cyber:CyberTeam

    bool bReturn = false;

    // Initialize libcurl
    curl_global_init(CURL_GLOBAL_DEFAULT);
    CURL* curl = curl_easy_init();
    
    if (curl)
    {
        // Set the FTP URL
        curl_easy_setopt(curl, CURLOPT_URL, _url.c_str());

        // Set the username and password
        curl_easy_setopt(curl, CURLOPT_USERPWD, _credentials.c_str());

        // Enable the creation of missing directories
        curl_easy_setopt(curl, CURLOPT_FTP_CREATE_MISSING_DIRS, CURLFTP_CREATE_DIR);

        // Set the upload option
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

        // Set the read function to read the file during upload
        curl_easy_setopt(curl, CURLOPT_READFUNCTION, ftp_read_callback);

        // Open the local file to upload
        FILE* file = fopen(_local_file_path.c_str(), "rb"); // Replace with the path to your local file
        if (!file)
        {
            spdlog::error("FTP: Failed to open local file '{}'", _local_file_path.c_str());
            return false;
        }

        // Get the file size
        fseek(file, 0, SEEK_END);
        long file_size = ftell(file);
        fseek(file, 0, SEEK_SET);

        // Set the file size for the upload
        curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, (curl_off_t)file_size);

        // Set the file to upload
        curl_easy_setopt(curl, CURLOPT_READDATA, file);

        // Perform the FTP upload
        CURLcode res = curl_easy_perform(curl);

        // Check for errors
        if (res != CURLE_OK)
            spdlog::error("FTP: curl_easy_perform() failed: {}", curl_easy_strerror(res));
        else
            bReturn = true;

        // Cleanup
        fclose(file);
        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();

    return bReturn;
}
