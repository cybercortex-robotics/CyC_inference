// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CFILEUTILS_H_
#define CFILEUTILS_H_

#include "CyC_TYPES.h"
#include <list>
#include <fstream>
#include "CStringUtils.h"
#include "os/CyC_FILESYSTEM.h"

class CFileUtils
{
public:
	CFileUtils();
	virtual ~CFileUtils();

	static bool FileExist(const char* fileName);

	static bool exists(const std::string &filePath);

    static bool FolderExist(const char* folderName);

	static CyC_LONG length(const std::string &filePath);

	static CyC_INT erase(const std::string &filePath);

	static CyC_INT rename(const std::string &oldFilePath, const std::string &newFilePath);

	static std::string getFileName(const std::string& filePath);

	static std::string getFolder(const std::string & filePath);

	static std::string getExtension(const std::string &filePath);

	static void onLoaded(const char* fileName);

	static void onError(const char* fileName);
};
#endif /* CFILEUTILS_H_ */
