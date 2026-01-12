// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CFileUtils.h"

CFileUtils::CFileUtils()
{
	
}

CFileUtils::~CFileUtils()
{

}

bool CFileUtils::FileExist(const char* fileName)
{
	FILE * infile;
	bool bReturn = true;
	infile = fopen(fileName, "r");

	if (infile == NULL)
	{
		//fclose (infile);
		bReturn = false;
	}
	else
	{
		fclose(infile);
		bReturn = true;
	}

	return bReturn;
}

bool CFileUtils::exists(const std::string &filePath)
{
	bool fileExists = false;
	std::ifstream in(filePath.c_str(), std::ios::in);
	if (in.is_open())
	{
		fileExists = true;
		in.close();
	}
	return fileExists;
}

bool CFileUtils::FolderExist(const char* folderName)
{
    return fs::is_directory(folderName);
}

CyC_LONG CFileUtils::length(const std::string &filePath)
{
	long fileSize = 0;
	FILE* fp = 0;
#ifdef _MSC_VER
	fopen_s(&fp, filePath.c_str(), "rb");
#else
	fp = fopen(filePath.c_str(), "rb");
#endif
	if (fp == NULL)
	{
		return 0;
	}

	fseek(fp, 0, SEEK_END);
	fileSize = ftell(fp);
	fclose(fp);

	return fileSize;
}

CyC_INT CFileUtils::erase(const std::string &filePath)
{
	return std::remove(filePath.c_str());
}

CyC_INT CFileUtils::rename(const std::string &oldFilePath,
	const std::string &newFilePath)
{
	return std::rename(oldFilePath.c_str(), newFilePath.c_str());
}

std::string CFileUtils::getFileName(const std::string & filePath)
{
	std::string fullPath = filePath;
	std::string name;
	for (CyC_INT i = (CyC_INT)fullPath.size() - 1; i >= 0; --i)
	{
		if (fullPath[i] == '/' || fullPath[i] == '\\')
			break;
		else
			name.insert(name.begin(), fullPath[i]);
	}
	return name;
}

std::string CFileUtils::getFolder(const std::string& filePath)
{
	size_t pos = filePath.find_last_of("\\/");
	return (std::string::npos == pos)
		? ""
		: filePath.substr(0, pos);
}

std::string CFileUtils::getExtension(const std::string &filePath)
{
	std::vector<std::string> tokens;
	CStringUtils::splitstring(filePath, ".", tokens);

	if (tokens.size())
		return tokens.back();

	return "";
}

void CFileUtils::onLoaded(const char *fileName)
{
    spdlog::info("Entered onLoaded!");
}

void CFileUtils::onError(const char *fileName)
{
    spdlog::info("Entered onError!");
}
