// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CyC_FILESYSTEM_H
#define CyC_FILESYSTEM_H

#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include) && !defined(__ANDROID__)  && !defined(RASPBERRY_PI)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs {
using namespace std::filesystem;
using ifstream = std::ifstream;
using ofstream = std::ofstream;
using fstream = std::fstream;
}
#endif // GHC_USE_STD_FS
#endif // .....
#ifndef GHC_USE_STD_FS
#include <os/ghc/filesystem.hpp>
namespace fs {
using namespace ghc::filesystem;
using ifstream = ghc::filesystem::ifstream;
using ofstream = ghc::filesystem::ofstream;
using fstream = ghc::filesystem::fstream;
} 
#endif // GHC_USE_STD_FS

#endif // CyC_FILESYSTEM_H