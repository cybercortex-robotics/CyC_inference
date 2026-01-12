// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <iostream>
#include <iomanip>
#include <string>
#include <exception>
#include <vector>
#include <algorithm>
#include <iterator>
#include <libconfig.h++>
#include <typeinfo>
#include <map>
#include "os/CyC_FILESYSTEM.h"
#include "CConfigParameters.h"
#include "CConfigCheck.h"
#include "CyC_TYPES.h"

namespace CConfigCheck
{

using CustomParameters = std::map<std::string, std::string>;

struct ConfigFilter
{
	CycDatablockKey key = { -1, -1 };
	std::string name;
	bool is_active = false;
	bool is_publishable = false;
	bool is_network_filter = false;
	bool does_replay_from_db = false;
	CycDatablockKeys input_sources;
	CustomParameters custom_parameters;
};

struct ConfigRoot
{
	int core_id = 0;
	std::string replay_db_path;
	std::string log_file_path;
	std::vector<ConfigFilter> filters;
};

static bool read_filters_config(
	const libconfig::Setting& config_filters,
	std::vector<ConfigFilter>& filters_out, 
	int& core_id
)
{
	filters_out.clear();

	for(const auto &filter: config_filters)
	{
		ConfigFilter tmp_filter;

		filter.lookupValue("Active", tmp_filter.is_active);
		/* Skip filter if it's inactive. */
		if(!tmp_filter.is_active)
			continue;

		tmp_filter.name = filter.getName();
		tmp_filter.key.nCoreID = core_id;

		filter.lookupValue("ID", tmp_filter.key.nFilterID);
		filter.lookupValue("IsPublishable", tmp_filter.is_publishable);
		filter.lookupValue("ReplayFromDB", tmp_filter.does_replay_from_db);
		filter.lookupValue("IsNetworkFilter", tmp_filter.is_network_filter);

		// Read the input sources
		const libconfig::Setting& input_sources = config_filters[tmp_filter.name.c_str()]["InputSources"];
		for(const auto &input_source: input_sources)
		{
			CycDatablockKey InputSourceParams;
			// lookupValue doesn't work with CyC_ULONG ??. So we use an int.
			int tmp;
			input_source.lookupValue("CoreID", tmp);
			InputSourceParams.nCoreID = tmp;
			input_source.lookupValue("FilterID", tmp);
			InputSourceParams.nFilterID = tmp;

			tmp_filter.input_sources.push_back(InputSourceParams);
		}

		// Read the filter parameters
		const libconfig::Setting& params = config_filters[tmp_filter.name.c_str()]["Parameters"];
		for(const auto &param: params)
		{
			std::string name, value;
			param.lookupValue("name", name);
			param.lookupValue("value", value);
			tmp_filter.custom_parameters[name] = value;
		}

		filters_out.push_back(tmp_filter);
	}

	return true;
}

static bool check_filter_input_sources(
	const std::vector<ConfigFilter> &filters,
	const ConfigFilter &curr_filter,
	const size_t &core_id,
	const std::string &config_file
)
{
	for(const auto& input: curr_filter.input_sources)
	{
		if(input.nCoreID != core_id)
			continue;

		bool found = false;
		for(const auto &filter: filters)
		{
			/* Don't compare against itself. */
			if(filter.key.nFilterID == curr_filter.key.nFilterID)
				continue;

			if(input.nFilterID == filter.key.nFilterID)
			{
				found = true;
				break;
			}
		}

		if(!found)
		{
			std::cout 
				<< config_file << ": [" << curr_filter.name << "] Could not find active InputSource with id " 
				<< input.nFilterID << ".\n";
			return false;
		}
	}

	return true;
}

static bool check_filter_parameters(
	const ConfigFilter &filter,
	const std::string &config_file
)
{
	for(const auto &param: filter.custom_parameters)
	{
		if(param.second.find('/') != std::string::npos)
		{
			fs::path path = fs::path(CConfigParameters::instance().getBasePath()) / fs::path(param.second);

			if ((param.second.substr(0, 6) != "wss://") &&	// ignore signaling uri
				(param.second.substr(0, 5) != "ws://") &&   // ignore signaling uri
				(param.second.substr(0, 6) != "ftp://") &&	// ignore ftp
				(param.second.substr(0, 7) != "sftp://") &&	// ignore sftp
				(param.second.substr(0, 7) != "http://") &&	// ignore http
				(param.second.substr(0, 8) != "https://") &&// ignore https
				(param.second.substr(0, 8) != "/dev/tty"))	// ignore tty port (used in Linux)
			{
				if (!exists(path))
				{
					std::cout
						<< config_file << ": [" << filter.name << "] Invalid path in parameter "
						<< param.first << " = \"" << path << "\"\n";
					return false;
				}
			}
			
			if(param.second.substr(param.second.size() - 5, 5) == ".conf")
			{
				if(!CConfigCheck::check(path))
					return false;
			}
		}
	}
	return true;
}

static bool check_filter_config(
	ConfigRoot& config_root, 
	std::string base_path
)
{
	std::vector<ConfigFilter>& FiltersConfiguration = config_root.filters;

	for (const auto& filter : FiltersConfiguration)
	{
		//std::cout << "Found filter: " << filter.sName << " with id: " << filter.key.nFilterID << '\n';
		if(!check_filter_input_sources(FiltersConfiguration, filter, config_root.core_id, base_path))
			return false;

		if(!check_filter_parameters(filter, base_path))
			return false;
	}
	
	return true;
}

bool exists(const std::string &configfile)
{
	return fs::exists(configfile);
}

bool check(const std::string &configfile)
{
	ConfigRoot config_root;

	if (!CConfigCheck::exists(configfile))
	{
		std::cout << "Config file \"" << configfile << "\" not found.\n";
		return false;
	}

	libconfig::Config configFile;
	try
	{
		// std::cout << confFile.c_str() << std::endl;
		configFile.readFile(configfile.c_str());
	}
	catch (libconfig::ParseException& ex)
	{
		std::cout << "Failed to read configuration with error: " << std::endl;
		std::cout << ex.getError() << " at line " << ex.getLine() << std::endl;
		return false;
	}

	const libconfig::Setting& rootConfig = configFile.getRoot();

	if(rootConfig.exists("Core"))
	{
		rootConfig["Core"].lookupValue("ID", config_root.core_id);
		rootConfig["Core"].lookupValue("ReplayDB", config_root.replay_db_path);

		std::string log_file = "log.txt";
		rootConfig["Core"].lookupValue("LogFile", log_file);
		config_root.log_file_path = std::move(log_file);
	}

	if(rootConfig.exists("Filters"))
	{
		const libconfig::Setting& Filters = rootConfig["Filters"];
		if(!read_filters_config(Filters, config_root.filters, config_root.core_id))
			return false;
	}

	return check_filter_config(config_root, configfile);
}

}