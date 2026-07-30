// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <iostream>
#include <iomanip>
#include <string>
#include <exception>
#include <vector>
#include <algorithm>
#include <iterator>
#pragma warning(disable : 4275)
#include <libconfig.h++>
#pragma warning(default : 4275)
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

static bool read_input_sources(
	const libconfig::Setting& filter,
	const int& core_id,
	const std::string& config_file,
	ConfigFilter& filter_out
)
{
	if(!filter.exists("InputSources"))
		return true;

	for(const auto &input_source: filter["InputSources"])
	{
		CycDatablockKey InputSourceParams;

		// An InputSource without an explicit CoreID refers to a filter of the current
		// core (same convention as CConfigParameters::readFiltersConfiguration).
		InputSourceParams.nCoreID = static_cast<CyC_ULONG>(core_id);
		if(input_source.exists("CoreID"))
		{
			const libconfig::Setting& source_core = input_source.lookup("CoreID");
			if(source_core.getType() == libconfig::Setting::TypeInt64)
				InputSourceParams.nCoreID = static_cast<CyC_ULONG>(static_cast<long long>(source_core));
			else if(source_core.getType() == libconfig::Setting::TypeInt)
				InputSourceParams.nCoreID = static_cast<CyC_ULONG>(static_cast<int>(source_core));
			else
			{
				std::cout
					<< config_file << ": [" << filter_out.name << "] InputSource with a non-integer CoreID.\n";
				return false;
			}
		}

		// lookupValue doesn't work with CyC_ULONG ??. So we use an int.
		int filter_id = -1;
		if(!input_source.lookupValue("FilterID", filter_id))
		{
			std::cout
				<< config_file << ": [" << filter_out.name << "] InputSource without a valid FilterID.\n";
			return false;
		}
		InputSourceParams.nFilterID = filter_id;

		input_source.lookupValue("Description", InputSourceParams.sDescription);

		filter_out.input_sources.push_back(InputSourceParams);
	}

	return true;
}

static void read_custom_parameters(
	const libconfig::Setting& filter,
	ConfigFilter& filter_out
)
{
	if(!filter.exists("Parameters"))
		return;

	for(const auto &param: filter["Parameters"])
	{
		std::string name, value;
		param.lookupValue("name", name);
		param.lookupValue("value", value);
		filter_out.custom_parameters[name] = value;
	}
}

static bool read_filters_config(
	const libconfig::Setting& config_filters,
	std::vector<ConfigFilter>& filters_out,
	int& core_id,
	const std::string& config_file
)
{
	filters_out.clear();

	for(const auto &filter: config_filters)
	{
		ConfigFilter tmp_filter;

		filter.lookupValue("Active", tmp_filter.is_active);

		tmp_filter.name = filter.getName();
		tmp_filter.key.nCoreID = core_id;

		filter.lookupValue("ID", tmp_filter.key.nFilterID);
		filter.lookupValue("IsPublishable", tmp_filter.is_publishable);
		filter.lookupValue("ReplayFromDB", tmp_filter.does_replay_from_db);
		filter.lookupValue("IsNetworkFilter", tmp_filter.is_network_filter);

		/* Inactive filters are kept in the list, so that an InputSource pointing to
		   one of them can be reported as inactive instead of as missing. Only their
		   own sources and parameters are of no interest. */
		if(tmp_filter.is_active)
		{
			if(!read_input_sources(filter, core_id, config_file, tmp_filter))
				return false;

			read_custom_parameters(filter, tmp_filter);
		}

		filters_out.push_back(tmp_filter);
	}

	return true;
}

static bool check_filter_input_sources(
	const std::vector<ConfigFilter> &filters,
	const ConfigFilter &curr_filter,
	const int &core_id,
	const std::string &config_file
)
{
	for(const auto& input: curr_filter.input_sources)
	{
		/* Sources belonging to another core are declared in that core's config file. */
		if(input.nCoreID != static_cast<CyC_ULONG>(core_id))
			continue;

		const ConfigFilter* source = nullptr;
		for(const auto &filter: filters)
		{
			/* Don't compare against itself. */
			if(filter.key.nFilterID == curr_filter.key.nFilterID)
				continue;

			if(input.nFilterID == filter.key.nFilterID)
			{
				source = &filter;
				break;
			}
		}

		if(source == nullptr)
		{
			std::cout
				<< config_file << ": [" << curr_filter.name << "] Could not find InputSource with id "
				<< input.nFilterID << " (\"" << input.sDescription << "\").\n";
			return false;
		}

		if(!source->is_active)
		{
			std::cout
				<< config_file << ": [" << curr_filter.name << "] InputSource with id "
				<< input.nFilterID << " (\"" << input.sDescription << "\") is not Active.\n";
			return false;
		}
	}

	return true;
}

// A path-valued parameter may carry several files as a comma-separated list
// (e.g. robot_config = "a.conf, b.conf"), so split on ',' and validate each entry
// on its own. A single path yields one (trimmed) entry, leaving that case unchanged.
static std::vector<std::string> split_path_list(const std::string &value)
{
	std::vector<std::string> out;
	std::size_t start = 0;

	while (true)
	{
		std::size_t const comma = value.find(',', start);
		std::size_t const end = (comma == std::string::npos) ? value.size() : comma;

		std::string const token = value.substr(start, end - start);
		auto const first = token.find_first_not_of(" \t");
		auto const last = token.find_last_not_of(" \t");
		if (first != std::string::npos)
			out.push_back(token.substr(first, last - first + 1));

		if (comma == std::string::npos)
			break;
		start = comma + 1;
	}

	return out;
}

static bool check_filter_parameters(
	const ConfigFilter &filter,
	const std::string &config_file
)
{
	for(const auto &param: filter.custom_parameters)
	{
		if(param.second.find('/') == std::string::npos)
			continue;

		// The value may list several files, comma-separated; check each one.
		for(const std::string &entry: split_path_list(param.second))
		{
			fs::path path = fs::path(CConfigParameters::instance().getBasePath()) / fs::path(entry);

			if ((entry.substr(0, 6) != "wss://") &&		// ignore signaling uri
				(entry.substr(0, 5) != "ws://") &&		// ignore signaling uri
				(entry.substr(0, 6) != "ftp://") &&		// ignore ftp
				(entry.substr(0, 7) != "sftp://") &&	// ignore sftp
				(entry.substr(0, 7) != "http://") &&	// ignore http
				(entry.substr(0, 8) != "https://") &&	// ignore https
				(entry.substr(0, 8) != "/dev/tty") &&	// ignore tty port (used in Linux)
				(entry.substr(0, 11) != "/dev/serial"))	// ignore serial port (used in Linux)
			{
				if (!exists(path))
				{
					std::cout
						<< config_file << ": [" << filter.name << "] Invalid path in parameter "
						<< param.first << " = \"" << path << "\"\n";
					return false;
				}
			}

			if(entry.size() >= 5 && entry.compare(entry.size() - 5, 5, ".conf") == 0)
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
		/* Inactive filters are only kept as possible InputSource targets. */
		if(!filter.is_active)
			continue;

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
		if(!read_filters_config(Filters, config_root.filters, config_root.core_id, configfile))
			return false;
	}

	return check_filter_config(config_root, configfile);
}

}