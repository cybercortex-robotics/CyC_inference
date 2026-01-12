// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "keypoints_parser_func.h"

void get_common_keypoint_ids_from_csv(
    const std::string& filepath,
    const std::vector<std::string>& imgs,
    std::vector<size_t>& ids_out
)
{
    csv::reader reader;
    if (!reader.open(filepath, ','))
    {
        std::cout << "Couldn't open CSV file '" << filepath << "'.\n";
        return;
    }
    
    std::vector<size_t> ids_freq;
    std::string f1, f2, sid, sx1, sy1, sx2, sy2;
    size_t row = 2; // start at 2, skipping the header.

    while (reader.read_row(f1, f2, sid, sx1, sy1, sx2, sy2))
    {
        const size_t cols = reader.get_row().size();
        if (cols != expected_cols)
        {
            std::cout <<
                "Wrong number of columns in CSV file '" << filepath <<
                "' at line " << row << ": expected " << expected_cols << " but got " << cols << ".\n";
            continue;
        }

        bool next = true;
        for (size_t i = 0; i < imgs.size() - 1; ++i)
        {
            if (f1 == imgs[i] && f2 == imgs[i + 1])
            {
                next = false;
                break;
            }
        }

        if (next)
            continue;

        size_t id;
        std::stringstream ss;
        ss << sid;
        ss >> id;

        if (ss.fail())
        {
            std::cout << "Couldn't parse CSV file '" << filepath << "' at line " << row << ".\n";
            continue;
        }

        if (id + 1 > ids_freq.size())
            ids_freq.resize(id + 1);
        ++ids_freq[id];

        ++row;
    }
    
    for (size_t i = 0; i < ids_freq.size(); ++i)
    {
        if (ids_freq[i] == imgs.size() - 1)
            ids_out.push_back(i);
    }
}

/* Returns 0 on success, -1 if the file couldn't be open
or -2 if the CSV file was in any way invalid. */
int get_keypoints_from_csv(
    const std::string& filepath,
    const std::vector<std::string>& imgs,
    std::vector<std::vector<Eigen::Vector2f>> &pts
)
{
    std::vector<std::string> tmpimgs = imgs;
    std::vector<std::vector<Eigen::Vector2f>> tmpvec;

    /* Sort imgs based on their name. */
    for (size_t i = 0; i < tmpimgs.size() - 1; ++i)
    {
        for (size_t k = 0; k < tmpimgs.size() - i - 1; ++k)
        {
            size_t a, b;
            std::stringstream ss;
            ss << tmpimgs[k].substr(0, tmpimgs[k].find('.'));
            ss >> a;
            ss.clear();
            ss << tmpimgs[k + 1].substr(0, tmpimgs[k + 1].find('.'));
            ss >> b;

            if (a > b)
            {
                std::string c = tmpimgs[k];
                tmpimgs[k] = tmpimgs[k + 1];
                tmpimgs[k + 1] = c;
            }
        }
    }

    assert(tmpimgs.size() > 1);
    
    tmpvec.resize(tmpimgs.size());

    std::vector<size_t> common_ids;
    get_common_keypoint_ids_from_csv(filepath, tmpimgs, common_ids);
    
    csv::reader reader;
    if (!reader.open(filepath, ','))
    {
        std::cout << "Couldn't open CSV file '" << filepath << "'.\n";
        return -1;
    }

    std::string f1, f2, sid, sx1, sy1, sx2, sy2;
    size_t row = 2;

    while (reader.read_row(f1, f2, sid, sx1, sy1, sx2, sy2))
    {
        const size_t cols = reader.get_row().size();
        if (cols != expected_cols)
        {
            std::cout <<
                "Wrong number of columns in CSV file '" << filepath <<
                "' at line " << row << ": expected " << expected_cols << " but got " << cols << ".\n";
            return -2;
        }

        bool next = true;
        size_t imgidx = 0;
        for (; imgidx < tmpimgs.size() - 1; ++imgidx)
        {
            if (f1 == tmpimgs[imgidx] && f2 == tmpimgs[imgidx + 1])
            {
                next = false;
                break;
            }
        }

        if (next)
            continue;

        size_t id;
        std::stringstream ss;
        ss << sid;
        ss >> id;

        bool match = false;
        for (auto& i : common_ids)
        {
            if (i == id)
            {
                match = true;
                break;
            }
        }
        
        if (!match)
            continue;

        size_t x1, y1, x2, y2;
        ss.clear();
        ss << sx1 << ' ' << sy1 << ' ' << sx2 << ' ' << sy2;
        ss >> x1 >> y1 >> x2 >> y2;
        
        if (ss.fail())
        {
            std::cout << "Couldn't parse CSV file '" << filepath << "' at line " << row << ".\n";
            return -2;
        }

        if(!imgidx)
            tmpvec[imgidx].emplace_back(Eigen::Vector2f(x1, y1));
        tmpvec[imgidx + 1].emplace_back(Eigen::Vector2f(x2, y2));

        ++row;
    }

    pts.resize(tmpimgs.size());
    for (size_t p = 0; p < imgs.size(); ++p)
    {
        for (size_t i = 0; i < tmpimgs.size(); ++i)
        {
            if (tmpimgs[i] == imgs[p])
                pts[p] = tmpvec[i];
        }
    }

    return 0;
}
