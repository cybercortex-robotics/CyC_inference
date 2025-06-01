// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CyC_CACHE_H
#define CyC_CACHE_H

#include "CyC_TYPES.h"
#include <unordered_map>
#include <set>
#include "nonstd/any.hpp"

class CCycCache
{
public:
    using key_t = CyC_TIME_UNIT;

    CCycCache()
        : m_size(CACHE_SIZE)
    {}

    CCycCache(size_t size)
        : m_size(size)
    {}

    CCycCache(const CCycCache&) = default;
    CCycCache(CCycCache&&) noexcept = default;
    CCycCache& operator=(const CCycCache&) = default;
    CCycCache& operator=(CCycCache&&) noexcept = default;
    ~CCycCache() = default;

    template <typename value_t>
    typename std::enable_if<!std::is_same<CcrOcTree, value_t>::value, void>::type
    insert(key_t key, const value_t& value, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT> sync = std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>())
    {
        while (m_cache_keys.size() >= m_size)
        {
            m_cache_data.erase(*m_cache_keys.begin());
            m_cache_keys.erase(m_cache_keys.begin());
            m_cache_inputs.erase(m_cache_inputs.begin());
        }

        m_cache_data.emplace(key, value);
        m_cache_keys.emplace(key);
        m_cache_inputs.emplace(key, sync);
    }

    template <typename value_t>
    typename std::enable_if<std::is_same<CcrOcTree, value_t>::value, void>::type
    insert(key_t key, const value_t& value, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT> sync = std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>())
    {
        while (m_cache_keys.size() >= m_size)
        {
            m_cache_data.erase(*m_cache_keys.begin());
            m_cache_keys.erase(m_cache_keys.begin());
            m_cache_inputs.erase(m_cache_inputs.begin());
        }

        // Construct empty environment
        m_cache_data[key] = CcrOcTree(0.1);
        
        // Get a reference
        // CcrOcTree& to = std::any_cast<CcrOcTree&>(m_cache_data[key]);
        CcrOcTree& to = *(m_cache_data[key].to_ptr<CcrOcTree>());
        
        // Correctly copy the environment
        to.~CcrOcTree();
        new (&to) CcrOcTree(value.getResolution());

        // Information about color is lost when the copy constructor/operator is used
        // Maybe implement an efficient copy constructor for ColorOcTree?
        for (auto it = value.begin_leafs(); it != value.end_leafs(); ++it)
        {
            auto* node = to.updateNode(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z(), true, true);
            node->setColor(it->getColor());
            node->setValue(it->getValue());
            node->setObjectClass(it->getObjectClass());
        }

        m_cache_keys.emplace(key);
        m_cache_inputs.emplace(key, sync);
    }

    size_t size() const
    {
        return m_cache_keys.size();
    }

    bool empty() const
    {
        return size() == 0;
    }

    void clear()
    {
        m_cache_data.clear();
        m_cache_keys.clear();
        m_cache_inputs.clear();
    }

    void erase(key_t key)
    {
        m_cache_data.erase(key);
        m_cache_keys.erase(key);
    }

    void resize(size_t new_size)
    {
        while (m_cache_keys.size() > new_size)
        {
            m_cache_data.erase(*m_cache_keys.begin());
            m_cache_keys.erase(m_cache_keys.begin());
            m_cache_inputs.erase(m_cache_inputs.begin());
        }

        m_size = new_size;
    }

    size_t count(key_t key) const
    {
        return m_cache_data.count(key);
    }

    template <typename value_t>
    value_t& front()
    {
        return at<value_t>(*m_cache_keys.begin());
    }

    template <typename value_t>
    value_t& back()
    {
        return at<value_t>(*m_cache_keys.rbegin());
    }

    template <typename value_t>
    value_t& at(key_t key)
    {
        return nonstd::any_cast<value_t&>(m_cache_data.at(key));
    }

    template <typename value_t>
    const value_t& front() const
    {
        return at<value_t>(*m_cache_keys.begin());
    }

    template <typename value_t>
    const value_t& back() const
    {
        return at<value_t>(*m_cache_keys.rbegin());
    }

    template <typename value_t>
    const value_t& at(key_t key) const
    {
        return nonstd::any_cast<const value_t&>(m_cache_data.at(key));
    }

    template <typename value_t>
    bool has_type(value_t&&) const
    {
        return empty()
            ? false
            : m_cache_data.begin()->second.type() == typeid(value_t);
    }

    const std::set<key_t>& keys() const
    {
        return m_cache_keys;
    }

    const std::unordered_map<CycDatablockKey, key_t>& sync(key_t key) const
    {
        return m_cache_inputs.at(key);
    }

private:
    size_t m_size = 0;

    std::unordered_map<key_t, nonstd::any> m_cache_data;
    std::set<key_t>                        m_cache_keys;

    // Stores the timestamps of the input sources, which were used to compute the data
    std::unordered_map<key_t, std::unordered_map<CycDatablockKey, key_t>> m_cache_inputs;
};

#endif // CyC_CACHE_H
