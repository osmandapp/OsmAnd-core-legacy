#pragma once

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <list>
#include <unordered_map>
#include <utility>
#include <vector>

/**
 * Small Java-like LinkedHashSet analogue.
 *
 * Properties:
 * - unique values;
 * - preserves insertion order;
 * - O(1) contains/add/remove by value on average;
 * - iteration goes in insertion order.
 *
 * Requirements:
 * - T must be hashable by Hash;
 * - T must be comparable by KeyEqual.
 */
template <
        typename T,
        typename Hash = std::hash<T>,
        typename KeyEqual = std::equal_to<T>>
class LinkedHashSet
{
private:
    using OrderedList = std::list<T>;
    using OrderedIterator = typename OrderedList::iterator;
    using Index = std::unordered_map<T, OrderedIterator, Hash, KeyEqual>;

public:
    using value_type = T;
    using iterator = typename OrderedList::iterator;
    using const_iterator = typename OrderedList::const_iterator;
    using size_type = typename OrderedList::size_type;

    LinkedHashSet() = default;

    LinkedHashSet(std::initializer_list<T> values)
    {
        addAll(values);
    }

    bool add(const T &value)
    {
        if (contains(value)) {
            return false;
        }

        order.push_back(value);
        OrderedIterator it = std::prev(order.end());
        index.emplace(*it, it);
        return true;
    }

    bool add(T &&value)
    {
        if (contains(value)) {
            return false;
        }

        order.push_back(std::move(value));
        OrderedIterator it = std::prev(order.end());
        index.emplace(*it, it);
        return true;
    }

    template <typename InputIt>
    void addAll(InputIt first, InputIt last)
    {
        for (; first != last; ++first) {
            add(*first);
        }
    }

    template <typename Range>
    void addAll(const Range &values)
    {
        for (const T &value : values) {
            add(value);
        }
    }

    bool contains(const T &value) const
    {
        return index.find(value) != index.end();
    }

    bool containsAll(const LinkedHashSet<T, Hash, KeyEqual> &values) const
    {
        for (const T &value : values) {
            if (!contains(value)) {
                return false;
            }
        }
        return true;
    }

    template <typename Range>
    bool containsAll(const Range &values) const
    {
        for (const T &value : values) {
            if (!contains(value)) {
                return false;
            }
        }
        return true;
    }

    bool remove(const T &value)
    {
        typename Index::iterator found = index.find(value);
        if (found == index.end()) {
            return false;
        }

        order.erase(found->second);
        index.erase(found);
        return true;
    }

    void removeAll(const LinkedHashSet<T, Hash, KeyEqual> &values)
    {
        for (const T &value : values) {
            remove(value);
        }
    }

    template <typename Range>
    void removeAll(const Range &values)
    {
        for (const T &value : values) {
            remove(value);
        }
    }

    void clear()
    {
        order.clear();
        index.clear();
    }

    bool empty() const
    {
        return order.empty();
    }

    size_type size() const
    {
        return order.size();
    }

    const T &front() const
    {
        return order.front();
    }

    const T &back() const
    {
        return order.back();
    }

    std::vector<T> toVector() const
    {
        return std::vector<T>(order.begin(), order.end());
    }

    iterator begin()
    {
        return order.begin();
    }

    iterator end()
    {
        return order.end();
    }

    const_iterator begin() const
    {
        return order.begin();
    }

    const_iterator end() const
    {
        return order.end();
    }

    const_iterator cbegin() const
    {
        return order.cbegin();
    }

    const_iterator cend() const
    {
        return order.cend();
    }

private:
    OrderedList order;
    Index index;
};
