#ifndef CHECK_PTR_H
#define CHECK_PTR_H

#include <memory>

template <typename T>
bool check_ptr(std::shared_ptr<T> p)
{
    return p.operator bool();
}

template <typename T, typename... U>
bool check_ptr(std::shared_ptr<T> p, U... args)
{
    return check_ptr(p) && check_ptr(args...);
}

template <typename T>
bool check_ptr(const std::unique_ptr<T>& p)
{
    return p.operator bool();
}

template <typename T, typename... U>
bool check_ptr(const std::unique_ptr<T>& p, const std::unique_ptr<U>&... args)
{
    return check_ptr(p) && check_ptr(args...);
}

#endif // CHECK_PTR_H
