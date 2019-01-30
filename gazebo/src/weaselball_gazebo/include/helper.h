#ifndef _HELPER_H_
#define _HELPER_H_

#include <algorithm>
#include <initializer_list>

template <typename T>
bool is_in(const T& v, std::initializer_list<T> lst)
{
    return std::find(std::begin(lst), std::end(lst), v) != std::end(lst);
}
#endif // _HELPER_H_
