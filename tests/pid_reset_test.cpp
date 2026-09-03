#include "PID.h"
#include <cstdio>
#include <type_traits>
#include <utility>

template<class T, class = void> struct HasReset : std::false_type {};
template<class T> struct HasReset<T, decltype(std::declval<T&>().Reset(), void())> : std::true_type {};

template<class T>
typename std::enable_if<!HasReset<T>::value, int>::type Run() {
    std::puts("FAIL: PID::Reset is not implemented");
    return 1;
}
template<class T>
typename std::enable_if<HasReset<T>::value, int>::type Run() {
    T dirty(10.f, 1.f, 1.5f, 0.5f), fresh(10.f, 1.f, 1.5f, 0.5f);
    dirty.Position(50.f, 10000.f);
    dirty.Position(-20.f, 10000.f);
    dirty.Filter(20.f);
    dirty.Reset();
    for (const auto e : dirty.m_error) if (e != 0.f) return 1;
    if (dirty.m_Kp != 10.f || dirty.m_Ti != 1.f || dirty.m_Td != 1.5f) return 1;
    if (dirty.Position(5.f, 10000.f) != fresh.Position(5.f, 10000.f)) return 1;
    if (dirty.Filter(10.f) != fresh.Filter(10.f)) return 1;
    dirty.Reset();
    dirty.Reset();
    if (dirty.Position(0.f, 10000.f) != 0.f) return 1;
    std::puts("PASS PID::Reset: errors, derivative, filter, gains and repeat reset");
    return 0;
}
int main() { return Run<PID>(); }
