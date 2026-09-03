#include "feeder.h"
#include <cstdio>
#include <type_traits>
#include <utility>

template<class T, class = void> struct HasDiagnostics : std::false_type {};
template<class T> struct HasDiagnostics<T, std::void_t<
    decltype(std::declval<T>().command.stop_reason),
    decltype(std::declval<T>().command.last_fault),
    decltype(std::declval<T>().CurrentLimit(16384))>> : std::true_type {};
#define CHECK(expr) do { if (!(expr)) { std::printf("FAIL diagnostics line %d: %s\n", __LINE__, #expr); return 1; } } while (0)

template<class T> int Run()
{
    if constexpr (!HasDiagnostics<T>::value)
    {
        std::puts("FAIL: zero-output reason and effective limit diagnostics missing");
        return 1;
    }
    else
    {
        T f;
        using C = std::decay_t<decltype(f.command)>;
        C& c = f.command;
        CHECK(f.max_current == 30000);
        CHECK(f.CurrentLimit(16384) == 16384);
        CHECK(f.CurrentLimit(30000) == 16384); // C620 hard protocol boundary
        CHECK(f.CurrentLimit(6000) == 6000);
        CHECK(f.CurrentLimit(-1) == 0);
        f.max_current = 10000;
        CHECK(f.CurrentLimit(16384) == 10000);
        f.max_current = -1;
        CHECK(f.CurrentLimit(16384) == 0);
        CHECK(c.Resolve(false, 0) == 0 && c.stop_reason == C::NOT_FIRE);
        c.OnRcFrame(2, 2, 660, 0);
        CHECK(c.Resolve(true, 0) == 0 && c.stop_reason == C::WAIT_FEEDBACK);
        c.OnFeedback(1);
        CHECK(c.Resolve(true, 1) == 0 && c.stop_reason == C::WAIT_NEUTRAL);
        c.OnRcFrame(2, 2, 0, 2);
        CHECK(c.Resolve(true, 2) == 0 && c.stop_reason == C::RUNNING && c.enabled);
        c.OnRcFrame(2, 2, 660, 3);
        CHECK(c.Resolve(true, 3) == -300 && c.stop_reason == C::RUNNING);
        c.OnFeedback(102);
        CHECK(c.Resolve(true, 103) == 0 && c.stop_reason == C::RC_TIMEOUT);
        c.OnRcFrame(2, 2, 660, 104);
        CHECK(c.Resolve(true, 104) == 0 && c.stop_reason == C::WAIT_NEUTRAL);
        CHECK(c.last_fault == C::RC_TIMEOUT);
        c.OnRcFrame(2, 2, 0, 105);
        CHECK(c.Resolve(true, 105) == 0 && c.enabled);
        c.OnRcFrame(2, 2, -660, 200);
        CHECK(c.Resolve(true, 202) == 0 && c.stop_reason == C::FEEDBACK_TIMEOUT);
        CHECK(c.last_fault == C::FEEDBACK_TIMEOUT);
        c.OnFeedback(203);
        CHECK(c.Resolve(true, 203) == 0 && c.stop_reason == C::WAIT_NEUTRAL);
        c.OnRcFrame(2, 2, 0, 204);
        CHECK(c.Resolve(true, 204) == 0 && c.enabled);
        c.OnRcFrame(2, 2, 800, 205);
        CHECK(c.Resolve(true, 205) == 0 && c.stop_reason == C::INVALID_RC);
        c.OnRcFrame(2, 2, 0, 206);
        c.direction = 0;
        CHECK(c.Resolve(true, 206) == 0 && c.stop_reason == C::INVALID_CONFIG);
        c.Disarm(C::OVER_TEMPERATURE);
        CHECK(c.last_fault == C::OVER_TEMPERATURE && !c.enabled);
        c.Disarm(C::INVALID_PID);
        CHECK(c.last_fault == C::INVALID_PID);
        std::puts("PASS zero-output reasons, fault retention and C620 current limit");
        return 0;
    }
}
int main() { return Run<FEEDER>(); }
