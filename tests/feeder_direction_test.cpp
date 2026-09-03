#include "feeder.h"
#include "pid.h"
#include <cstdio>
#include <cmath>

#define CHECK(expr) do { if (!(expr)) { std::printf("FAIL direction line %d: %s\n", __LINE__, #expr); return 1; } } while (0)

int main()
{
    FEEDER unit;
    auto& command = unit.command;
    CHECK(unit.max_current == 30000);
    CHECK(command.max_speed == 300);
    command.OnFeedback(0);
    command.OnRcFrame(2, 2, 0, 0);
    CHECK(command.Resolve(true, 0) == 0 && command.enabled);
    // Installed mechanism: NEGATIVE motor RPM feeds, POSITIVE retracts.
    command.OnRcFrame(2, 2, 660, 1);
    CHECK(command.Resolve(true, 1) == -300); // stick up must feed
    command.OnRcFrame(2, 2, 330, 2);
    CHECK(command.Resolve(true, 2) == -145);
    command.OnRcFrame(2, 2, 0, 3);
    CHECK(command.Resolve(true, 3) == 0);
    command.OnRcFrame(2, 2, -660, 4);
    CHECK(command.Resolve(true, 4) == 300); // stick down must retract
    command.OnRcFrame(2, 2, -330, 5);
    CHECK(command.Resolve(true, 5) == 145);
    CHECK(command.Resolve(false, 6) == 0 && !command.enabled);

    // Execute the real PID implementation, not a simulated motor response.
    // This checks sign symmetry; it does NOT claim the loaded wheel can rotate.
    PID positive(10.0f, 0.0f, 1.5f, 0.0f);
    PID negative(10.0f, 0.0f, 1.5f, 0.0f);
    for (int index = 0; index < 50; ++index)
    {
        const float p = positive.Position(300.0f, 10000.0f);
        const float n = negative.Position(-300.0f, 10000.0f);
        CHECK(std::fabs(p + n) < 0.001f);
        CHECK(p > 0.0f && n < 0.0f);
        if (index > 0) CHECK(p == 3000.0f && n == -3000.0f);
    }
    std::puts("PASS installed direction and real PID sign symmetry (50 cycles)");
    return 0;
}
