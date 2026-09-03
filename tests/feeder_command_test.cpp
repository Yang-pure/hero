#include <cstdio>
#include <cstdint>

#if __has_include("feeder_command.h")
#include "feeder_command.h"
static int checks = 0;
#define CHECK(expr) do { ++checks; if (!(expr)) { std::printf("FAIL line %d: %s\n", __LINE__, #expr); return 1; } } while (0)

static void Arm(FeederCommand& command, uint32_t tick) {
    command.OnFeedback(tick);
    command.OnRcFrame(2, 2, 0, tick);
    command.Resolve(true, tick);
}

// Generic policy cases explicitly use native motor polarity.
// Installed feed polarity is verified separately in feeder_direction_test.
static FeederCommand NativeDirection() {
    FeederCommand c;
    c.direction = 1;
    return c;
}

int main() {
    FeederCommand c = NativeDirection();
    CHECK(c.Resolve(true, 0) == 0 && !c.enabled);
    for (uint8_t left = 1; left <= 3; ++left) {
        for (uint8_t right = 1; right <= 3; ++right) {
            c = NativeDirection();
            Arm(c, 10);
            c.OnRcFrame(left, right, 660, 11);
            CHECK(c.Resolve(true, 11) == ((left == 2 && right == 2) ? 300 : 0));
        }
    }
    c = NativeDirection();
    c.OnFeedback(0);
    c.OnRcFrame(2, 2, 660, 0);
    CHECK(c.Resolve(true, 0) == 0 && !c.armed); // no startup jump
    Arm(c, 1);
    CHECK(c.enabled);
    c.OnRcFrame(2, 2, 660, 2);
    CHECK(c.Resolve(true, 2) == 300);
    c.OnRcFrame(2, 2, -660, 3);
    CHECK(c.Resolve(true, 3) == -300);
    c.OnRcFrame(2, 2, 330, 4);
    CHECK(c.Resolve(true, 4) == 145); // deadband removed, not a fixed-speed switch
    c.OnRcFrame(2, 2, -330, 5);
    CHECK(c.Resolve(true, 5) == -145);
    for (int16_t axis = -20; axis <= 20; ++axis) {
        c.OnRcFrame(2, 2, axis, 6);
        CHECK(c.Resolve(true, 6) == 0);
    }
    c.direction = -1;
    c.OnRcFrame(2, 2, 660, 7);
    CHECK(c.Resolve(true, 7) == -300);
    c.max_speed = 100;
    CHECK(c.Resolve(true, 7) == -100);
    CHECK(c.Resolve(false, 7) == 0 && !c.armed);
    CHECK(c.Resolve(true, 8) == 0); // exit requires neutral again
    Arm(c, 8);
    c.OnRcFrame(2, 2, 661, 9);
    CHECK(c.Resolve(true, 9) == 0 && !c.enabled);
    Arm(c, 10);
    c.OnRcFrame(0, 2, 100, 11);
    CHECK(c.Resolve(true, 11) == 0 && !c.enabled);
    c = NativeDirection();
    Arm(c, 0);
    c.OnRcFrame(2, 2, 660, 1);
    c.OnFeedback(99);
    CHECK(c.Resolve(true, 100) == 300); // age=99
    CHECK(c.Resolve(true, 101) == 0 && !c.enabled); // age=100
    c.OnRcFrame(2, 2, 660, 102);
    c.OnFeedback(102);
    CHECK(c.Resolve(true, 102) == 0); // reconnect while held
    Arm(c, 103);
    c.OnRcFrame(2, 2, -660, 104);
    CHECK(c.Resolve(true, 104) == -300);
    c = NativeDirection();
    Arm(c, 1);
    c.OnRcFrame(2, 2, 660, 100);
    CHECK(c.Resolve(true, 100) == 300); // feedback age=99
    CHECK(c.Resolve(true, 101) == 0); // motor feedback loss
    c.OnFeedback(102);
    CHECK(c.Resolve(true, 102) == 0); // no automatic restart
    Arm(c, 103);
    c.OnRcFrame(2, 2, 660, 104);
    CHECK(c.Resolve(true, 104) == 300);
    c = NativeDirection();
    Arm(c, UINT32_MAX - 30);
    c.OnRcFrame(2, 2, 660, UINT32_MAX - 20);
    CHECK(c.Resolve(true, 5) == 300);
    CHECK(c.Resolve(true, 100) == 0);
    c = NativeDirection();
    Arm(c, 0);
    c.OnRcFrame(2, 2, 660, 1);
    c.OnRcFrame(2, 2, 660, 150); // timeout even if Resolve wasn't polled
    c.OnFeedback(150);
    CHECK(c.Resolve(true, 150) == 0);
    c = NativeDirection();
    Arm(c, 0);
    c.deadband = 660;
    CHECK(c.Resolve(true, 0) == 0 && !c.enabled);
    c = NativeDirection();
    Arm(c, 0);
    c.max_speed = -1;
    CHECK(c.Resolve(true, 0) == 0 && !c.enabled);
    c = NativeDirection();
    Arm(c, 0);
    c.direction = 0;
    CHECK(c.Resolve(true, 0) == 0 && !c.enabled);
    std::printf("PASS feeder_command: %d checks\n", checks);
    return 0;
}
#else
int main() {
    std::puts("FAIL: FeederCommand is not implemented");
    return 1;
}
#endif
