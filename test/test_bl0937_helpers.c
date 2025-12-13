#include "unity.h"
#include "bl0937_filter.h"
#include <math.h>
#include <stdint.h>

TEST_CASE("EMA basic behavior", "[bl0937]") {
    float x = 10.0f;
    float y = bl0937_ema(NAN, x, 0.2f);
    TEST_ASSERT_EQUAL_FLOAT(x, y);

    float y2 = bl0937_ema(10.0f, 20.0f, 0.5f);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, y2);
}

TEST_CASE("Notify deadband triggers", "[bl0937]") {
    uint32_t last = 0;
    uint32_t now = 1000;

    bool n1 = bl0937_should_notify(0.0f, 0.0f, 0.5f, now, &last, 500);
    TEST_ASSERT_TRUE(n1);

    uint32_t last2 = last;
    bool n2 = bl0937_should_notify(10.0f, 10.1f, 0.5f, now+100, &last2, 10000);
    TEST_ASSERT_FALSE(n2);

    bool n3 = bl0937_should_notify(10.0f, 11.0f, 0.5f, now+200, &last2, 10000);
    TEST_ASSERT_TRUE(n3);
}
