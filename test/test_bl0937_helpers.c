/**
   Copyright 2026 Achim Pieters | StudioPieters®

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   for more information visit https://www.studiopieters.nl
 **/

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
