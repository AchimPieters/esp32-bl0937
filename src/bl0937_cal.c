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

#include "bl0937_cal.h"

esp_err_t bl0937_calculate_calibration(const bl0937_reading_t *s,
                                       const bl0937_cal_ref_t *r,
                                       bl0937_config_t *cfg) {
    if (!s || !r || !cfg) return ESP_ERR_INVALID_ARG;

    if (r->ref_voltage_v > 0.0f && s->voltage_v > 0.0f) {
        cfg->cal_vrms *= (r->ref_voltage_v / s->voltage_v);
    }
    if (r->ref_current_a > 0.0f && s->current_a > 0.0f) {
        cfg->cal_irms *= (r->ref_current_a / s->current_a);
    }
    if (r->ref_power_w > 0.0f && s->power_w > 0.0f) {
        cfg->cal_power *= (r->ref_power_w / s->power_w);
    }
    return ESP_OK;
}
