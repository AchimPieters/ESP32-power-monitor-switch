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

#include "custom_charcteristc.h"

// Public characteristics (used in main.c)
// Eve Energy expects these as numeric (float) characteristics.
homekit_characteristic_t energy_voltage = HOMEKIT_CHARACTERISTIC_(CUSTOM_VOLTAGE, 0.0f);
homekit_characteristic_t energy_current = HOMEKIT_CHARACTERISTIC_(CUSTOM_CURRENT, 0.0f);
homekit_characteristic_t energy_power   = HOMEKIT_CHARACTERISTIC_(CUSTOM_POWER,   0.0f);
homekit_characteristic_t energy_energy  = HOMEKIT_CHARACTERISTIC_(CUSTOM_ENERGY,  0.0f);
