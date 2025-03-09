////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021 Mateusz Malinowski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include "ahrs/ahrs_math.h"

/** Scale to convert angles from radians to degrees. */
static const float RAD_TO_DEG = 180.0f / M_PI;

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(const float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void quatToAngles(IMUData& data)
{
    data.mAngles[1] = 2 * (data.mQuat[0] * data.mQuat[2] - data.mQuat[1] * data.mQuat[3]);
    if (fabs(data.mAngles[1]) >= 1)
    {
        data.mAngles[1] = std::copysign(M_PI / 2, data.mAngles[1]) * RAD_TO_DEG;
    }
    else
    {
        data.mAngles[1] = asin(data.mAngles[1]) * RAD_TO_DEG;
    }
    data.mAngles[0] = atan2(2 * (data.mQuat[0] * data.mQuat[1] + data.mQuat[2] * data.mQuat[3]), 1 - 2 * (data.mQuat[1] * data.mQuat[1] + data.mQuat[2] * data.mQuat[2])) * RAD_TO_DEG;
    data.mAngles[2] = atan2(2 * (data.mQuat[0] * data.mQuat[3] + data.mQuat[1] * data.mQuat[2]), 1 - 2 * (data.mQuat[2] * data.mQuat[2] + data.mQuat[3] * data.mQuat[3])) * RAD_TO_DEG;
}
