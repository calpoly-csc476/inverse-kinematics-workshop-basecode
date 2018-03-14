
#pragma once

#include "Util.h"

#include "glm/glm.hpp"


glm::vec3 HSV(float const H, float const S, float const V)
{
	// Convert hsv floats ([0-1],[0-1],[0-1]) to rgb floats ([0-1],[0-1],[0-1]), from Foley & van Dam p593
	// also http://en.wikipedia.org/wiki/HSL_and_HSV
	// Copied from imgui.cpp

	//
	// The MIT License (MIT)
	// 
	// Original Work Copyright (c) 2014-2015 Omar Cornut and ImGui contributors
	// Modified Work Copyright (c) 2017
	// 
	// Permission is hereby granted, free of charge, to any person obtaining a copy
	// of this software and associated documentation files (the "Software"), to deal
	// in the Software without restriction, including without limitation the rights
	// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	// copies of the Software, and to permit persons to whom the Software is
	// furnished to do so, subject to the following conditions:
	// 
	// The above copyright notice and this permission notice shall be included in all
	// copies or substantial portions of the Software.
	// 
	// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	// SOFTWARE.
	//

	if (S == 0.0f)
	{
		// gray
		return glm::vec3(V);
	}

	float const Hue = fmodf(H, 1.0f) / (60.0f/360.0f);
	int   i = (int) Hue;
	float f = Hue - (float)i;
	float p = V * (1.0f - S);
	float q = V * (1.0f - S * f);
	float t = V * (1.0f - S * (1.0f - f));

	switch (i)
	{
	case 0:
		return glm::vec3(V, t, p);
	case 1:
		return glm::vec3(q, V, p);
	case 2:
		return glm::vec3(p, V, t);
	case 3:
		return glm::vec3(p, q, V);
	case 4:
		return glm::vec3(t, p, V);
	case 5:
	default:
		return glm::vec3(V, p, q);
	}
}

float frand()
{
	return std::rand() / (float) RAND_MAX;
}

float nrand()
{
	return (std::rand() / (float) RAND_MAX) * 2.f - 1.f;
}
