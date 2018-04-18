#pragma once
#include <math.h>
#include "maths_funcs.h"

class Hand
{
	//local positions of:
	//origin, skull/goal, 1st finger's root, 1st finger, 2nd finger's root, 2nd finger, 3rd finger's root, 3rd finger, fingertip, palm
	mat4 local, local0, local1, local2, local3, local4, local5, local6, local7, local10;
	//to keep each joint's rotation in a limit
	float alreadyRotDeg5 = 0.0;
	float alreadyRotDeg3 = 0.0;
	float alreadyRotDeg1 = 0.0;

	float deg1, deg2, deg3;
	int tried;
	
	vec3 skull; //the goal

public:

	mat4 global, global0, global1, global2, global3, global4, global5, global6, global7, global10;

	Hand(const mat4& origin, const float& L0, const float& L1, const float& L2, const float& L3, const vec3& goal, const int& max_try);
	~Hand();
};

