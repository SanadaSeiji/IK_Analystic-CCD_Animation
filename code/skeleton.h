#pragma once
#include <math.h>
#include "maths_funcs.h"
class skeleton
{
	//origin
	mat4 local, local0, local1, local2, local3, local4, local5, local10;
	float theta1, theta2, theta6; 
	float lowerZ;
	vec3 p0, p1, p2, p3;
	float p4, p5;
	vec3 skull;

	void setBones(const float& L1, const float& L2, const vec3& goal, const float& B1, float& t);
	//void IKanalysitic(const float& L1, const float& L2, const vec3& goal);
	

public:
	mat4 global, global0, global1, global2, global3, global4, global5, global10;

	skeleton(const float& L1, const float& L2, const vec3& goal, const float& B1, float& t);
	~skeleton();

	
	//void spline(const float& L1, const float& L2, const vec3& goal, float& t);
};

