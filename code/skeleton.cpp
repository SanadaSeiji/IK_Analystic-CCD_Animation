
#include "stdafx.h"
#include "skeleton.h"

#define M_PI       3.14159265358979323846   // pi
#define ONE_DEG_IN_RAD (2.0 * M_PI) / 360.0 // 0.017444444
#define ONE_RAD_IN_DEG 57.2957795


skeleton::skeleton(const float& L1, const float& L2, const vec3& goal, const float& B1, float& t)
{
	setBones(L1, L2, goal, B1, t);
}


skeleton::~skeleton()
{
}

void skeleton::setBones(const float& L1, const float& L2, const vec3& goal, const float& B1, float& t) {

	//origin
	local = identity_mat4();
	global = local;
	//torso
	local10 = identity_mat4();
	local10 = translate(local10, vec3(B1, 0.0, 0.0));
	global10 = global*local10;
	//root1's position
	local1 = identity_mat4();
	global1 = global * local1;
	//upperArm's position
	local2 = identity_mat4();
	local2 = translate(local2, vec3(0.0, 0.0, 0.5*L1));
	global2 = global1 * local2;
	//joint between upper and lower arms
	local3 = identity_mat4();
	local3 = translate(local3, vec3(0.0, 0.0, 0.5*L1));
	global3 = global2 * local3;
	//lowerArm's position
	local4 = identity_mat4();
	local4 = translate(local4, vec3(0.0, 0.0, 0.5*L2));
	global4 = global3 * local4;
	//end factor - useful for hand
	local5 = identity_mat4();
	local5 = translate(local5, vec3(0.0, 0.0, 0.5*L2));
	global5 = global4 * local5;
	// goal's position
	local0 = identity_mat4();
	local0 = translate(local0, goal);

	//transform back to origin
	global0 = global * local0;

	//------------------calculate IK thetas-----------------------------
	vec3 thetas = IKanalystic(L1, L2, goal);
	theta1 = thetas.v[0];
	theta2 = thetas.v[1];
	theta6 = thetas.v[2];

	//this is when you move the arms
	lowerZ = L1*cos(theta1 * ONE_DEG_IN_RAD) - 0.5*L1*cos(theta1 * ONE_DEG_IN_RAD);
	lowerZ = lowerZ < 0 ? -lowerZ : lowerZ;

	//first move to change y axis
	local1 = rotate_x_deg(local1, -theta6);
	global1 = global * local1;
	global2 = global1 * local2;
	global3 = global2 * local3;
	global4 = global3 * local4;
	global5 = global4 * local5;

	//----------------------spline----------------------------
	//cubic bezier
	//4 control point
	//now pos
	p0 = vec3(0.0, 0.0, 0.0);
	//desired pos //later get from IK
	p3 = vec3(0.0, 0.0, 0.0);
	//4 control point: using goal's position and ist section 
	//control for x-z plain for now
	p1 = vec3(L1, 0.0, L1);
	p2 = vec3(goal.v[0], 0.0, goal.v[2]);
	//control for y
	float p4 = L1;
	float p5 = goal.v[1];
	//calculate interpolate position for every frame
	// manupilation of t does not work in update() ???
	//3D Bezier cubic:
	vec3 interP = cubicBezier3D(p0, p1, p2, p3, p4, p5, t);
	//vec3 interP = cubicBezier(p0, p1, p2, p3, t);

	//origin fly
	local = translate(local1, interP);
	global = local;
	//torso follow origin
	global10 = global * local10;

	//really transform rotate root joint
	//first rotate
	//rotate around origin
	local1 = rotate_y_deg(local1, -theta1 * t); //then transform back
												//then translate
	local1 = translate(local1, p3);

	global1 = global * local1;
	//upperarm follows
	global2 = global1 * local2;
	//global3 = global2 * local3;

	//lowerArm joint
	p0 = vec3(0.0, 0.0, 0.0);
	//desired pos //later get from IK
	p3 = vec3(0.0, 0.0, lowerZ);
	//3D Bezier cubic:
	interP = cubicBezier3D(p0, p1, p2, p3, p4, p5, t);
	//move back to origin
	local3 = translate(local3, vec3(0.0, 0.0, -0.5*L1));
	//rotate
	local3 = rotate_y_deg(local3, theta2 * t);
	local3 = translate(local3, p3);
	global3 = global2 *local3;
	//lower arm follows
	global4 = global3*local4;
	global5 = global4 * local5;

}

