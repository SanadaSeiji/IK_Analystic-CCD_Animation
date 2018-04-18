#include "stdafx.h"
#include "Hand.h"


Hand::Hand(const mat4& origin, const float& L0, const float& L1, const float& L2, const float& L3,  const vec3& goal, const int& max_try)
{
	tried = 0;

	//--------------------------
	//origin
	global = origin; // origin is the end factor of the arm

	//palm on origin
	local10 = identity_mat4();
	global10 = global * local10;

	//1st finger's root position
	local1 = identity_mat4();
	local1 = translate(local1, vec3(0.0, 0.0, L0));//L0 is palm length
	global1 = global * local1;
	//1st finger's  position
	local2 = identity_mat4();
	local2 = translate(local2, vec3(0.0, 0.0, 0.5*L1));
	global2 = global1 * local2;

	//2nd finger's root position
	local3 = identity_mat4();
	local3 = translate(local3, vec3(0.0, 0.0, 0.5*L1));
	global3 = global2 * local3;

	//2nd finger's position
	local4 = identity_mat4();
	local4 = translate(local4, vec3(0.0, 0.0, 0.5*L2));
	global4 = global3 * local4;


	//invisible endPoint's position
	//for global4 is NOT endpoint
	local5 = identity_mat4();
	//local5 = rotate_y_deg(local5, deg3);
	local5 = translate(local5, vec3(0.0, 0.0, 0.5*L2));
	global5 = global4 * local5;

	//3rd fingers position
	local6 = identity_mat4();
	local6 = translate(local6, vec3(0.0, 0.0, 0.5*L3));
	global6 = global5 * local6;

	//end 
	local7 = identity_mat4();
	local7 = translate(local7, vec3(0.0, 0.0, 0.5*L3));
	global7 = global6 * local7;



	// goal's position
	local0 = identity_mat4();
	local0 = translate(local0, goal);


    global0 = local0;

	//----------------------------------
	

	//-----------------------------------------
	while (!((fabsf(global7.m[12] - goal.v[0]) < 0.1) //while goal position hasn't been reached
		&& (fabsf(global7.m[14] - goal.v[2]) < 0.1)) && (tried <  max_try)) {

		
		local5 = handCCD(global5, global7, global0, local5, L2, alreadyRotDeg5);//calculate IK angles of 3 sections using CCD
		//updating world position of children
		global5 = global4 * local5;
		global6 = global5 * local6;
		global7 = global6 * local7;

		//--------------------------
		local3 = handCCD(global3, global7, global0, local3, L1, alreadyRotDeg3);//calculate IK angles of 3 sections using CCD
		//updating world position of children
		global3 = global2 * local3;
		global4 = global3 * local4;
		global5 = global4 * local5;
		global6 = global5 * local6;
		global7 = global6 * local7;

		//---------------------------------------
		
		local1 = handCCD(global1, global7, global0, local1, 2*L0, alreadyRotDeg1);//calculate IK angles of 3 sections using CCD
		//updating world position of children
		global1 = global * local1;
		global2 = global1 * local2;
		global3 = global2 * local3;
		global4 = global3 * local4;
		global5 = global4 * local5;
		global6 = global5 * local6;
		global7 = global6 * local7;


		tried++;
	}




}


Hand::~Hand()
{
}
