#pragma once
#include <map>
#include <string>

namespace robotto_msgs{
	const int NO_OBJECT = -1;
	enum atwork_object {ATWORK_START=100,
		F20_20_B=ATWORK_START, // 100
		F20_20_G, // 101
		S40_40_B, // 102
		S40_40_G, // 103
		M20_100,  // 104
		M20,      // 105
		M30, 	  // 106
		R20, 	  // 107
		ATWORK_END};
	enum rockin_object {ROCKIN_START=200,
		BEARING_BOX=ROCKIN_START, // 200
        BEARING,       // 201
		AXIS, 		   // 202
		DISTANCE_TUBE, // 203
		MOTOR, 		   // 204
		ROCKIN_END};
	enum ppt_cavity {PPT_ATWORK_START=300,
		PPT_H_F20_20=PPT_ATWORK_START, // 300
		PPT_H_S40_40,  // 301
		PPT_H_M20_100, // 302
		PPT_H_M20,     // 303
		PPT_H_M30,     // 304
		PPT_H_R20,     // 305
		PPT_V_F20_20,  // 306
		PPT_V_S40_40,  // 307
		PPT_V_M20_100, // 308
		PPT_V_M20,     // 309
		PPT_V_M30,     // 310
		PPT_V_R20,     // 311
		PPT_ATWORK_END};
	enum container {CONTAINER_START=400,
		RED_CONTAINER=CONTAINER_START, // 401
		BLUE_CONTAINER, // 402
		CONTAINER_END};

	extern const std::map<int, std::string> objString;

	extern std::string objstr(const int id);
}
