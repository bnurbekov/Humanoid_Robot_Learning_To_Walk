#ifndef _ROS_SUB_BUS_H_
#define _ROS_SUB_BUS_H_
/* Read only - STARTS */
#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifndef _DEFINED_TYPEDEF_FOR_SL_Bus_cpg_optimized_geometry_msgs_Quaternion_
#define _DEFINED_TYPEDEF_FOR_SL_Bus_cpg_optimized_geometry_msgs_Quaternion_ 
typedef struct {
	real_T X;
	real_T Y;
	real_T Z;
	real_T W;
} SL_Bus_cpg_optimized_geometry_msgs_Quaternion;
#endif


#ifndef _DEFINED_TYPEDEF_FOR_SL_Bus_cpg_optimized_geometry_msgs_Point_
#define _DEFINED_TYPEDEF_FOR_SL_Bus_cpg_optimized_geometry_msgs_Point_ 
typedef struct {
	real_T X;
	real_T Y;
	real_T Z;
} SL_Bus_cpg_optimized_geometry_msgs_Point;
#endif


#ifndef _DEFINED_TYPEDEF_FOR_SL_Bus_cpg_optimized_geometry_msgs_Pose_
#define _DEFINED_TYPEDEF_FOR_SL_Bus_cpg_optimized_geometry_msgs_Pose_ 
typedef struct {
	SL_Bus_cpg_optimized_geometry_msgs_Point Position;
	SL_Bus_cpg_optimized_geometry_msgs_Quaternion Orientation;
} SL_Bus_cpg_optimized_geometry_msgs_Pose;
#endif


#ifndef _DEFINED_TYPEDEF_FOR_SL_Bus_cpg_optimized_geometry_msgs_Vector3_
#define _DEFINED_TYPEDEF_FOR_SL_Bus_cpg_optimized_geometry_msgs_Vector3_ 
typedef struct {
	real_T X;
	real_T Y;
	real_T Z;
} SL_Bus_cpg_optimized_geometry_msgs_Vector3;
#endif


#ifndef _DEFINED_TYPEDEF_FOR_SL_ROS_SUB_MSG_
#define _DEFINED_TYPEDEF_FOR_SL_ROS_SUB_MSG_ 
typedef struct {
	SL_Bus_cpg_optimized_geometry_msgs_Pose SdfMidFoot1;
	SL_Bus_cpg_optimized_geometry_msgs_Vector3 FootBF1;
	real_T Joints[30];
	real_T ComEst[3];
	real_T CopEst[2];
	SL_Bus_cpg_optimized_geometry_msgs_Vector3 FootBF0;
	SL_Bus_cpg_optimized_geometry_msgs_Pose SdfMidFoot0;
} SL_ROS_SUB_MSG;
#endif

/* Read only - ENDS */

#endif
