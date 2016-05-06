// Win32Project1.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME ROS_sub
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
/* %%%-SFUNWIZ_defines_Changes_BEGIN --- EDIT HERE TO _END */
#define NUM_INPUTS           0

#define NUM_OUTPUTS          2
/* Output Port  0 */
#define OUT_PORT_0_NAME      isNew
#define OUTPUT_0_WIDTH       1
#define OUTPUT_DIMS_0_COL    1
#define OUTPUT_0_DTYPE       real_T
#define OUTPUT_0_COMPLEX     COMPLEX_NO
#define OUT_0_FRAME_BASED    FRAME_NO
#define OUT_0_BUS_BASED      0
#define OUT_0_BUS_NAME       
#define OUT_0_DIMS           1-D
#define OUT_0_ISSIGNED        1
#define OUT_0_WORDLENGTH      8
#define OUT_0_FIXPOINTSCALING 1
#define OUT_0_FRACTIONLENGTH  3
#define OUT_0_BIAS            0
#define OUT_0_SLOPE           0.125
/* Output Port  1 */
#define OUT_PORT_1_NAME      msg
#define OUTPUT_1_WIDTH       1
#define OUTPUT_DIMS_1_COL    1
#define OUTPUT_1_DTYPE       real_T
#define OUTPUT_1_COMPLEX     COMPLEX_NO
#define OUT_1_FRAME_BASED    FRAME_NO
#define OUT_1_BUS_BASED      1
#define OUT_1_BUS_NAME       SL_ROS_SUB_MSG
#define OUT_1_DIMS           1-D
#define OUT_1_ISSIGNED        1
#define OUT_1_WORDLENGTH      8
#define OUT_1_FIXPOINTSCALING 1
#define OUT_1_FRACTIONLENGTH  3
#define OUT_1_BIAS            0
#define OUT_1_SLOPE           0.125

#define NPARAMS              0

#define SAMPLE_TIME_0        INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES      0
#define DISC_STATES_IC       [0]
#define NUM_CONT_STATES      0
#define CONT_STATES_IC       [0]

#define SFUNWIZ_GENERATE_TLC 1
#define SOURCEFILES "__SFB__"
#define PANELINDEX           6
#define USE_SIMSTRUCT        0
#define SHOW_COMPILE_STEPS   0                   
#define CREATE_DEBUG_MEXFILE 0
#define SAVE_CODE_ONLY       1
#define SFUNWIZ_REVISION     3.0

#include "simstruc.h"
#include "ROS_sub.h"

#include <ros/ros.h>
#include <wrecs_msgs/sf_state_est.h>
#include <atomic>
//#include "mex.h"

/*
* Code Generation Environment flag (simulation or standalone target).
 */
static int_T isSimulationTarget;
/*  Utility function prototypes. */
static int_T GetRTWEnvironmentMode(SimStruct *S);
/* Macro used to check if Simulation mode is set to accelerator */
#define isDWorkPresent (((!ssRTWGenIsCodeGen(S) || isSimulationTarget) && !ssIsExternalSim(S))  || ssIsRapidAcceleratorActive(S))

ros::Subscriber sub;
ros::AsyncSpinner* spinner;

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{

	DECL_AND_INIT_DIMSINFO(outputDimsInfo);
	ssSetNumSFcnParams(S, NPARAMS);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return; /* Parameter mismatch will be reported by Simulink */
	}

	ssSetNumContStates(S, NUM_CONT_STATES);
	ssSetNumDiscStates(S, NUM_DISC_STATES);


	if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;

	if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
	/* Output Port 0 */
	ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
	ssSetOutputPortDataType(S, 0, SS_DOUBLE);
	ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);
	/* Output Port 1 */

	/* Register SL_ROS_SUB_MSG datatype for Output port 1 */

#if defined(MATLAB_MEX_FILE)
	if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY)
	{
		DTypeId dataTypeIdReg;
		ssRegisterTypeFromNamedObject(S, "SL_ROS_SUB_MSG", &dataTypeIdReg);
		if(dataTypeIdReg == INVALID_DTYPE_ID) return;
		ssSetOutputPortDataType(S,1, dataTypeIdReg);
	}
#endif

	ssSetBusOutputObjectName(S, 1, (void *) "SL_ROS_SUB_MSG");
	ssSetOutputPortWidth(S, 1, OUTPUT_1_WIDTH);
	ssSetOutputPortComplexSignal(S, 1, OUTPUT_1_COMPLEX);
	ssSetBusOutputAsStruct(S, 1, OUT_1_BUS_BASED);
	ssSetOutputPortBusMode(S, 1, SL_BUS_MODE);
	if (ssRTWGenIsCodeGen(S)) {
		isSimulationTarget = GetRTWEnvironmentMode(S);
		if (isSimulationTarget == -1) {
			ssSetErrorStatus(S, " Unable to determine a valid code generation environment mode");
			return;
		}
		isSimulationTarget |= ssRTWGenIsModelReferenceSimTarget(S);
	}

	/* Set the number of dworks */
	if (!isDWorkPresent) {
		if (!ssSetNumDWork(S, 0)) return;
	}
	else {
		if (!ssSetNumDWork(S, 1)) return;
	}


	if (isDWorkPresent) {

		/*
		* Configure the dwork 0 (y1BUS)
		*/
#if defined(MATLAB_MEX_FILE)

		if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
			DTypeId dataTypeIdReg;
			ssRegisterTypeFromNamedObject(S, "SL_ROS_SUB_MSG", &dataTypeIdReg);
			if (dataTypeIdReg == INVALID_DTYPE_ID) return;
			ssSetDWorkDataType(S, 0, dataTypeIdReg);
		}

#endif

		ssSetDWorkUsageType(S, 0, SS_DWORK_USED_AS_DWORK);
		ssSetDWorkName(S, 0, "y1BUS");
		ssSetDWorkWidth(S, 0, DYNAMICALLY_SIZED);
		ssSetDWorkComplexSignal(S, 0, COMPLEX_NO);
	}
	ssSetNumSampleTimes(S, 1);
	ssSetNumRWork(S, 0);
	ssSetNumIWork(S, 0);
	ssSetNumPWork(S, 0);
	ssSetNumModes(S, 0);
	ssSetNumNonsampledZCs(S, 0);

	ssSetSimulinkVersionGeneratedIn(S, "8.7");

	/* Take care when specifying exception free code - see sfuntmpl_doc.c */
	ssSetOptions(S, 0);
}




/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */

SL_ROS_SUB_MSG stored_msg;
std::atomic_flag lock = ATOMIC_FLAG_INIT;
bool newMessageArrived = false;
void robotPoseCallback(const wrecs_msgs::sf_state_est::ConstPtr& msg)
{
	if (!lock.test_and_set()) {
		for (int i = 0; i < 30; i++) {
			stored_msg.Joints[i] = msg->joints[i];
		}

		for (int i = 0; i < 3; i++) {
			stored_msg.ComEst[i] = msg->com_est[i];
		}

		for (int i = 0; i < 2; i++) {
			stored_msg.CopEst[i] = msg->cop_est[i];
		}


		stored_msg.FootBF0.X = msg->foot_b_F[0].x;
		stored_msg.FootBF0.Y = msg->foot_b_F[0].y;
		stored_msg.FootBF0.Z = msg->foot_b_F[0].z;

		stored_msg.FootBF1.X = msg->foot_b_F[1].x;
		stored_msg.FootBF1.Y = msg->foot_b_F[1].y;
		stored_msg.FootBF1.Z = msg->foot_b_F[1].z;

		stored_msg.SdfMidFoot0.Position.X = msg->sdf_mid_foot[0].position.x;
		stored_msg.SdfMidFoot0.Position.Y = msg->sdf_mid_foot[0].position.y;
		stored_msg.SdfMidFoot0.Position.Z = msg->sdf_mid_foot[0].position.z;

		stored_msg.SdfMidFoot0.Orientation.W = msg->sdf_mid_foot[0].orientation.w;
		stored_msg.SdfMidFoot0.Orientation.X = msg->sdf_mid_foot[0].orientation.x;
		stored_msg.SdfMidFoot0.Orientation.Y = msg->sdf_mid_foot[0].orientation.y;
		stored_msg.SdfMidFoot0.Orientation.Z = msg->sdf_mid_foot[0].orientation.z;

		stored_msg.SdfMidFoot1.Position.X = msg->sdf_mid_foot[1].position.x;
		stored_msg.SdfMidFoot1.Position.Y = msg->sdf_mid_foot[1].position.y;
		stored_msg.SdfMidFoot1.Position.Z = msg->sdf_mid_foot[1].position.z;

		stored_msg.SdfMidFoot1.Orientation.W = msg->sdf_mid_foot[1].orientation.w;
		stored_msg.SdfMidFoot1.Orientation.X = msg->sdf_mid_foot[1].orientation.x;
		stored_msg.SdfMidFoot1.Orientation.Y = msg->sdf_mid_foot[1].orientation.y;
		stored_msg.SdfMidFoot1.Orientation.Z = msg->sdf_mid_foot[1].orientation.z;

        newMessageArrived = true;

		lock.clear();
	}
}

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
	//ROS stuff
	int argc = 0;
	char** argv = NULL;
	ros::init(argc, argv, "MATLAB_ros_sub");

    spinner = new ros::AsyncSpinner(2);
    spinner->start();
	ros::NodeHandle n;

    sub = n.subscribe("/cmu/robot_pose", 1000, robotPoseCallback);

	/* Bus Information */
	slDataTypeAccess *dta = ssGetDataTypeAccess(S);
	const char *bpath = ssGetPath(S);
	DTypeId SL_Bus_cpg_optimized_geometry_msgs_PointId = ssGetDataTypeId(S, "SL_Bus_cpg_optimized_geometry_msgs_Point");
	DTypeId SL_Bus_cpg_optimized_geometry_msgs_PoseId = ssGetDataTypeId(S, "SL_Bus_cpg_optimized_geometry_msgs_Pose");
	DTypeId SL_Bus_cpg_optimized_geometry_msgs_QuaternionId = ssGetDataTypeId(S, "SL_Bus_cpg_optimized_geometry_msgs_Quaternion");
	DTypeId SL_Bus_cpg_optimized_geometry_msgs_Vector3Id = ssGetDataTypeId(S, "SL_Bus_cpg_optimized_geometry_msgs_Vector3");
	DTypeId SL_ROS_SUB_MSGId = ssGetDataTypeId(S, "SL_ROS_SUB_MSG");

	int_T *busInfo = (int_T *)malloc(46 * sizeof(int_T));
	if (busInfo == NULL) {
		ssSetErrorStatus(S, "Memory allocation failure");
		return;
	}

	/* Calculate offsets of all primitive elements of the bus */

	busInfo[0] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 2);
	busInfo[1] = 30 * dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[2] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 3);
	busInfo[3] = 3 * dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[4] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 4);
	busInfo[5] = 2 * dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[6] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PointId, 0);
	busInfo[7] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[8] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PointId, 1);
	busInfo[9] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[10] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PointId, 2);
	busInfo[11] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[12] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_QuaternionId, 0);
	busInfo[13] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[14] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_QuaternionId, 1);
	busInfo[15] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[16] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_QuaternionId, 2);
	busInfo[17] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[18] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_QuaternionId, 3);
	busInfo[19] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[20] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_Vector3Id, 0);
	busInfo[21] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[22] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_Vector3Id, 1);
	busInfo[23] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[24] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_Vector3Id, 2);
	busInfo[25] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[26] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 5) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_Vector3Id, 0);
	busInfo[27] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[28] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 5) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_Vector3Id, 1);
	busInfo[29] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[30] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 5) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_Vector3Id, 2);
	busInfo[31] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[32] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 6) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PointId, 0);
	busInfo[33] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[34] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 6) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PointId, 1);
	busInfo[35] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[36] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 6) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 0) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PointId, 2);
	busInfo[37] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[38] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 6) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_QuaternionId, 0);
	busInfo[39] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[40] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 6) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_QuaternionId, 1);
	busInfo[41] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[42] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 6) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_QuaternionId, 2);
	busInfo[43] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	busInfo[44] = dtaGetDataTypeElementOffset(dta, bpath, SL_ROS_SUB_MSGId, 6) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_PoseId, 1) + dtaGetDataTypeElementOffset(dta, bpath, SL_Bus_cpg_optimized_geometry_msgs_QuaternionId, 3);
	busInfo[45] = dtaGetDataTypeSize(dta, bpath, ssGetDataTypeId(S, "double"));
	ssSetUserData(S, busInfo);
  }
#endif /*  MDL_START */

#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)
{
	ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
	ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}

#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)

static void mdlSetWorkWidths(SimStruct *S)
{
	/* Set the width of DWork(s) used for marshalling the IOs */
	if (isDWorkPresent) {

		/* Update dwork 0 */
		ssSetDWorkWidth(S, 0, ssGetOutputPortWidth(S, 1));

	}

}

#endif

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{    
	real_T *isNew = (real_T *)ssGetOutputPortRealSignal(S, 0);
    SL_ROS_SUB_MSG *msg = (SL_ROS_SUB_MSG *)ssGetOutputPortSignal(S, 1);    
	int_T* busInfo = (int_T *)ssGetUserData(S);
    
    //mexPrintf("Acquiring lock...");
	while (lock.test_and_set());
    //mexPrintf("Acquired lock. Outputing message...");
    isNew[0] = (int)newMessageArrived;

    //memcpy(msg, &stored_msg, sizeof(SL_ROS_SUB_MSG));
    if (newMessageArrived) {
		*msg = stored_msg;
        
		/*Copy temporary structure into output bus*/
		/*(void)memcpy(msg + busInfo[0], stored_msg.Joints, busInfo[1]);
		(void)memcpy(msg + busInfo[2], stored_msg.ComEst, busInfo[3]);
		(void)memcpy(msg + busInfo[4], stored_msg.CopEst, busInfo[5]);
		(void)memcpy(msg + busInfo[6], &stored_msg.SdfMidFoot1.Position.X, busInfo[7]);
		(void)memcpy(msg + busInfo[8], &stored_msg.SdfMidFoot1.Position.Y, busInfo[9]);
		(void)memcpy(msg + busInfo[10], &stored_msg.SdfMidFoot1.Position.Z, busInfo[11]);
		(void)memcpy(msg + busInfo[12], &stored_msg.SdfMidFoot1.Orientation.X, busInfo[13]);
		(void)memcpy(msg + busInfo[14], &stored_msg.SdfMidFoot1.Orientation.Y, busInfo[15]);
		(void)memcpy(msg + busInfo[16], &stored_msg.SdfMidFoot1.Orientation.Z, busInfo[17]);
		(void)memcpy(msg + busInfo[18], &stored_msg.SdfMidFoot1.Orientation.W, busInfo[19]);
		(void)memcpy(msg + busInfo[20], &stored_msg.FootBF1.X, busInfo[21]);
		(void)memcpy(msg + busInfo[22], &stored_msg.FootBF1.Y, busInfo[23]);
		(void)memcpy(msg + busInfo[24], &stored_msg.FootBF1.Z, busInfo[25]);
		(void)memcpy(msg + busInfo[26], &stored_msg.FootBF0.X, busInfo[27]);
		(void)memcpy(msg + busInfo[28], &stored_msg.FootBF0.Y, busInfo[29]);
		(void)memcpy(msg + busInfo[30], &stored_msg.FootBF0.Z, busInfo[31]);
		(void)memcpy(msg + busInfo[32], &stored_msg.SdfMidFoot0.Position.X, busInfo[33]);
		(void)memcpy(msg + busInfo[34], &stored_msg.SdfMidFoot0.Position.Y, busInfo[35]);
		(void)memcpy(msg + busInfo[36], &stored_msg.SdfMidFoot0.Position.Z, busInfo[37]);
		(void)memcpy(msg + busInfo[38], &stored_msg.SdfMidFoot0.Orientation.X, busInfo[39]);
		(void)memcpy(msg + busInfo[40], &stored_msg.SdfMidFoot0.Orientation.Y, busInfo[41]);
		(void)memcpy(msg + busInfo[42], &stored_msg.SdfMidFoot0.Orientation.Z, busInfo[43]);
		(void)memcpy(msg + busInfo[44], &stored_msg.SdfMidFoot0.Orientation.W, busInfo[45]);*/
    }
    
    newMessageArrived = false;
    lock.clear();
}



#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
	/*Free stored bus information*/
	int_T *busInfo = (int_T *)ssGetUserData(S);
	if (busInfo != NULL) {
		free(busInfo);
	}

	delete spinner;
}

static int_T GetRTWEnvironmentMode(SimStruct *S)
{
	int_T status = -1;
	mxArray *plhs[1];
	mxArray *prhs[1];
	int_T err;

	/*
	* Get the name of the Simulink block diagram
	*/
	prhs[0] = mxCreateString(ssGetModelName(ssGetRootSS(S)));
	plhs[0] = NULL;

	/*
	* Call "isSimulationTarget = rtwenvironmentmode(modelName)" in MATLAB
	*/
	mexSetTrapFlag(1);
	err = mexCallMATLAB(1, plhs, 1, prhs, "rtwenvironmentmode");
	mexSetTrapFlag(0);
	mxDestroyArray(prhs[0]);

	/*
	* Set the error status if an error occurred
	*/
	if (err) {
		if (plhs[0]) {
			mxDestroyArray(plhs[0]);
			plhs[0] = NULL;
		}
		ssSetErrorStatus(S, "Unknown error during call to 'rtwenvironmentmode'.");
		return -1;
	}

	/*
	* Get the value returned by rtwenvironmentmode(modelName)
	*/
	if (plhs[0]) {
		status = (int_T)(mxGetScalar(plhs[0]) != 0);
		mxDestroyArray(plhs[0]);
		plhs[0] = NULL;
	}

	return (status);
}

/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

