//#include <rtapi_string.h>
//#include <stdint.h>
#include <stdlib.h>

#include "rtapi.h"              /* RTAPI realtime OS API */
#include "rtapi_app.h"          /* RTAPI realtime module decls */
#include "hal.h"                /* HAL public API decls */


#include "Fusion/FusionAhrs.h"
#include "Fusion/FusionBias.h"
#include "Fusion/FusionCalibration.h"
#include "Fusion/FusionCompass.h"
#include "Fusion/FusionTypes.h"


/* module information */
MODULE_AUTHOR("Boris Skegin");
MODULE_DESCRIPTION("Madgwick Fusion for IMU Sensor");
MODULE_LICENSE("LGPL");


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

typedef union { float f; uint32_t u; } fu; // defining ionion for float vonversion

/** This structure contains the runtime data for the component.
*/

static int count = 1;           /* number of instances */
RTAPI_MP_INT( count, "number of Madgwick instances");

static int debug = 0;
RTAPI_MP_INT( debug, "debug level");

FusionBias* fusionBias = {0};  // static ?
FusionAhrs* fusionAhrs = {0};

const float samplePeriod = 0.0005f;  // replace this value with actual sample period in seconds, should be sensor period, not HAL period

FusionVector3 gyroscopeSensitivity = {
    .axis.x = 1.0f,
    .axis.y = 1.0f,
    .axis.z = 1.0f,
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

FusionVector3 accelerometerSensitivity = {
    .axis.x = 1.0f,
    .axis.y = 1.0f,
    .axis.z = 1.0f,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

FusionVector3 hardIronBias = {
    .axis.x = 0.0f,
    .axis.y = 0.0f,
    .axis.z = 0.0f,
}; // replace these values with actual hard-iron bias in uT if known


typedef struct {
	
   // Input pins
   hal_u32_t *gyroX;
   hal_u32_t *gyroY;
   hal_u32_t *gyroZ;

   hal_u32_t *accelX;
   hal_u32_t *accelY;
   hal_u32_t *accelZ;

   hal_u32_t *magX;
   hal_u32_t *magY;
   hal_u32_t *magZ;

   // Output pins
   hal_float_t *quatW;
   hal_float_t *quatX;
   hal_float_t *quatY;
   hal_float_t *quatZ;

} hal_madgwick_t;

static  hal_madgwick_t* madgwick_data;	

/* other globals */
static int comp_id;             /* component ID */


/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
static int  madgwick_export( hal_madgwick_t* addr, const char* name);
static void updateAHRS(void *arg, long period);
static void updateAHRS_NonMag(void *arg, long period);
static inline void *lcec_zalloc(size_t size);
static inline float ieee_float(const uint32_t* uRep); 

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main( void)
{
    const char name[] = "madgwick";

    if (debug) {
        rtapi_print_msg( RTAPI_MSG_ERR, "MADGWICK: INFO: madgwick %d starting...\n", count);
    }
	
	/* connect to the HAL */
    comp_id = hal_init(name);
    if (comp_id < 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "MADGEWICK: ERROR: hal_init() failed\n");
        return -1;
    }
	
	 /* allocate shared memory for madgwick data */
    madgwick_data = hal_malloc( sizeof( hal_madgwick_t));
    if (madgwick_data == 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "MADGWICK: ERROR: hal_malloc(1) failed\n");
        hal_exit( comp_id);
        return -1;
    }
	
	/* allocate data for Fusion structures */
	fusionBias = lcec_zalloc(sizeof(FusionBias));
	if ( fusionBias == 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "MADGWICK: ERROR: fusionBias malloc failed\n");
        hal_exit( comp_id);
        return -1;
    }
	
	fusionAhrs = lcec_zalloc(sizeof(FusionAhrs));
	if ( fusionAhrs == 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "MADGWICK: ERROR: fusionAhrs malloc failed\n");
        hal_exit( comp_id);
        return -1;
    }
	
	/* export variables and functions */
    int retval = madgwick_export( madgwick_data, name);

    if (retval != 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "MADGWICK: ERROR: madgwick var export failed\n");
        hal_exit( comp_id);
        return -1;
    }
    rtapi_print_msg( RTAPI_MSG_INFO, "MADGWICK: loaded MADGWICK fusion component\n");
    hal_ready( comp_id);
	
	
	 // Initialise gyroscope bias correction algorithm
    FusionBiasInitialise(fusionBias, 0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(fusionAhrs, 0.5f); // gain = 0.5

    // Set optional magnetic field limits
    FusionAhrsSetMagneticField(fusionAhrs, 20.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT
	
	
	
    return 0;
}


void rtapi_app_exit( void)
{
    free(fusionBias);
	free(fusionAhrs);
	hal_exit( comp_id);
}


/***********************************************************************
            *   Update functions to be exported     *
************************************************************************/

static void updateAHRS(void *arg, long period)
{
    	hal_madgwick_t* m_data = arg;
		// Calibrate gyroscope
        FusionVector3 uncalibratedGyroscope = {
            .axis.x = ieee_float(m_data->gyroX), /* replace this value with actual gyroscope x axis measurement in lsb */
            .axis.y = ieee_float(m_data->gyroY), /* replace this value with actual gyroscope y axis measurement in lsb */
            .axis.z = ieee_float(m_data->gyroZ), /* replace this value with actual gyroscope z axis measurement in lsb */
        };
        FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate accelerometer
        FusionVector3 uncalibratedAccelerometer = {
            .axis.x = ieee_float(m_data->accelX), /* replace this value with actual accelerometer x axis measurement in lsb */
            .axis.y = ieee_float(m_data->accelY), /* replace this value with actual accelerometer y axis measurement in lsb */
            .axis.z = ieee_float(m_data->accelZ), /* replace this value with actual accelerometer z axis measurement in lsb */
        };
        FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate magnetometer
        FusionVector3 uncalibratedMagnetometer = {
            .axis.x = *(m_data->magX)*100.0, /* replace this value with actual magnetometer x axis measurement in uT, mult with 100 because input is in Gauss */
            .axis.y = *(m_data->magY)*100.0, /* replace this value with actual magnetometer y axis measurement in uT */
            .axis.z = *(m_data->magZ)*100.0, /* replace this value with actual magnetometer z axis measurement in uT */
        };
        FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);
		
		   // Update gyroscope bias correction algorithm
        calibratedGyroscope = FusionBiasUpdate(fusionBias, calibratedGyroscope);

        // Update AHRS algorithm
        FusionAhrsUpdate(fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, samplePeriod);
		
	#define Q fusionAhrs->quaternion.element  // define shorthand label for more readable code

        *(m_data->quatW) = Q.w ;
        *(m_data->quatX) = Q.x ;
        *(m_data->quatY) = Q.y ;
        *(m_data->quatZ) = Q.z ;
		
	#undef Q // undefine shorthand label
		
	
}

static void updateAHRS_NonMag(void *arg, long period)
{
	    hal_madgwick_t* m_data = arg;
		// Calibrate gyroscope
        FusionVector3 uncalibratedGyroscope = {
            .axis.x = *(m_data->gyroX), /* replace this value with actual gyroscope x axis measurement in lsb */
            .axis.y = *(m_data->gyroY), /* replace this value with actual gyroscope y axis measurement in lsb */
            .axis.z = *(m_data->gyroZ), /* replace this value with actual gyroscope z axis measurement in lsb */
        };
        FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate accelerometer
        FusionVector3 uncalibratedAccelerometer = {
            .axis.x = *(m_data->accelX), /* replace this value with actual accelerometer x axis measurement in lsb */
            .axis.y = *(m_data->accelY), /* replace this value with actual accelerometer y axis measurement in lsb */
            .axis.z = *(m_data->accelZ), /* replace this value with actual accelerometer z axis measurement in lsb */
        };
        FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);
		
		   // Update gyroscope bias correction algorithm
        calibratedGyroscope = FusionBiasUpdate(fusionBias, calibratedGyroscope);

        // Update AHRS algorithm
        FusionAhrsUpdateWithoutMagnetometer(fusionAhrs, calibratedGyroscope, calibratedAccelerometer,samplePeriod);
		
	#define Q fusionAhrs->quaternion.element  // define shorthand label for more readable code

        *(m_data->quatW) = Q.w ;
        *(m_data->quatX) = Q.x ;
        *(m_data->quatY) = Q.y ;
        *(m_data->quatZ) = Q.z ;
		
	#undef Q // undefine shorthand label
}

static int  madgwick_export( hal_madgwick_t* addr, const char* prefix)
{
	int retval;
	
	// Input pins:

    retval = hal_pin_u32_newf( HAL_IN, &(addr->gyroX), comp_id, "%s.gyroX", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->gyroY), comp_id, "%s.gyroY", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->gyroZ), comp_id, "%s.gyroZ", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->accelX), comp_id, "%s.accelX", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->accelY), comp_id, "%s.accelY", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->accelZ), comp_id, "%s.accelZ", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->magX), comp_id, "%s.magX", prefix);
    if (retval != 0) {
        return retval;
    }
    
	retval = hal_pin_u32_newf( HAL_IN, &(addr->magY), comp_id, "%s.magY", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_u32_newf( HAL_IN, &(addr->magZ), comp_id, "%s.magZ", prefix);
    if (retval != 0) {
        return retval;
    }


   // Output pins:
    retval = hal_pin_float_newf( HAL_OUT, &(addr->quatW), comp_id, "%s.quatW", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_float_newf( HAL_OUT, &(addr->quatX), comp_id, "%s.quatX", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_float_newf( HAL_OUT, &(addr->quatY), comp_id, "%s.quatY", prefix);
    if (retval != 0) {
        return retval;
    }
	
	retval = hal_pin_float_newf( HAL_OUT, &(addr->quatZ), comp_id, "%s.quatZ", prefix);
    if (retval != 0) {
        return retval;
    }
  
  
   /* export processing function */
    char bufMAG[ HAL_NAME_LEN + 1];
    rtapi_snprintf( bufMAG, sizeof( bufMAG), "%s.updateAHRS", prefix);
    retval = hal_export_funct( bufMAG, updateAHRS, addr, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "MADGWICK: ERROR: updateAHRS function export failed\n");
        hal_exit( comp_id);
        return -1;
    }
	
	char bufNon_MAG[ HAL_NAME_LEN + 1];
    rtapi_snprintf( bufNon_MAG, sizeof( bufNon_MAG), "%s.updateAHRS_NonMag", prefix);
    retval = hal_export_funct( bufNon_MAG, updateAHRS_NonMag, addr, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg( RTAPI_MSG_ERR, "MADGWICK: ERROR: updateAHRS_NonMag function export failed\n");
        hal_exit( comp_id);
        return -1;
    }
		
    return 0;
}

static inline void *lcec_zalloc(size_t size) {
  void *p = malloc(size);
  if (p) memset(p, 0, size);
  return p;
}

static inline float ieee_float(const uint32_t* uRep) {
   fu un = { .u = *uRep };
   return un.f; 
}




