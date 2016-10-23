// this contains useful calibrated values for the paint bot arm
// unless specifically stated, all units are 'orc io units' that is,
// that they are somehow proportional to the real thing

// ---- Initialize ---- //
// it would be cool if portmap was used in the later arrays, but for now
// we assume that items are always in the order elbow-shoulder-hip
#define ELBOW 0
#define SHOULDER 1
#define HIP 2

#define ELBOW_MOTOR_PORT 0
#define SHOULDER_MOTOR_PORT 1
#define HIP_MOTOR_PORT 2
 int MOTOR_PORTMAP[] = { ELBOW_MOTOR_PORT, SHOULDER_MOTOR_PORT, HIP_MOTOR_PORT };

#define ELBOW_ENCODER_PORT 0
#define SHOULDER_ENCODER_PORT 1
#define HIP_ENCODER_PORT 2
 int ENCODER_PORTMAP[] = { ELBOW_ENCODER_PORT, SHOULDER_ENCODER_PORT, HIP_ENCODER_PORT };

// depending on the orientation of your motor, you may need to multiply the change in ticks
// by -1 (if it is mounted in a mirrored way)
#define ELBOW_REVERSE_THETA -1
#define SHOULDER_REVERSE_THETA 1
#define HIP_REVERSE_THETA 1
 int REVERSE_THETA[] = { ELBOW_REVERSE_THETA, SHOULDER_REVERSE_THETA, HIP_REVERSE_THETA };

// ---- General ---- //
#define ENCODER_RESOLUTION 500
 
#define SHOULDER_GEAR_REDUCTION ( 65.5 * 5.0 )
#define ELBOW_GEAR_REDUCTION ( 65.5 * 5.0 )
#define HIP_GEAR_REDUCTION 65.5
double GEAR_REDUCTION[] = { ELBOW_GEAR_REDUCTION, 
			 SHOULDER_GEAR_REDUCTION, HIP_GEAR_REDUCTION };

//#define SHOULDER_TICKS_PER_RADIAN ( ENCODER_RESOLUTION * SHOULDER_GEAR_REDUCTION / ( 2.0 * M_PI ) ) 
//#define ELBOW_TICKS_PER_RADIAN ( ENCODER_RESOLUTION * ELBOW_GEAR_REDUCTION / ( 2.0 * M_PI ) ) 
//#define HIP_TICKS_PER_RADIAN ( ENCODER_RESOLUTION * HIP_GEAR_REDUCTION / ( 2.0 * M_PI ) )

#define SHOULDER_TICKS_PER_RADIAN 200000 / M_PI
#define ELBOW_TICKS_PER_RADIAN 200000 / M_PI
#define HIP_TICKS_PER_RADIAN 180000 / M_PI
double TICKS_PER_RADIAN[] = { ELBOW_TICKS_PER_RADIAN, 
			   SHOULDER_TICKS_PER_RADIAN, HIP_TICKS_PER_RADIAN };
			       
#define SHOULDER_THETA_OFFSET 0
#define ELBOW_THETA_OFFSET 0
#define HIP_THETA_OFFSET 0
double THETA_OFFSET[] = { ELBOW_THETA_OFFSET, 
		       SHOULDER_THETA_OFFSET, HIP_THETA_OFFSET };

// ---- Limits ---- //
#define MAX_CURRENT 0  // not currently being used

// pwm limits are always positive (code will check to make sure
// that we are within valid regions: <--(-max,-min)--0--(min,max)-->
#define HIP_MAX_THETA 3
#define HIP_MIN_THETA -3
#define HIP_MAX_PWM 80
#define HIP_MIN_PWM 30

#define SHOULDER_MAX_THETA 1.2
#define SHOULDER_MIN_THETA -1.2
#define SHOULDER_MAX_PWM 20
#define SHOULDER_MIN_PWM 10

#define ELBOW_MAX_THETA 1.2
#define ELBOW_MIN_THETA -1.2
#define ELBOW_MAX_PWM 15
#define ELBOW_MIN_PWM 10

 double MAX_THETA[] = { ELBOW_MAX_THETA, 
			  SHOULDER_MAX_THETA, HIP_MAX_THETA };
 double MIN_THETA[] = { ELBOW_MIN_THETA, 
			  SHOULDER_MIN_THETA, HIP_MIN_THETA };
 int MAX_PWM[] = { ELBOW_MAX_PWM, 
			SHOULDER_MAX_PWM, HIP_MAX_PWM };
 int MIN_PWM[] = { ELBOW_MIN_PWM, 
			SHOULDER_MIN_PWM, HIP_MIN_PWM };




// ---- PID constants ---- //

// for velocity control
#define ORC_ARM_MIN_ANGULAR_VEL 0.0001 // rss: 0.00125 // Radians / second 
#define ORC_ARM_MAX_ANGULAR_VEL 0.0035 // Radians / second
#define ORC_ARM_FF_GAIN ( (90 / ORC_ARM_MAX_ANGULAR_VEL ) * 0.9 );
#define ORC_ARM_P_GAIN 8
#define ORC_ARM_D_GAIN 15
#define ORC_ARM_I_GAIN 0 // 3

// for position control
#define ELBOW_THETA_P_GAIN 0.008
#define ELBOW_THETA_D_GAIN 0.018 // 0.018
#define ELBOW_THETA_I_GAIN 0.003 // 0.003

#define SHOULDER_THETA_P_GAIN 0.008
#define SHOULDER_THETA_D_GAIN 0.018 // 0.018
#define SHOULDER_THETA_I_GAIN 0.003 // 0.003

#define HIP_THETA_P_GAIN 0.016
#define HIP_THETA_D_GAIN 0.036 // 0.018
#define HIP_THETA_I_GAIN 0.006 // 0.003

double THETA_P_GAIN[] = { ELBOW_THETA_P_GAIN, 
		       SHOULDER_THETA_P_GAIN, HIP_THETA_P_GAIN };
double THETA_I_GAIN[] = { ELBOW_THETA_I_GAIN, 
		       SHOULDER_THETA_I_GAIN, HIP_THETA_I_GAIN };
double THETA_D_GAIN[] = { ELBOW_THETA_D_GAIN, 
		       SHOULDER_THETA_D_GAIN, HIP_THETA_D_GAIN };
