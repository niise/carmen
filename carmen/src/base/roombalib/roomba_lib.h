/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef CARMEN_PIONEER_PARAMS_H
#define CARMEN_PIONEER_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

// OI Modes
#define OI_MODE_OFF				0
#define OI_MODE_PASSIVE			1
#define OI_MODE_SAFE			2
#define OI_MODE_FULL			3

// Delay after mode change in ms
#define OI_DELAY_MODECHANGE_MS	20

// Charging states
#define OI_CHARGING_NO			0
#define OI_CHARGING_RECOVERY	1
#define OI_CHARGING_CHARGING	2
#define OI_CHARGING_TRICKLE		3
#define OI_CHARGING_WAITING		4
#define OI_CHARGING_ERROR		5

// IR Characters
#define FORCE_FIELD						161
#define GREEN_BUOY						164
#define GREEN_BUOY_FORCE_FIELD			165
#define RED_BUOY						168
#define RED_BUOY_FORCE_FIELD			169
#define RED_BUOY_GREEN_BUOY				172
#define RED_BUOY_GREEN_BUOY_FORCE_FIELD	173
#define VIRTUAL_WALL					162

// Positions
#define LEFT				0
#define RIGHT				1
#define FRONT_LEFT			2
#define FRONT_RIGHT			3
#define CENTER_LEFT			4
#define CENTER_RIGHT		5
#define OMNI				2
#define MAIN_BRUSH			2
#define SIDE_BRUSH			3

// Buttons
#define BUTTON_CLOCK		7
#define BUTTON_SCHEDULE		6
#define BUTTON_DAY			5
#define BUTTON_HOUR			4
#define BUTTON_MINUTE		3
#define BUTTON_DOCK			2
#define BUTTON_SPOT			1
#define BUTTON_CLEAN		0

// Roomba Dimensions
#define ROOMBA_BUMPER_X_OFFSET		0.050
#define ROOMBA_DIAMETER				0.330

#define ROOMBA_MAX_LIN_VEL_MM_S		500
#define ROOMBA_MAX_ANG_VEL_RAD_S	2
#define ROOMBA_MAX_RADIUS_MM		2000

// Roomba Dimensions
#define ROOMBA_BUMPER_X_OFFSET		0.050
#define ROOMBA_DIAMETER				0.330
#define ROOMBA_AXLE_LENGTH			0.227

#define ROOMBA_MAX_LIN_VEL_MM_S		500
#define ROOMBA_MAX_ANG_VEL_RAD_S	2
#define ROOMBA_MAX_RADIUS_MM		2000

//Rooma LED
#define ROOMBA_LED_DEBRIS			0x01
#define ROOMBA_LED_SPOT				0x02
#define ROOMBA_LED_DOCK				0x04
#define ROOMBA_LED_CHECK_ROBOT		0x08
#define ROOMBA_LED_PWR_INTENSITY_MAX	0xFF

typedef enum {

		// Command opcodes
		OI_OPCODE_RESET = 7,
		OI_OPCODE_START = 128,
		OI_OPCODE_BAUD = 129,
		OI_OPCODE_CONTROL = 130,
		OI_OPCODE_SAFE = 131,
		OI_OPCODE_FULL = 132,
		OI_OPCODE_POWER = 133,
		OI_OPCODE_SPOT = 134,
		OI_OPCODE_CLEAN = 135,
		OI_OPCODE_MAX = 136,
		OI_OPCODE_DRIVE = 137,
		OI_OPCODE_MOTORS = 138,
		OI_OPCODE_LEDS = 139,
		OI_OPCODE_SONG = 140,
		OI_OPCODE_PLAY = 141,
		OI_OPCODE_SENSORS = 142,
		OI_OPCODE_FORCE_DOCK = 143,
		OI_OPCODE_PWM_MOTORS = 144,
		OI_OPCODE_DRIVE_DIRECT = 145,
		OI_OPCODE_DRIVE_PWM = 146,
		OI_OPCODE_STREAM = 148,
		OI_OPCODE_QUERY = 149,
		OI_OPCODE_PAUSE_RESUME_STREAM = 150,
		OI_OPCODE_SCHEDULE_LEDS = 162,
		OI_OPCODE_DIGIT_LEDS_RAW = 163,
		OI_OPCODE_DIGIT_LEDS_ASCII = 164,
		OI_OPCODE_BUTTONS = 165,
		OI_OPCODE_SCHEDULE = 167,
		OI_OPCODE_SET_DAY_TIME = 168

	} OI_Opcode;

typedef enum {
		MODE_OFF		= 0,
		MODE_PASS,
		MODE_SAFE,
		MODE_FULL,
} RoombaMode;

typedef enum {
		SENSOR_BUTTON 						= 18,
		SENSOR_DISTANCE						= 19, //broken on firmware < 3.3.0
		SENSOR_ANGLE						= 20, //broken on firmware < 3.4.0
		SENSOR_CHG_STATUS					= 21, // one of ChargingState
		SENSOR_VOL 							= 22, //mV
		SENSOR_CUR 							= 23, //mA
		SENSOR_TEMPERATURE 					= 24, //degC
		SENSOR_BATT_CHG						= 25, //mAh
		SENSOR_BATT_CAPACITY 				= 26,
		SENSOR_CLIFF_LEFT_SIGNAL 			= 28,
		SENSOR_CLIFF_FNT_LEFT_SIGNAL		= 29,
		SENSOR_CLIFF_FNT_RIGHT_SIGNAL		= 30,
		SENSOR_CLIFF_RIGHT_SIGNAL			= 31,
		SENSOR_CHG_SRC_AVAILABLE			= 34,
		SENSOR_OIMODE						= 35, // one of Mode
		SENSOR_SONG_PLAYING 				= 37,
		SENSOR_NUM_STREAM_PACKET			= 38,
		SENSOR_REQ_TED_VELOCITY				= 39,
		SENSOR__REQ_RADIUS 					= 40,
		SENSOR_REQ_R_VELOCITY				= 41,
		SENSOR_REQ_TED_L_VELOCITY			= 42,
		SENSOR_LEFT_ENCODER_COUNTS			= 43,
		SENSOR_RIGHT_ENCODER_COUNTS			= 44,
		SENSOR_RIGHT_BUMPER 				= 45,
		SENSOR_LIGHT_BUMPER_L_SIGNAL 		= 46,
		SENSOR_LIGHT_BUMPER_FNT_L_SIGNAL	= 47,
		SENSOR_LIGHT_BUMPER_CENTER_L_SIGNAL	= 48,
		SENSOR_LIGHT_BUMPER_CENTER_R_SIGNAL	= 49,
		SENSOR_LIGHT_BUMPER_FNT_R_SIGNAL	= 50,
		SENSOR_LIGHT_BUMPER_R_SIGNAL 		= 51,
		SENSOR_L_MOTOR_CUR					= 54, //mA
		SENSOR_R_MOTOR_CUR					= 55, //mA
		SENSOR_MAIN_BRUSH_MOTOR_CUR			= 56, //mA
		SENSOR_SIDE_BRUSH_MOTOR_CUR			= 57, //mA
		SENSOR_STATUS						= 58,
} Sensor_ID;

typedef struct
{
  /* Serial port to which the robot is connected */
  char serial_port[PATH_MAX];
  /* File descriptor associated with serial connection (-1 if no valid
   * connection) */
  int fd;
  /* Current operation mode; one of ROOMBA_MODE_* */
  unsigned char mode;
  /* Integrated odometric position [m m rad] */
  double ox, oy, oa;

  /* Various Boolean flags */
  int bumper_left, bumper_right;
  unsigned char wheeldrop_caster, wheeldrop_left, wheeldrop_right;
  unsigned char wall;
  unsigned char cliff_left, cliff_frontleft, cliff_frontright, cliff_right;
  unsigned char virtual_wall;
  unsigned char overcurrent_driveleft, overcurrent_driveright;
  unsigned char overcurrent_mainbrush, overcurrent_sidebrush;
  unsigned char overcurrent_vacuum;
  unsigned char dirtdetector_right, dirtdetector_left;
  unsigned char remote_opcode;
  unsigned char button_power, button_spot, button_clean, button_max;

  /* One of ROOMBA_CHARGING_* */
  unsigned char charging_state;
  /* Volts */
  double voltage;
  /* Amps */
  double current;
  /* degrees C */
  double temperature;
  /* Ah */
  double charge;
  /* Capacity */
  double capacity;
  /* Velocity */
  double velocity;
} roomba_comm_t;




#ifdef __cplusplus
}
#endif

#endif
