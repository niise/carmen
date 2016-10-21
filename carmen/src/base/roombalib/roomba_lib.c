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

#include <carmen/carmen.h>
#include <carmen/drive_low_level.h>
#include <limits.h>

#include "roomba_lib.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <netinet/in.h>
#include <stdint.h>
#include <sys/poll.h>

/*** Some pioneer internal constants.  I have them defined here since the ***/
/*** outside world doesn't need to know about them.                       ***/
#define ROOMBA_DELAY_MODECHANGE_MS      200
#define ROOMBA_WHEEL_MAX_MM			500

#define ROOMBA_SONG_NUM_MAX			4
#define ROOMBA_SONG_LENGTH_MAX		16
#define ROOMBA_SONG_LENGTH_MIN		1
#define ROOMBA_SENSOR_PACKET_SIZE       26

#define MIN(a,b) ((a < b) ? (a) : (b))
#define MAX(a,b) ((a > b) ? (a) : (b))
#define NORMALIZE(z) atan2(sin(z), cos(z))

/*** When we receive an information packet back from the robot, it is     ***/
/*** initially read into a raw information packet as if to a big char     ***/
/*** buffer.  It is then converted to a regular information packet, which ***/
/*** contains more useful data types.                                     ***/

roomba_comm_t* roomba_dev;


/*** Return value:  the length of the string read                       ***/
int roomba_read_string(roomba_comm_t *roomba, unsigned char *buf, int length)
{
  int num;

  num = carmen_serial_readn(roomba->fd, buf, length);

  return num;
}

int roomba_send_string(roomba_comm_t *roomba, unsigned char *buf, int length)
{
  int err;


  err = carmen_serial_writen(roomba->fd, buf, length);
  if(err < 0) {
	carmen_warn("Wirte Fail Command %d, Length %d \n",buf[0], length);
	return err;
  }

  return 0;
}

int roomba_parse_sensor_packet(roomba_comm_t* roomba, unsigned char* buf, size_t buflen)
{
  unsigned char flag;
  int16_t signed_int;
  uint16_t unsigned_int;
  double dist, angle;
  int idx;

  if(buflen != ROOMBA_SENSOR_PACKET_SIZE)
  {
    puts("roomba_parse_sensor_packet():packet is wrong size");
    return(-1);
  }

  idx = 0;

  /* Bumps, wheeldrops */
  flag = buf[idx++];
  roomba->bumper_right = (flag >> 0) & 0x01;
  roomba->bumper_left = (flag >> 1) & 0x01;
  roomba->wheeldrop_right = (flag >> 2) & 0x01;
  roomba->wheeldrop_left = (flag >> 3) & 0x01;
  roomba->wheeldrop_caster = (flag >> 4) & 0x01;

  roomba->wall = buf[idx++] & 0x01;
  roomba->cliff_left = buf[idx++] & 0x01;
  roomba->cliff_frontleft = buf[idx++] & 0x01;
  roomba->cliff_frontright = buf[idx++] & 0x01;
  roomba->cliff_right = buf[idx++] & 0x01;
  roomba->virtual_wall = buf[idx++] & 0x01;

  flag = buf[idx++];
  roomba->overcurrent_sidebrush = (flag >> 0) & 0x01;
  roomba->overcurrent_vacuum = (flag >> 1) & 0x01;
  roomba->overcurrent_mainbrush = (flag >> 2) & 0x01;
  roomba->overcurrent_driveright = (flag >> 3) & 0x01;
  roomba->overcurrent_driveleft = (flag >> 4) & 0x01;

  roomba->dirtdetector_left = buf[idx++];
  roomba->dirtdetector_right = buf[idx++];
  roomba->remote_opcode = buf[idx++];

  flag = buf[idx++];
  roomba->button_max = (flag >> 0) & 0x01;
  roomba->button_clean = (flag >> 1) & 0x01;
  roomba->button_spot = (flag >> 2) & 0x01;
  roomba->button_power = (flag >> 3) & 0x01;

  memcpy(&signed_int, buf+idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  dist = signed_int / 1e3;

  memcpy(&signed_int, buf+idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  angle = (2.0 * (signed_int / 1e3)) / ROOMBA_AXLE_LENGTH;

  /* First-order odometric integration */
  roomba->oa = NORMALIZE(roomba->oa + angle);
  roomba->ox += dist * cos(roomba->oa);
  roomba->oy += dist * sin(roomba->oa);

  roomba->charging_state = buf[idx++];

  memcpy(&unsigned_int, buf+idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  roomba->voltage = unsigned_int / 1e3;

  memcpy(&signed_int, buf+idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  roomba->current = signed_int / 1e3;

  roomba->temperature = (char)(buf[idx++]);

  memcpy(&unsigned_int, buf+idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  roomba->charge = unsigned_int / 1e3;

  memcpy(&unsigned_int, buf+idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  roomba->capacity = unsigned_int / 1e3;

  assert(idx == ROOMBA_SENSOR_PACKET_SIZE);

  return(0);
}

int roomba_init_serial_port(roomba_comm_t *roomba, unsigned int baudrate) {

  struct termios term;


  if(roomba->fd >= 0)
  {
	puts("roomba connection already open!");
	return -1;
  }
  fflush(stdout);
  if((roomba->fd = open(roomba->serial_port,
				  O_RDWR | O_NONBLOCK, S_IRUSR | S_IWUSR )) < 0 )
  {
	perror("roomba_init_serial_port:open:");
	return -1;
  }

  if(tcflush(roomba->fd, TCIFLUSH) < 0 )
  {
	perror("roomba_init_serial_port:tcflush():");
	close(roomba->fd);
	roomba->fd = -1;
	return -1;
  }
  if(tcgetattr(roomba->fd, &term) < 0 )
  {
	perror("roomba_init_serial_port:tcgetattr():");
	close(roomba->fd);
	roomba->fd = -1;
	return -1;
  }

  cfmakeraw(&term);
  if( baudrate == 115200 ) {
	cfsetispeed(&term, B115200);
	cfsetospeed(&term, B115200);
  }
  else {
	carmen_warn("Buadrate %d is not supported by roomba", baudrate);
	close(roomba->fd);
    roomba->fd = -1;
    return -1;
  }

  if(tcsetattr(roomba->fd, TCSAFLUSH, &term) < 0 )
  {
    perror("roomba_init_serial_port:tcsetattr():");
    close(roomba->fd);
    roomba->fd = -1;
    return -1;
  }

  return 0;
}

int roomba_cmd_reset(roomba_comm_t *roomba) {

  unsigned char buf[1];
  int err;
  buf[0] = OI_OPCODE_RESET;

  err = roomba_send_string(roomba, buf, sizeof(buf));

  if (err) {
	return err;
  }
  roomba->mode = MODE_OFF;
  return 0;
}

int roomba_cmd_start(roomba_comm_t *roomba) {

  unsigned char buf[1];
  int err;
  buf[0] = OI_OPCODE_START;

  err = roomba_send_string(roomba, buf, sizeof(buf));

  if (err) {
	carmen_warn("Fail to set Start Mode");
	return err;
  }
  roomba->mode = MODE_PASS;
  return 0;
}

int roomba_cmd_safe(roomba_comm_t *roomba) {

  unsigned char buf[1];
  int err;
  if(roomba->mode == MODE_OFF) {
	carmen_warn("Roomba is in OFF mode, Please Change to PASS mode");
	return -1;
  }

  buf[0] = OI_OPCODE_CONTROL;
  err = roomba_send_string(roomba, buf, sizeof(buf));

  if(err) {
	carmen_warn("Fail to set Safe Mode");
	return err;
  }
  roomba->mode = MODE_SAFE;
  return 0;
}

int roomba_cmd_full(roomba_comm_t *roomba) {

  unsigned char buf[1];
  int err;

  if(roomba->mode == MODE_OFF) {
	carmen_warn("Roomba is in OFF mode, Please Change to PASS mode");
	return -1;
  }

  buf[0] = OI_OPCODE_FULL;
  err = roomba_send_string(roomba, buf, sizeof(buf));

  if(err) {
	carmen_warn("Fail to set Full Mode");
	return err;
  }
  roomba->mode = MODE_PASS;
  return 0;
}

int roomba_cmd_power(roomba_comm_t *roomba) {

  unsigned char buf[1];
  int err;

  if(roomba->mode == MODE_OFF) {
	carmen_warn("Roomba is in OFF mode, Please Change to PASS mode");
	return -1;
  }

  buf[0] = OI_OPCODE_POWER;

  err = roomba_send_string(roomba, buf, sizeof(buf));

  if (err) {
	return err;
  }
  roomba->mode = MODE_PASS;
  return 0;
}

int roomba_cmd_stop(roomba_comm_t *roomba) {

  unsigned char buf[1];
  int err;

  if(roomba->mode == MODE_OFF) {
	carmen_warn("Roomba is in OFF mode, Please Change to PASS mode");
	return -1;
  }

  buf[0] = OI_OPCODE_SPOT;
  err = roomba_send_string(roomba, buf, sizeof(buf));

  if (err) {
	carmen_warn("Fail to Stop Roomba");
	return err;
  }
  roomba->mode = MODE_PASS;
  return 0;
}

int roomba_cmd_clean(roomba_comm_t *roomba) {

  unsigned char buf[1];
  int err;

  if(roomba->mode == MODE_OFF) {
	carmen_warn("Roomba is in OFF mode, Please Change to PASS mode");
	return -1;
  }

  buf[0] = OI_OPCODE_CLEAN;

  err = roomba_send_string(roomba, buf, sizeof(buf));

  if (err) {
	return err;
  }
  roomba->mode = MODE_PASS;
  return 0;
}

int roomba_cmd_led(roomba_comm_t *roomba, unsigned char led_bit, unsigned char color, unsigned char led_intensity) {

  unsigned char buf[4];
  int err;

  if(roomba->mode == MODE_OFF || roomba->mode == MODE_PASS) {
	return -1;
  }

  buf[0] = OI_OPCODE_LEDS;
  buf[1] = led_bit;
  buf[2] = color;
  buf[3] = led_intensity;

  err = roomba_send_string(roomba, buf, sizeof(buf));

  if (err) {
	return err;
  }

  return 0;
}

int roomba_cmd_song(roomba_comm_t *roomba, unsigned char song_num, unsigned char song_len, unsigned char *song_buf) {

  unsigned char buf[35];
  int err,i;

  if(roomba->mode == MODE_OFF) {
	return -1;
  }

  if(song_num > ROOMBA_SONG_NUM_MAX || song_len < ROOMBA_SONG_LENGTH_MIN || song_len > ROOMBA_SONG_LENGTH_MAX)
	return -1;

  buf[0] = OI_OPCODE_SONG;
  buf[1] = song_num;
  buf[2] = song_len;

  for (i=0; i < song_len; i++)
  {
	buf[3+(2*i)] = song_buf[2*i];
	buf[3+(2*i)+1] = song_buf[(2*i) + 1];
  }

  err = roomba_send_string(roomba, buf, (song_len * 2) + 3 );

  if (err) {
	return err;
  }

  return 0;
}

int roomba_cmd_play(roomba_comm_t *roomba, unsigned char song_num) {

  unsigned char buf[2];
  int err;

  if(roomba->mode == MODE_OFF || roomba->mode == MODE_PASS) {
	return -1;
  }

  if(song_num > ROOMBA_SONG_NUM_MAX)
	return -1;


  buf[0] = OI_OPCODE_PLAY;
  buf[1] = song_num;

  err = roomba_send_string(roomba, buf, sizeof(buf));

  if (err) {
	return err;
  }

  return 0;
}

int roomba_cmd_drive_direct(roomba_comm_t *roomba, int vr, int vl) {
  unsigned char buf[5];

  if(roomba->mode == MODE_OFF) {
	carmen_warn("Roomba is in OFF mode, Please Change to PASS mode");
	return -1;
  }
  carmen_warn("Right Wheel = %d, Left Wheel = %d\n", vr, vl);
  if(vr < -ROOMBA_WHEEL_MAX_MM || vr > ROOMBA_WHEEL_MAX_MM)
	carmen_warn("Right Wheel speed is out of limit");
  if(vl < -ROOMBA_WHEEL_MAX_MM || vl > ROOMBA_WHEEL_MAX_MM)
	carmen_warn("Left Wheel speed is out of limit");

  vr = MAX(vr, -ROOMBA_WHEEL_MAX_MM);
  vr = MIN(vr, ROOMBA_WHEEL_MAX_MM);

  vl = MAX(vl, -ROOMBA_WHEEL_MAX_MM);
  vl = MIN(vl, ROOMBA_WHEEL_MAX_MM);
  buf[0] = OI_OPCODE_DRIVE_DIRECT;
  buf[1] = (unsigned char)(vr >>8);
  buf[2] = (unsigned char)(vr & 0xFF);
  buf[3] = (unsigned char)(vl >>8);
  buf[4] = (unsigned char)(vl & 0xFF);

  return roomba_send_string(roomba, buf, sizeof(buf));
}

int roomba_cmd_sensor(roomba_comm_t* roomba, unsigned char id, unsigned char *databuf,unsigned char data_len, int timeout)
{
  struct pollfd ufd[1];
  unsigned char cmdbuf[2];
  int retval;
  int numread;
  unsigned int totalnumread;
  //int i;

  cmdbuf[0] = OI_OPCODE_SENSORS;
  /* Zero to get all sensor data */
  cmdbuf[1] = id;

  if(roomba_send_string(roomba, cmdbuf, 2) < 0)
  {
    perror("roomba_get_sensors():write():");
    return(-1);
  }

  ufd[0].fd = roomba->fd;
  ufd[0].events = POLLIN;

  totalnumread = 0;
  while(totalnumread < data_len)
  {
    retval = poll(ufd,1,timeout);

    if(retval < 0)
    {
      if(errno == EINTR)
        continue;
      else
      {
        perror("roomba_get_sensors():poll():");
        return(-1);
      }
    }
    else if(retval == 0)
    {
      printf("roomba_get_sensors: poll timeout\n");
      return(-1);
    }
    else
    {
      if((numread = roomba_read_string(roomba,databuf+totalnumread,data_len-totalnumread)) < 0)
      {
        perror("roomba_get_sensors():read()");
        return(-1);
      }
      else
      {
        totalnumread += numread;
        /*printf("read %d bytes; buffer so far:\n", numread);
        for(i=0;i<totalnumread;i++)
          printf("%x ", databuf[i]);
        puts("");
        */
      }
    }
  }

  return totalnumread;
}


int roomba_init_mode(roomba_comm_t *roomba) {
  int err;

  usleep(ROOMBA_DELAY_MODECHANGE_MS * 1e3);

  err = roomba_cmd_start(roomba);
  if (err) return err;

  usleep(ROOMBA_DELAY_MODECHANGE_MS * 1e3);

  err = roomba_cmd_safe(roomba);
  if (err) return err;

  usleep(ROOMBA_DELAY_MODECHANGE_MS * 1e3);

  return 0;
}
void roomba_print(roomba_comm_t* r)
{
  printf("mode: %d\n", r->mode);
  printf("position: %.3lf %.3lf %.3lf\n", r->ox, r->oy, r->oa);
  printf("bumpers: l:%d r:%d\n", r->bumper_left, r->bumper_right);
  printf("wall: %d virtual wall: %d\n", r->wall, r->virtual_wall);
  printf("wheeldrops: c:%d l:%d r:%d\n",
         r->wheeldrop_caster, r->wheeldrop_left, r->wheeldrop_right);
  printf("cliff: l:%d fl:%d fr:%d r:%d\n",
         r->cliff_left, r->cliff_frontleft, r->cliff_frontright, r->cliff_right);
  printf("overcurrent: dl:%d dr:%d mb:%d sb:%d v:%d\n",
         r->overcurrent_driveleft, r->overcurrent_driveright,
         r->overcurrent_mainbrush, r->overcurrent_sidebrush, r->overcurrent_vacuum);
  printf("dirt: l:%d r:%d\n", r->dirtdetector_left, r->dirtdetector_right);
  printf("remote opcode: %d\n", r->remote_opcode);
  printf("buttons: p:%d s:%d c:%d m:%d\n",
         r->button_power, r->button_spot, r->button_clean, r->button_max);
  printf("charging state: %d\n", r->charging_state);
  printf("battery: voltage:%.3lf current:%.3lf temp:%.3lf charge:%.3lf capacity:%.3f\n",
         r->voltage, r->current, r->temperature, r->charge, r->capacity);

}

void roomba_destroy(roomba_comm_t* roomba)
{
  free(roomba);
}

int carmen_base_direct_sonar_on(void)
{
  carmen_warn("Sonar on isn't support by Roomba\n");

  return 0;
}

int carmen_base_direct_sonar_off(void)
{

  carmen_warn("Sonar off isn't support by Roomba\n");

  return 0;


  //  return 0;
}

int carmen_base_direct_reset(void)
{
  int err;

  err = roomba_cmd_drive_direct(roomba_dev, 0, 0);
  usleep(ROOMBA_DELAY_MODECHANGE_MS * 1e3);

 /* err = roomba_cmd_reset(roomba_dev);
  if(err) return -1;

  usleep(ROOMBA_DELAY_MODECHANGE_MS * 1e3);

  err = roomba_init_mode(roomba_dev);
  if(err) return -1;
  carmen_warn("Reset Complete\n");
*/
  return 0;
}

int carmen_base_direct_initialize_robot(char *model, char *dev)
{
  roomba_comm_t* roomba = NULL;
  roomba = calloc(1,sizeof(roomba_comm_t));
  assert(roomba);
  roomba->fd = -1;
  roomba->mode = MODE_OFF;
  strncpy(roomba->serial_port, dev,sizeof(roomba->serial_port)-1);

  if(roomba_init_serial_port(roomba, 115200)) {
	carmen_warn("Fail to connect Roomba Device %s aaa\n", roomba->serial_port);
	roomba_destroy(roomba);
	return -1;

  }
  if(roomba_init_mode(roomba)) {
	carmen_warn("Fail to init Roomba Mode");
	roomba_destroy(roomba);
	return -1;
  }

  roomba_cmd_led(roomba, ROOMBA_LED_DEBRIS | ROOMBA_LED_SPOT | ROOMBA_LED_DOCK | ROOMBA_LED_CHECK_ROBOT, 0xFF, 0x80);
  usleep(500 * 1e3);
  roomba_cmd_led(roomba, 0x00, 0xFF, 0x80);
  usleep(500 * 1e3);
  roomba_cmd_led(roomba, ROOMBA_LED_DEBRIS | ROOMBA_LED_SPOT | ROOMBA_LED_DOCK | ROOMBA_LED_CHECK_ROBOT, 0xFF, 0x80);
  usleep(500 * 1e3);
  roomba_cmd_led(roomba, 0x00, 0xFF, 0x80);
  usleep(500 * 1e3);
  roomba_cmd_led(roomba, ROOMBA_LED_DEBRIS | ROOMBA_LED_SPOT | ROOMBA_LED_DOCK | ROOMBA_LED_CHECK_ROBOT, 0xFF, 0x80);
  usleep(500 * 1e3);
  roomba_cmd_led(roomba, 0x00, 0xFF, 0x80);

  roomba_dev = roomba;
  return 0;
}

int carmen_base_direct_shutdown_robot(void)
{
  int err;

  err = roomba_cmd_drive_direct(roomba_dev, 0, 0);

  if(close(roomba_dev->fd) < 0)
	return -1;

  carmen_warn("Shutdown Roomba");

  return 0;
}

int carmen_base_direct_set_acceleration(double acceleration)
{
  carmen_warn("Acceleration isn't support by Roomba\n");

  return 0;
}

int carmen_base_direct_set_deceleration(double deceleration)
{
  carmen_warn("Deceleration isn't support by Roomba\n");

  return 0;
}

int carmen_base_direct_set_velocity(double tv, double rv)
{
  /** Cyrill, 13.Jan.2004
      Pioneer drives much nicer when using VEL2 instead
      of VEL and RVEL commands.

      ActiveMedia's strage dist_conf_factor comes from:
      distBetweemLeftAndRightWheel = 0.002 / dist_conf_factor
  */
  int err = 0;
  double vl = (tv  - ROOMBA_AXLE_LENGTH * rv / 2) * 1e3;
  double vr = (tv  + ROOMBA_AXLE_LENGTH * rv / 2) * 1e3;
  err = roomba_cmd_drive_direct(roomba_dev, (int)vr, (int)vl);

  if(err < 0)
	return -1;

  return 0;

  /* The old method to send the velocities to the robot.
     Pioneer Robots (can) drive quite poor if using
     VEL and RVEL command, that's why I changed the
     code to the  VEL2 command */

/*   int err;  */
/*   int arg; */
/*   arg = tv * 1000; */
/*   err = pioneer_send_command1(PIONEER_VEL, arg); */
/*   if (err < 0) */
/*     return -1; */

/*   arg = carmen_radians_to_degrees(rv); */
/*   err = pioneer_send_command1(PIONEER_RVEL, arg); */
/*   if (err < 0) */
/*     return -1; */
/*   return 0; */
}


int carmen_base_direct_update_status(double* update_timestamp)
{
  unsigned char databuf[ROOMBA_SENSOR_PACKET_SIZE];
  int num;
  num = roomba_cmd_sensor(roomba_dev, 0, databuf, ROOMBA_SENSOR_PACKET_SIZE,-1);
  roomba_parse_sensor_packet(roomba_dev, databuf, (size_t)num);
  roomba_print(roomba_dev);

  return 0;
}

int carmen_base_direct_get_state(double *displacement, double *rotation,
			     double *tv, double *rv)
{
  return 0;
}

int carmen_base_direct_get_integrated_state(double *x, double *y, double *theta,
					double *tv, double *rv)
{
  unsigned char databuf[2];
  int num;
  int vr, vl;
  double v,r;
  num = roomba_cmd_sensor(roomba_dev, SENSOR_REQ_R_VELOCITY, databuf, 2,-1);

  if(num !=2)
	return -1;
  vr = ((int)databuf[0] << 8) + databuf[1];

  num = roomba_cmd_sensor(roomba_dev, SENSOR_REQ_TED_L_VELOCITY, databuf, 2,-1);

  if(num !=2)
	return -1;
  vl = ((int)databuf[0] << 8) + databuf[1];

  v = ((double)vr + (double)vl) / (2* 1e3);
  r = ((double)vr - (double)vl) / (ROOMBA_AXLE_LENGTH * 1e3);

  *x = roomba_dev->ox;
  *y = roomba_dev->oy;
  *theta = roomba_dev->oa;
  *tv = v;
  *rv = r;
  carmen_warn("TV = %.3lf, RV = %.3lf\n", v , r);

  return 0;
}

int carmen_base_direct_send_binary_data(unsigned char *data, int size)
{
  return roomba_send_string(roomba_dev, data, size);
}

int carmen_base_direct_get_binary_data(unsigned char **data, int *size)
{

  return 0;
}

int carmen_base_direct_get_bumpers(unsigned char *state, int num_bumpers)
{
  return 0;
}

void carmen_base_direct_arm_get(double servos[] __attribute__ ((unused)),
				int num_servos __attribute__ ((unused)),
				double *currents __attribute__ ((unused)),
				int *gripper __attribute__ ((unused)))
{
  carmen_warn("%s not supported by roomba.\n", __FUNCTION__);
}

void carmen_base_direct_arm_set(double servos[] __attribute__ ((unused)),
				int num_servos __attribute__ ((unused)))
{
  carmen_warn("%s not supported by roomba.\n", __FUNCTION__);
}

int carmen_base_direct_get_sonars(double *ranges,
				  carmen_point_t *positions
				  __attribute__ ((unused)),
				  int num_sonars)
{
  carmen_warn("No sonars in Roomba.\n");

  return 0;
}
