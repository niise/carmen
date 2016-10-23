#ifndef CARMEN_CPP_BASE_MESSAGE_H
#define CARMEN_CPP_BASE_MESSAGE_H

#include "cpp_global.h"
#include "cpp_genericmessage.h"
#include "cpp_point.h"




class OdometryMessage : public GenericMessage {
 public:
  OdometryMessage();
  OdometryMessage(const OrientedPoint& robotPose, double vTrans=0.0, double vRot=0.0, double acc=0.0);
  OdometryMessage(const OdometryMessage& x);
  OdometryMessage(const carmen_base_odometry_message& x);
  OdometryMessage(carmen_base_odometry_message* x);
  OdometryMessage(char* s);
  virtual ~OdometryMessage();

  void init();
  void clone(const OdometryMessage& x);
  void clone(const carmen_base_odometry_message& x);
  void setOdometryMessage(carmen_base_odometry_message* x);
  void free();

  OrientedPoint getRobotPose() const;
  void setRobotPose(const OrientedPoint& robotPose );

  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, X, x);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, Y, y);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, Theta, theta);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, TV, tv);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, RV, rv);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, Acceleration, acceleration);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, char*, Host, host);
  STRUCT_PARAM_VIRTUAL_SET_GET(*m_msg, double, Timestamp, timestamp, public, public);

  inline operator carmen_base_odometry_message() const {return *m_msg;}
  inline operator carmen_base_odometry_message*() const {return m_msg;}

  virtual const char* getMessageID() const {
    return "ODOM";
  };

  virtual void save(carmen_FILE *logfile, double logger_timestamp);
  virtual char*  fromString(char* s);
  virtual GenericMessage* clone() const;

 protected:
  carmen_base_odometry_message * m_msg;
};

#endif 
