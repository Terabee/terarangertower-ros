/****************************************************************************
 *
 * Copyright (C) 2014 Flavio Fontana & Luis Rodrigues. All rights reserved.
 * Author: Flavio Fontana <fly.fontana@gmail.com>
 * Author: Luis Rodrigues <luis.rodrigues@terabee.com>

 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:  
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name Teraranger_tower nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <string>
#include "teraranger_tower/teraranger_tower.h"
#include <ros/console.h>
#include <limits>

namespace teraranger_tower
{

Teraranger_tower::Teraranger_tower()
{
  // Get paramters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_, std::string("/dev/ttyACM0"));

  // Publishers
  scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 50);

  // Create serial port
  serial_port_ = new SerialPort();
  
  // Set callback function for the serial ports
  serial_data_callback_function_ = boost::bind(&Teraranger_tower::serialDataCallback, this, _1);
  serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

  // Connect serial port
  if (!serial_port_->connect(portname_))
  {
    ros::shutdown();
    return;
  }

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());

  // Set operation Mode
 setMode(BINARY_MODE);

  // Dynamic reconfigure
  dyn_param_server_callback_function_ = boost::bind(&Teraranger_tower::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);
}

Teraranger_tower::~Teraranger_tower()
{
}

uint8_t Teraranger_tower::crc8(uint8_t *p, uint8_t len)
{
  uint16_t i;
  uint16_t crc = 0x0;

  while (len--)
  {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

void Teraranger_tower::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  sensor_msgs::LaserScan scan;
  ros::Time current_time = ros::Time::now();

  scan.header.frame_id="scan";

  scan.scan_time = 0.012;
  scan.range_min = 0.2;
  scan.range_max = 14.0;
  scan.header.stamp = current_time - ros::Duration(scan.scan_time);
  scan.angle_max = 3.142;
  scan.angle_min = -scan.angle_max;
  scan.time_increment = scan.scan_time /8; 
  scan.angle_increment = 2 * 3.141592 /8;
  scan.ranges.resize(8);
  
  double inf = std::numeric_limits<double>::infinity();
  double DistanceToCenter = 0.6;
  for (int i=0; i<8; i++)
  {
	  scan.ranges[i]=inf;
  }

 if (single_character != 'T' && buffer_ctr < 19)
  {
    // not begin of serial feed so add char to buffer
    input_buffer[buffer_ctr++] = single_character;
    return;
  }


  else if (single_character == 'T')
  {
    
    if (buffer_ctr == 19)
    {
      // end of feed, calculate
      int16_t crc = crc8(input_buffer, 18);

      if (crc == input_buffer[18])
      {
        int16_t range0 = input_buffer[2] << 8;
        range0 |= input_buffer[3];
        int16_t range1 = input_buffer[4] << 8;
        range1 |= input_buffer[5];
        int16_t range2 = input_buffer[6] << 8;
        range2 |= input_buffer[7];
        int16_t range3 = input_buffer[8] << 8;
        range3 |= input_buffer[9];
        int16_t range4 = input_buffer[10] << 8;
        range4 |= input_buffer[11];
        int16_t range5 = input_buffer[12] << 8;
        range5 |= input_buffer[13];
        int16_t range6 = input_buffer[14] << 8;
        range6 |= input_buffer[15];
        int16_t range7 = input_buffer[16] << 8;
        range7 |= input_buffer[17];

//commment the sensors that you don't want to use
        scan.ranges[0]=range0*0.001+DistanceToCenter;
        scan.ranges[1]=range1*0.001+DistanceToCenter;
        scan.ranges[2]=range2*0.001+DistanceToCenter;
        scan.ranges[3]=range3*0.001+DistanceToCenter;
        scan.ranges[4]=range4*0.001+DistanceToCenter;
        scan.ranges[5]=range5*0.001+DistanceToCenter;
        scan.ranges[6]=range6*0.001+DistanceToCenter;
        scan.ranges[7]=range7*0.001+DistanceToCenter;
        

        scan_publisher_.publish(scan);        
      }
      else
      {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      }
    }
    else
    {
      ROS_DEBUG("[%s] reveived T but did not expect it, reset buffer without evaluating data",
               ros::this_node::getName().c_str());
    }
  }
  else
  {
    ROS_DEBUG("[%s] buffer_overflowed without receiving T, reset input_buffer", ros::this_node::getName().c_str());
  }

  // reset
  buffer_ctr = 0;

  // clear struct
  bzero(&input_buffer, BUFFER_SIZE);

  // store T
  input_buffer[buffer_ctr++] = 'T';
}  //


//set mode original


void Teraranger_tower::setMode(const char *c)
{
 serial_port_->sendChar(c);
}

void Teraranger_tower::dynParamCallback(const teraranger_tower::Teraranger_towerConfig &config, uint32_t level)
{
  if (config.Mode == teraranger_tower::Teraranger_tower_Fast)
  {
    setMode(FAST_MODE);
  }

  if (config.Mode == teraranger_tower::Teraranger_tower_Precise)
  {
    setMode(PRECISE_MODE);
  }

  if (config.Mode == teraranger_tower::Teraranger_tower_Outdoor)
  {
    setMode(OUTDOOR_MODE);
  }
} 

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teraranger_tower");
  teraranger_tower::Teraranger_tower tera_bee;
  ros::spin();

  return 0;
}
