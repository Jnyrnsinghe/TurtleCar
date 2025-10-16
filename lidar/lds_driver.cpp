/*******************************************************************************
* Copyright (c) 2016, Hitachi-LG Data Storage
* Copyright (c) 2017, ROBOTIS
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

 /* Authors: SP Kong, JH Yang, Pyo */
 /* maintainer: Pyo */

#include "lds_driver.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <boost/asio.hpp>

namespace lds
{
LFCDLaser::LFCDLaser(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
  : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  // Below command is not required after firmware upgrade (2017.10)
  boost::asio::write(serial_, boost::asio::buffer("b", 1));  // start motor
}

LFCDLaser::~LFCDLaser()
{
  boost::asio::write(serial_, boost::asio::buffer("e", 1));  // stop motor
}

void LFCDLaser::poll()
{
  uint8_t temp_char;
  uint8_t start_count = 0;
  bool got_scan = false;
  boost::array<uint8_t, 2520> raw_bytes;
  uint8_t good_sets = 0;
  uint32_t motor_speed = 0;
  rpms=0;
  int index;

  // Arrays to store all 360 measurements
  std::vector<float> ranges(360, 0.0);
  std::vector<float> intensities(360, 0.0);

  while (!shutting_down_ && !got_scan)
  {
    // Wait until first data sync of frame: 0xFA, 0xA0
    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count],1));

    if(start_count == 0)
    {
      if(raw_bytes[start_count] == 0xFA)
      {
        start_count = 1;
      }
    }
    else if(start_count == 1)
    {
      if(raw_bytes[start_count] == 0xA0)
      {
        start_count = 0;

        // Now that entire start sequence has been found, read in the rest of the message
        got_scan = true;

        boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[2], 2518));

        //read data in sets of 6
        for(uint16_t i = 0; i < raw_bytes.size(); i=i+42)
        {
          if(raw_bytes[i] == 0xFA && raw_bytes[i+1] == (0xA0 + i / 42)) //&& CRC check
          {
            good_sets++;
            motor_speed += (raw_bytes[i+3] << 8) + raw_bytes[i+2]; //accumulate count for avg. time increment
            rpms=(raw_bytes[i+3]<<8|raw_bytes[i+2])/10;

            for(uint16_t j = i+4; j < i+40; j=j+6)
            {
              index = 6*(i/42) + (j-4-i)/6;

              // Four bytes per reading
              uint8_t byte0 = raw_bytes[j];
              uint8_t byte1 = raw_bytes[j+1];
              uint8_t byte2 = raw_bytes[j+2];
              uint8_t byte3 = raw_bytes[j+3];

              // Remaining bits are the range in mm
              uint16_t intensity = (byte1 << 8) + byte0;
              uint16_t range = (byte3 << 8) + byte2;

              // Store in arrays (convert to meters)
              ranges[359-index] = range / 1000.0;
              intensities[359-index] = intensity;
            }
          }
        }

        // Filter and display only front, back, left, right data
        displayCardinalDirections(ranges);
        
        // Send data to OpenCR via serial
        sendToOpenCR(ranges);
      }
      else
      {
        start_count = 0;
      }
    }
  }
}

void LFCDLaser::displayCardinalDirections(const std::vector<float>& ranges)
{
  // Define angle ranges for each direction (in degrees)
  const int FRONT_START = 150;  // -5 degrees
  const int FRONT_END = 210;      // +5 degrees
  const int RIGHT_START = 60;   // 85 degrees
  const int RIGHT_END = 95;     // 95 degrees
  const int BACK_START = 325;   // 175 degrees
  const int BACK_END = 35;     // 185 degrees
  const int LEFT_START = 265;   // 265 degrees
  const int LEFT_END = 300;     // 275 degrees
  
  // Calculate averages for each direction (handling wrap-around for front)
  float front_avg = calculateAverage(ranges, FRONT_START, FRONT_END);
  float right_avg = calculateAverage(ranges, RIGHT_START, RIGHT_END);
  float back_avg = calculateAverageWithWrap(ranges, BACK_START, BACK_END);
  float left_avg = calculateAverage(ranges, LEFT_START, LEFT_END);
  
  // Clear screen and display clean output
  //printf("\033[2J\033[1;1H");  // Clear terminal screen
//  printf("=== LIDAR CARDINAL DIRECTIONS ===\n");
//  printf("Front:  %6.3f m\n", front_avg);
//  printf("Right:  %6.3f m\n", right_avg);
//  printf("Back:   %6.3f m\n", back_avg);
//  printf("Left:   %6.3f m\n", left_avg);
//  printf("RPM:    %d\n", rpms);
 // printf("===============================\n");
}

float LFCDLaser::calculateAverage(const std::vector<float>& ranges, int start_angle, int end_angle)
{
  float sum = 0.0;
  int count = 0;
  
  for (int i = start_angle; i <= end_angle; i++) {
    if (ranges[i] > 0.1 && ranges[i] < 10.0) { // Filter valid ranges (0.1m to 10m)
      sum += ranges[i];
      count++;
    }
  }
  
  return (count > 0) ? sum / count : 0.0;
}

float LFCDLaser::calculateAverageWithWrap(const std::vector<float>& ranges, int start_angle, int end_angle)
{
  float sum = 0.0;
  int count = 0;
  
  // Handle wrap-around (for front: 355-359 and 0-5)
  for (int i = start_angle; i <= 359; i++) {
    if (ranges[i] > 0.1 && ranges[i] < 10.0) {
      sum += ranges[i];
      count++;
    }
  }
  for (int i = 0; i <= end_angle; i++) {
    if (ranges[i] > 0.1 && ranges[i] < 10.0) {
      sum += ranges[i];
      count++;
    }
  }
  
  return (count > 0) ? sum / count : 0.0;
}

void LFCDLaser::sendToOpenCR(const std::vector<float>& ranges)
{
  try {
    // Open serial connection to OpenCR (/dev/ttyS1)
    static boost::asio::io_service io_service;
    static boost::asio::serial_port opencr_serial(io_service, "/dev/ttyS1");
    opencr_serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
    
    // Calculate cardinal directions
    const int FRONT_START = 150;
    const int FRONT_END = 210;
    const int RIGHT_START = 60;
    const int RIGHT_END = 95;
    const int BACK_START = 325;
    const int BACK_END = 35;
    const int LEFT_START = 265;
    const int LEFT_END = 300;
    
    float front = calculateAverage(ranges, FRONT_START, FRONT_END);
    float right = calculateAverage(ranges, RIGHT_START, RIGHT_END);
    float back = calculateAverageWithWrap(ranges, BACK_START, BACK_END);
    float left = calculateAverage(ranges, LEFT_START, LEFT_END);
    
    // Create data string to send
    // Format: "LIDAR:front,right,back,left\n"
    std::string data = "LIDAR:" + 
                      std::to_string(front) + "," +
                      std::to_string(right) + "," +
                      std::to_string(back) + "," +
                      std::to_string(left) + "\n";
    
    // Send data to OpenCR
    boost::asio::write(opencr_serial, boost::asio::buffer(data.c_str(), data.length()));
    
  } catch (std::exception& e) {
    printf("Error sending to OpenCR: %s\n", e.what());
  }
}

}

int main(int argc, char **argv)
{
  std::string port;
  int baud_rate;
  uint16_t rpms;
  port = "/dev/ttyUSB0";
  baud_rate = 230400;
  boost::asio::io_service io;

  try
  {
    lds::LFCDLaser laser(port, baud_rate, io);

    while (1)
    {
      laser.poll();
    }
    laser.close();

    return 0;
  }
  catch (boost::system::system_error ex)
  {
    printf("An exception was thrown: %s", ex.what());
    return -1;
  }
}
