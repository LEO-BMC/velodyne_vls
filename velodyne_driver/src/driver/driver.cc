/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017, Velodyne LiDAR INC., Algorithms and Signal Processing Group
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "driver.h"

namespace velodyne_driver {
static double prev_frac_packet = 0;
inline std::string toBinary(int n) {
  std::string r;
  while (n != 0) {
    r = (n % 2 == 0 ? "0" : "1") + r;
    n /= 2;
  }
  while (r.length() != 8) {
    r = '0' + r;
  }
  return r;
}

inline double convertBinaryToDecimal(std::string binaryString) {
  double value = 0;
  int indexCounter = 0;
  for (int i = binaryString.length() - 1; i >= 0; i--) {

    if (binaryString[i] == '1') {
      value += pow(2, indexCounter);
    }
    indexCounter++;
  }
  return value;
}

inline double computeTimeStamp(velodyne_msgs::VelodyneScanPtr scan, int index) {

  std::string digit4 = toBinary(scan->packets[index].data[1203]);
  std::string digit3 = toBinary(scan->packets[index].data[1202]);
  std::string digit2 = toBinary(scan->packets[index].data[1201]);
  std::string digit1 = toBinary(scan->packets[index].data[1200]);
  std::string digit = digit4 + digit3 + digit2 + digit1; // string concatenation
  double value = convertBinaryToDecimal(digit);
  // compute the seconds from the beginning of that hour to when the data being captured
  double time_stamp = (double) value / 1000000;
  return time_stamp;
}

/** Utility function for Velodyne Driver
 *  gets the number of laser beams fired concurrently 
 *  for different sensor models 
*/

inline int get_concurrent_beams(uint8_t sensor_model) {
/*
Strongest 0x37 (55)   HDL-32E 0x21 (33)
Last Return 0x38 (56) VLP-16 0x22 (34)
Dual Return 0x39 (57) Puck LITE 0x22 (34)
         -- --        Puck Hi-Res 0x24 (36)
         -- --        VLP-32C 0x28 (40)
         -- --        Velarray 0x31 (49)
         -- --        VLS-128 0xA1 (161)
*/

  switch (sensor_model) {
    case 33:return (2); // hdl32e
    case 34:return (1); // vlp16 puck lite
    case 36:return (1); // puck hires  (same as vlp16 ?? need to check)
    case 40:return (2); // vlp32c
    case 49:return (2); // velarray
    case 161:return (8); // vls128
    case 99:return (8); // vls128
    default:ROS_WARN_STREAM(
          "[Velodyne Ros driver]Default assumption of device id .. Defaulting to HDL64E with 2 simultaneous firings");
      return (2); // hdl-64e

  }
}

/** Utility function for Velodyne Driver
 *  gets the number of packet multiplier for dual return mode vs 
 *  single return mode 
*/

inline int get_rmode_multiplier(uint8_t sensor_model, uint8_t packet_rmode) {
  /*
     HDL64E 2
     VLP32C 2
     HDL32E 2
     VLS128 3
     VLSP16 2
 */
  if (packet_rmode == 57) {
    switch (sensor_model) {
      case 33:return (2); // hdl32e
      case 34:return (2); // vlp16 puck lite
      case 36:return (2); // puck hires
      case 40:return (2); // vlp32c
      case 49:return (2); // velarray
      case 161:return (3); // vls128
      case 99:return (3); // vls128
      default:ROS_WARN_STREAM(
            "[Velodyne Ros driver]Default assumption of device id .. Defaulting to HDL64E with 2x number of packekts for Dual return");
        return (2); // hdl-64e
    }
  } else {
    return (1);
  }
}

/** Utility function for the Velodyne driver 
 *
 *  provides a estimated value for number of packets in 
 *  1 full scan at current operating rpm estimate of the sensor 
 *  This value is used by the poll() routine to assemble 1 scan from 
 *  required number of packets 
 *  @returns number of packets in full scan 
 */

inline int get_auto_npackets(uint8_t sensor_model,
                             uint8_t packet_rmode,
                             double auto_rpm,
                             double firing_cycle,
                             int active_slots) {
  double rps = auto_rpm / 60.0;
  double time_for_360_degree_scan = 1.0 / rps;
  double total_number_of_firing_cycles_per_full_scan = time_for_360_degree_scan / firing_cycle;
  double total_number_of_firings_per_full_scan = total_number_of_firing_cycles_per_full_scan
      * get_concurrent_beams(sensor_model);
  double total_number_of_points_captured_for_single_return = active_slots * total_number_of_firings_per_full_scan;
  double total_number_of_packets_per_full_scan = total_number_of_points_captured_for_single_return / 384;
  double total_number_of_packets_per_second = total_number_of_packets_per_full_scan / time_for_360_degree_scan;
  double auto_npackets = get_rmode_multiplier(sensor_model, packet_rmode)
      * floor((total_number_of_packets_per_full_scan + prev_frac_packet));
  prev_frac_packet =
      get_rmode_multiplier(sensor_model, packet_rmode) * (total_number_of_packets_per_full_scan + prev_frac_packet)
          - auto_npackets;
  return (auto_npackets);
}

/** Utility function for the Velodyne driver 
 *
 *  provides a estimated value for number of packets in 
 *  1 second at current operating rpm estimate of the sensor 
 *  This value is used by the pcap reader (InputPCAP class ) 
 *  to pace the speed of packet reading.
 *  @returns number of packets per second 
 */

inline double get_auto_packetrate(uint8_t sensor_model,
                                  uint8_t packet_rmode,
                                  double auto_rpm,
                                  double firing_cycle,
                                  int active_slots) {
  double rps = auto_rpm / 60.0;
  double time_for_360_degree_scan = 1.0 / rps;
  double total_number_of_firing_cycles_per_full_scan = time_for_360_degree_scan / firing_cycle;
  double total_number_of_firings_per_full_scan = total_number_of_firing_cycles_per_full_scan
      * get_concurrent_beams(sensor_model);
  double total_number_of_points_captured_for_single_return = active_slots * total_number_of_firings_per_full_scan;
  double total_number_of_packets_per_full_scan = total_number_of_points_captured_for_single_return / 384;
  double total_number_of_packets_per_second = total_number_of_packets_per_full_scan / time_for_360_degree_scan;
  return ((get_rmode_multiplier(sensor_model, packet_rmode) * total_number_of_packets_per_second));
}
/** Constructor for the Velodyne driver 
 *
 *  provides a binding to ROS node for processing and 
 *  configuration 
 *  @returns handle to driver object
 */

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh) {
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));
  double packet_rate;                   // packet frequency (Hz)
  std::string model_full_name;
  if ((config_.model == "64E_S2") ||
      (config_.model == "64E_S2.1"))    // generates 1333312 points per second
  {                                   // 1 packet holds 384 points
    packet_rate = 3472.17;            // 1333312 / 384
    model_full_name = std::string("HDL-") + config_.model;
    slot_time = 1.2e-6; // basic slot time
    num_slots = 116;                     // number of active + maintenence slots
    active_slots = 32;                  // number of active slots
  } else if (config_.model == "64E") {
    packet_rate = 2600.0;
    model_full_name = std::string("HDL-") + config_.model;
    slot_time = 1.2e-6; // basic slot time
    num_slots = 116;                     // number of slots
    active_slots = 32;                  // number of active slots
  } else if (config_.model == "32E") {
    packet_rate = 1808.0;
    model_full_name = std::string("HDL-") + config_.model;
    slot_time = 1.152e-6; // basic slot time
    num_slots = 40;                     // number of slots
    active_slots = 32;                  // number of active slots
  } else if (config_.model == "VLP32C") {
    packet_rate = 3014; // 12 groups of 32 firings where a pair of 2 firings corresponds to 55.296us -> 1/(12*55.296us)
    model_full_name = "VLP-32C";
    slot_time = 2.304e-6; // basic slot time
    num_slots = 24;                     // number of slots
    active_slots = 16;                  // number of active slots
  } else if (config_.model == "VLS128") {
    packet_rate = 12507; // 3 groups of 128 firings where a set of 8 firings corresponds to 55.296us -> 1/(12*55.296us)
    model_full_name = "VLS-128";
    slot_time = 2.665e-6;              // basic slot time
    num_slots = 20;                    // number of slots
    active_slots = 16;                 // number of active slots
  } else if (config_.model == "VLP16") {
    packet_rate = 1507;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
    model_full_name = "VLP-16";
    //     slot_time = 2.304e-6; // basic slot time
    slot_time = 2.304e-6; // basic slot time
    num_slots = 24;                     // number of slots
    active_slots = 16;                  // number of active slots
  } else {
    ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
    packet_rate = 2600.0;
    slot_time = 1.2e-6;                  // basic slot time
    num_slots = 116;                     // number of slots
    active_slots = 32;                   // number of active slots
  }
  std::string deviceName(std::string("Velodyne ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  private_nh.getParam("rpm", config_.rpm);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate
  auto_rpm = config_.rpm;

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  int npackets = (int) ceil(packet_rate / frequency);
  private_nh.param("npackets", config_.npackets, npackets);
  private_nh.getParam("npackets", config_.npackets);
  private_nh.setParam("npackets", npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");
  private_nh.param("auto_rpm_alpha", auto_alpha, 0.999);
  private_nh.getParam("auto_rpm_alpha", auto_alpha);
  ROS_INFO_STREAM("Automatic RPM smoothing coeff " << auto_alpha << " (1 means no tracking, zero means no smoothing) ");

  private_nh.param("pcap", dump_file, std::string(""));

  int udp_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne_driver::
                                                        VelodyneNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig>::
  CallbackType f;
  f = boost::bind(&VelodyneDriver::callback, this, _1, _2);
  srv_->setCallback(f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  if (dump_file != "")                  // have PCAP file?
  {
    // read data from packet capture file
    input_.reset(new velodyne_driver::InputPCAP(private_nh, udp_port,
                                                packet_rate, dump_file));
  } else {
    // read data from live socket
    input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port));
  }

  // raw packet output topic
  output_ =
      node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
  firing_cycle = slot_time * num_slots; // firing cycle time


  double revs_per_sec = config_.rpm / 60.0f;
  millis_revolution_ = static_cast<int>(std::round(1000.0 / revs_per_sec));
  ROS_INFO_STREAM("millis_revolution = " << millis_revolution_);
  millis_target_next_ = 0;
  millis_initialized_ = false;
}

/** poll the device
 *
 * poll is used by nodelet to bind to the ROS thread.
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void) {
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.


  auto get_time_millis = []() {
    return static_cast<uint64_t>(std::floor(ros::Time::now().toSec() * 1000));
  };

  if (!millis_initialized_) {
    uint64_t time_since_beginning = get_time_millis() / millis_revolution_ - 1;
    millis_target_next_ = time_since_beginning * millis_revolution_;
    millis_initialized_ = true;
  }

  while (millis_target_next_ < get_time_millis()) {
    millis_target_next_ += millis_revolution_;
    ROS_INFO_STREAM("hmm");
  }

  scan->packets.resize(config_.npackets * 1.2);

  int index_packet = 0;
  bool millis_switch_ = false;
  while (!millis_switch_) {
    if (get_time_millis() > millis_target_next_) {
      millis_target_next_ += millis_revolution_;
      millis_switch_ = true;
    }
    while (true) {
      // keep reading until full packet received
      int rc = input_->getPacket(&scan->packets[index_packet], config_.time_offset);
      if (rc == 0) break;       // got a full packet?
      if (rc < 0) return false; // end of file reached?
    }

    index_packet++;
    if (index_packet > scan->packets.size() - 1) {
      ROS_WARN_STREAM("index_packet is too big");
      break;
    }
  }
  scan->packets.resize(index_packet);




  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");

  uint64_t millis_target_start = millis_target_next_ - millis_revolution_;

  scan->header.stamp.fromNSec(millis_target_start * 1000000);

  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  return true;
}

void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
                              uint32_t level) {
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
  auto_alpha = config.auto_rpm_alpha;
}

} // namespace velodyne_driver
