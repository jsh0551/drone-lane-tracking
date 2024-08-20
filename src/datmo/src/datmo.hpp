/*
 * Copyright (c) 2020, Robobrain.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Konstantinos Konstantinidis */

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include <math.h>       /* atan */
#include <omp.h>      //Multi-threading
#include <vector>
#include <random>
#include <algorithm> // for sort(), min()
#include <chrono>
#include <iostream>
#include <fstream>

// #include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/msg/laser_scan.hpp"
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <geometry_msgs/Point.h>
#include "geometry_msgs/msg/point.hpp"

// #include <tf/tf.h>
// #include <tf2_ros/transform_listener.h>

// #include <datmo/TrackArray.h>
// #include <datmo/Track.h>
#include "datmo_msg/msg/track_array.hpp" 

#include "cluster.hpp"

typedef std::pair<double, double> Point;
typedef std::vector<double> l_shape;
typedef std::vector<l_shape> l_shapes;
typedef std::vector<Point> pointList;


using namespace std;
// This node segments the point cloud based on the break-point detector algorithm.
// This algorithm is based on "L-Shape Model Switching-Based Precise Motion Tracking 
// of Moving Vehicles Using Laser Scanners.

class Datmo  : public rclcpp::Node
// class Datmo
{
public:
  Datmo();
  ~Datmo();

  // void callback(const sensor_msgs::LaserScan::ConstPtr &);
  // void Clustering(const sensor_msgs::LaserScan::ConstPtr& , vector<pointList> &);
  // void visualiseGroupedPoints(const vector<pointList> &);
  // void transformPointList(const pointList& , pointList& );

  void callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in);
  void Clustering(const sensor_msgs::msg::LaserScan::SharedPtr scan_in , vector<pointList> &clusters);
  // void publish_data(const datmo_msg::msg::TrackArray::SharedPtr data);

  // tf::TransformListener tf_listener;

  // explicit Datmo(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  // virtual ~Datmo();

private:
  // ros::Publisher pub_marker_array; 
  // ros::Publisher pub_tracks_box_kf;
  // ros::Subscriber sub_scan;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
  // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub;
  rclcpp::Publisher<datmo_msg::msg::TrackArray>::SharedPtr pub_tracks_box_kf;

  // sensor_msgs::LaserScan scan;
  vector<Cluster> clusters;

  std::shared_ptr<sensor_msgs::msg::LaserScan> scan;


  //Tuning Parameteres
  double dt;
  // ros::Time time;
  rclcpp::Time time;

  rclcpp::Time time1;
  rclcpp::Time time2;

  //initialised as one, because 0 index take the msgs that fail to be initialized
  unsigned long int cg       = 1;//group counter to be used as id of the clusters
  unsigned long int cclusters= 1;//counter for the cluster objects to be used as id for the markers

  //Parameters
  double dth;
  double euclidean_distance;
  int max_cluster_size;
  bool p_marker_pub;
  bool w_exec_times;
  string lidar_frame;
  string world_frame;

  // rclcpp::Publisher<ArithmeticArgument>::SharedPtr arithmetic_argument_publisher_;
  // rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  // rclcpp::AsyncParametersClient::SharedPtr parameters_client_;

};
