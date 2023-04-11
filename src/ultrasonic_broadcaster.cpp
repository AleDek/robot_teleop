
#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <tf/transform_broadcaster.h>


#define N_SENSOR 7

float vectDistCm[N_SENSOR];
ros::Time tstamp;
const float angles[N_SENSOR]={-1.570,-1.0470,-0.5230,0.00,0.5230,1.0470,1.570};
float r_skirt = 0.10; 

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
  for(int i =0; i< N_SENSOR ; i++)
  {
    vectDistCm[i] = array->data.at(i);
  }

  return;
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "ultrasonicNode");

  ros::NodeHandle node;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  ros::Subscriber subUs = node.subscribe("/robot_base/ultrasonic_data", 100, arrayCallback);
  ros::Publisher pubs[N_SENSOR];

  char topicName[20];
  for(int i =0;i<7;i++){
    sprintf(topicName,"range%d_data",i+1);
    pubs[i] = node.advertise<sensor_msgs::Range>( topicName, 10);
  }
  
  float Sin[N_SENSOR];
  float Cos[N_SENSOR];
  for(int i =0;i<N_SENSOR;i++){
    Sin[i] = sin(angles[i]);
    Cos[i] = cos(angles[i]);
  }
  
  const char* frameid[N_SENSOR] = {"us1_ranger","us2_ranger","us3_ranger","us4_ranger","us5_ranger","us6_ranger","us7_ranger"};
  sensor_msgs::Range range_msg;
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 22.0*3.14/180.0;
  range_msg.min_range = 0.010;  
  range_msg.max_range = 4.0;   

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    tstamp =  ros::Time::now();

    for(int i=0;i<N_SENSOR;i++){
      transform.setOrigin( tf::Vector3(r_skirt*Cos[i],r_skirt*Sin[i], 0.1550) );
      q.setRPY(0, 0, angles[i]);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, tstamp, "base_link", frameid[i]));

      range_msg.header.frame_id =  frameid[i];
      range_msg.range = vectDistCm[i]/100.00;
      range_msg.header.stamp = tstamp;
      pubs[i].publish(range_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}