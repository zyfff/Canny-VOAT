#include <canny_evt_minus/canny_evt_minus_offline.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <ros/ros.h>
#include <string>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "canny_evt_minus_node" );

  ros::NodeHandle nh;

  image_transport::ImageTransport it( nh );

  std::string config_path( argv[1] );

  canny_evt_minus::offline::setVerbose();

  canny_evt_minus::offline::setVisualize();

  canny_evt_minus::offline::prelude( config_path );

  canny_evt_minus::offline::init( &nh, &it );

  canny_evt_minus::offline::dataloading();

  canny_evt_minus::offline::main();

  ROS_INFO( "finished..." );

  ros::shutdown();

  return 0;
}