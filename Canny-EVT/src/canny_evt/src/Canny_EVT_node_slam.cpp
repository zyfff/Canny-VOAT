#include <canny_evt/canny_evt_offline.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <ros/ros.h>
#include <string>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "canny_evt_node" );

  ros::NodeHandle nh;

  image_transport::ImageTransport it( nh );

  std::string config_path( argv[1] );

  canny_evt::offline::setVerbose();

  canny_evt::offline::setVisualize();

  canny_evt::offline::prelude( config_path );

  canny_evt::offline::init( &nh, &it );

  canny_evt::offline::dataloading();

  canny_evt::offline::main();

  ROS_INFO( "finished..." );

  ros::shutdown();

  return 0;
}