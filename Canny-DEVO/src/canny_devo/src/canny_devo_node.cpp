#include <canny_devo/canny_devo_offline.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <ros/ros.h>
#include <string>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "canny_devo_node" );

  ros::NodeHandle nh;

  image_transport::ImageTransport it( nh );

  std::string config_path( argv[1] );

  canny_devo::offline::setVerbose();

  canny_devo::offline::setVisualize();

  canny_devo::offline::prelude( config_path );

  canny_devo::offline::init( &nh, &it );

  canny_devo::offline::dataloading();

  canny_devo::offline::main();

  ROS_INFO( "finished..." );

  ros::shutdown();

  return 0;
}