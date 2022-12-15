#include <canny_vt/canny_vt_offline.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <ros/ros.h>
#include <string>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "canny_vt_node" );

  ros::NodeHandle nh;

  image_transport::ImageTransport it( nh );

  std::string config_path( argv[1] );

  canny_vt::offline::setVerbose();

  canny_vt::offline::setVisualize();

  canny_vt::offline::prelude( config_path );

  canny_vt::offline::init( &nh, &it );

  canny_vt::offline::dataloading();

  canny_vt::offline::main();

  ROS_INFO( "finished..." );

  ros::shutdown();

  return 0;
}