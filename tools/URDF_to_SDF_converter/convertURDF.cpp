/**
 * @file convertURDF.cpp
 */
#include <sdf/interface/parser_urdf.hh>

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  urdf2gazebo::URDF2Gazebo u2g;
  TiXmlDocument xml_sdf;
  xml_sdf = u2g.InitModelFile( argv[1] );
  xml_sdf.SaveFile("test.sdf" );

  return 0;
}
