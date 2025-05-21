#include <iostream>
#include <fstream>
#include <string>
#include <tug_protobuf_example_msg/example_msg.pb.h>


int main(int /*argc*/, char* /*argv*/[])
{
  std::cout << "tug_protobuf_cpp_example_node" << std::endl;

  example::Person example_person;  
  example_person.set_name("example name");

  std::cout << "name: " << example_person.name() << std::endl;
}
