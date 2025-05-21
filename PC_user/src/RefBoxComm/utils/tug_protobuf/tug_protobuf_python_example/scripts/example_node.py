#!/usr/bin/env python
from tug_protobuf_example_msg.msg import example_msg_pb2

if __name__ == "__main__":
    print "tug_protobuf_python_example_node"
    example_person = example_msg_pb2.Person()
    example_person.name = "Example_name"
    print example_person
