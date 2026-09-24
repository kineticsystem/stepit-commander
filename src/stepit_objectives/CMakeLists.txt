cmake_minimum_required(VERSION 3.16)
project(stepit_objectives)

find_package(ament_cmake REQUIRED)

# The objectives, and the subtrees they are built from, are plain XML files in
# objectives/: install them where the commander server can find them. The
# folder is listed in the `behavior_trees` parameter of the server.
#
# stepit_behaviors.xml, the node models of the behaviors, sits next to the
# objectives so that an editor opening the folder knows every node type. It is
# left out of the install on purpose: it holds no tree for the server to load,
# and the server learns the behaviors from the plugin.
install(DIRECTORY objectives DESTINATION share/${PROJECT_NAME}/
  PATTERN stepit_behaviors.xml EXCLUDE)

ament_package()
