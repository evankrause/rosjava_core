This repo contains both a full pure Java implementation of ROS, and an additional wrapper layer to make using ROS from DIARC easier.

To use DIARC with ROS, clone this repo and run the `./gradlew buildAndPublishDiarcRos` command.

This will generate rosjava messages for
any ROS packages you have installed and sourced on your machine. If you add/install more packages later, you'll need to run the 'buildAndPublishDiarcRos'
command again to include any new messages that are on your system.
