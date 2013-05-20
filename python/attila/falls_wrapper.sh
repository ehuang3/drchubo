#!/bin/bash


USE_DIR="$1"
if [ -n USE_DIR ]
then
    rm ../data/nmf_dir
    ln -s $USE_DIR ../data/nmf_dir
fi

# if [ -n $ROS_PACKAGE_PATH ]
# then
#     source /usr/share/drcsim-2.5/setup.sh
#     export $ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
# fi

roslaunch attila nine_million_falls.launch
