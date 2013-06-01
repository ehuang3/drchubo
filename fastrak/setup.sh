#!/bin/bash -e

# Become root
if [ $UID -ne 0 ]; then
        echo "-- Becoming root"
        exec sudo $0
fi

echo "-- Updating apt sources"
echo "deb http://code.golems.org/debian precise golems.org" | sudo tee -a /etc/apt/sources.list

echo "-- Installing packages"
apt-get update
apt-get install autoconf automake libtool autoconf-archive libach-dev ach-utils \
 libblas3gf libblas-dev liblapack3gf liblapack-dev maxima libamino-dev gfortran \
  protobuf-c-compiler libprotobuf-c0-dev libamino-dev
