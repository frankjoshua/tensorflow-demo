#!/bin/bash

docker run -it -p 8888:8888 -p 6006:6006 -v $PWD/notebooks:/tf tensorflow/tensorflow:latest-py3-jupyter # Start a Jupyter notebook server 