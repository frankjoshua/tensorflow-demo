#!/bin/bash

docker run -it -p 8888:8888 -p 6006:6006 -v $PWD:/notebooks tensorflow/tensorflow # Start a Jupyter notebook server 