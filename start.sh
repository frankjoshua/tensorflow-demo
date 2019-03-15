#!/bin/bash

docker run -it -p 8888:8888 -v $PWD:/notebooks tensorflow/tensorflow # Start a Jupyter notebook server 