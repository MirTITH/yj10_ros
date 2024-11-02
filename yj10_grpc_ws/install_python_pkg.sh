#!/bin/bash

set -e

script_dir=$(cd $(dirname $0); pwd)
python3 -m pip install $script_dir/build/grasp_model_grpc_msg/proto/python