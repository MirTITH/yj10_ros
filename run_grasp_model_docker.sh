#!/bin/bash

set -e

script_dir_rel=$(dirname "$0")
script_dir=$(cd "${script_dir_rel}"; pwd)
grasp_model_dir=$(cd "${script_dir}/grasp_model"; pwd)

echo "grasp_model_dir: $grasp_model_dir"

docker run --rm -it --network=host --ipc=host --runtime=nvidia --gpus all -v "${grasp_model_dir}":"/workspace/grasp_model" grasp_model_he
