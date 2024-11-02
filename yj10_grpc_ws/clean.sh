#!/bin/bash

set -e

script_dir=$(cd $(dirname $0); pwd)

rm -rf $script_dir/build
rm -rf $script_dir/install

