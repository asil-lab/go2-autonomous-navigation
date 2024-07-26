#!/bin/bash
# set -e

# # Install ONNX Runtime for inference and training acceleration
# cd /tmp
# wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz
# tar xf onnxruntime-linux-x64-1.7.0.tgz
# mkdir -p ~/.local/bin ~/.local/include/onnxruntime ~/.local/lib ~/.local/share/cmake/onnxruntime
# rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ ~/.local/include/onnxruntime
# rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ ~/.local/lib
# rsync -a ~/git/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/cmake/onnxruntime
