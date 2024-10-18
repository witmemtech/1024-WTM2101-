"""
/* Copyright (c) 2024 Jean-Marc Valin */
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
"""

import torch
from torch import nn
import torch.nn.functional as F
import sys
import os
import numpy as np
import onnx
from onnx import helper
from net_base import WitmemBaseNet
from base import *

sys.path.append(os.path.join(os.path.split(__file__)[0], '..'))


class RNNoise(WitmemBaseNet):
    def __init__(self, input_dim=42, output_dim=22):
        super(RNNoise, self).__init__()

        self.input_dim = input_dim
        self.output_dim = output_dim

        self.model_input_shape = [1, input_dim]
        self.model_output_shape = [1, output_dim]
        self.model_in_name = 'in'
        self.model_out_name = 'out'

        self.layer1 = nn.Linear(input_dim, 256, bias=False)
        self.relu1 = nn.ReLU(True)
        self.layer2 = nn.Linear(256, 256, bias=False)
        self.relu2 = nn.ReLU(True)
        self.layer3 = nn.Linear(256, output_dim, bias=False)
        self.sigmoid3 = nn.Sigmoid()
        self.quant_layer = dict()


    def forward(self, features):
        x = self.layer1(features)
        x = self.relu1(x)
        x = self.layer2(x)
        x = self.relu2(x)
        x = self.layer3(x)
        out = self.sigmoid3(x)
        return out

    def forward_quant(self, features):
        def quant_linear(src_layer, _input, name, G=1024):
            if name not in self.quant_layer:
                w = src_layer.weight.data
                w_temp = G * w
                w_temp = torch.clip_(torch.round(w_temp), -128, 127)
                b = None
                self.quant_layer[name] = [w_temp, b]
            output = F.linear(_input, self.quant_layer[name][0])
            output /= G
            output = torch.clip_(torch.round(output), -128, 127)
            return output



        output1 = quant_linear(self.layer1, features, "layer_1")
        output1 = self.relu1(output1)

        output1 = quant_linear(self.layer2, output1, "layer_2")
        output1 = self.relu2(output1)

        output1 = quant_linear(self.layer3, output1, "layer_3")
        output1 = self.sigmoid3(output1/4)
        return output1

    def onnx_convert(self, weight_root, onnx_save_path):
        node_linear_1, w_b = self.makeNodeLinear(os.path.join(weight_root, 'layer1_weight.npy'), idx=1,
                                                 input_name=self.model_in_name)
        node_scale1 = self.makeNodeScale('linear_out1', 'scale_out1', 1024)
        node_relu1 = self.makeNodeRelu('scale_out1', 'relu_out1')
        self.all_nodes.extend([node_linear_1, node_scale1, node_relu1])
        self.initializer.extend(w_b)

        node_linear_2, w_b = self.makeNodeLinear(os.path.join(weight_root, 'layer2_weight.npy'), idx=2,
                                                 input_name='relu_out1')
        node_scale2 = self.makeNodeScale('linear_out2', 'scale_out2', 1024)
        node_relu2 = self.makeNodeRelu('scale_out2', 'relu_out2')
        self.all_nodes.extend([node_linear_2, node_scale2, node_relu2])
        self.initializer.extend(w_b)

        node_linear_3, w_b = self.makeNodeLinear(os.path.join(weight_root, 'layer3_weight.npy'), idx=3,
                                                 input_name='relu_out2')
        node_scale3 = self.makeNodeScale('linear_out3', self.model_out_name, 1024)
        self.all_nodes.extend([node_linear_3, node_scale3])
        self.initializer.extend(w_b)

        inputs = [
            helper.make_tensor_value_info(self.model_in_name, onnx.TensorProto.FLOAT, list(self.model_input_shape))]
        outputs = [
            helper.make_tensor_value_info(self.model_out_name, onnx.TensorProto.FLOAT, list(self.model_output_shape))]

        graph = helper.make_graph(self.all_nodes, 'model', inputs, outputs, self.initializer)
        model = helper.make_model(graph)

        with open(onnx_save_path, "wb") as of:
            of.write(model.SerializeToString())


if __name__ == "__main__":
    model = RNNoise()
    inpt = np.random.random([32, 1, 42]).astype(np.float32)
    inpt = torch.from_numpy(inpt)

    denoise_output, vad_output = model(inpt)
    print("denoise_output")
    print(denoise_output.shape)
    print("vad_output")
    print(vad_output.shape)
