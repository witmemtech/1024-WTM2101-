import os
import numpy as np
import torch
import onnx

from rnnoise import RNNoise

def save_input():
    scal=4
    data = np.memmap("./model/training.bin", dtype='float32', mode='r')
    data=data*scal
    value = data.round()
    np.save("./model/input.npy", value)


def save_node_weight(weight_path):
    torch_weight = torch.load(weight_path)

    save_path = os.path.join(os.path.dirname(weight_path),"params")
    if not os.path.exists(save_path):
        os.mkdir(save_path)

    i=0
    for key in torch_weight.keys():
        key1 = key.replace('.', '_')
        item_save_weightpath = os.path.join(save_path,"{}.npy".format(key1))
        value = (torch_weight[key]*1024).round().clip(min = -128, max = 127)
        # np.savetxt("test%d.txt"%i,value)
        i=i+1
        np.save(item_save_weightpath, value.detach().cpu().numpy())

if __name__ == '__main__':

    net = RNNoise()

    # 存储模型每个结点的权重
    weight_path = './output/checkpoints/denoise_50.pth'
    save_node_weight(weight_path)

    # onnx格式转换
    onnx_save_path = os.path.join("model", "Model_denoise.onnx")
    weight_root = os.path.join("model", "params")
    net.onnx_convert(weight_root, onnx_save_path)

    print('{} model conversion completed!'.format(onnx_save_path))
