import onnx
import numpy as np
import torch
import os


# def sigmoid(x):
#     sig = 1 / (1 + np.exp(-x))
#     return sig


# def tanh_q(x, qx, qy):
#     x = x / (2**qx)
#     x = np.tanh(x)
#     x = (x * (2**qy)).round().clip(-128, 127)
#     return x


# def sigmoid_q(x, qx, qy):
#     x = x / (2 ** qx)
#     x = sigmoid(x)
#     x = (x * (2 ** qy)).round().clip(-128, 127)
#     return x


def tanh_q(x, qx, qy):
    x = x / (2 ** qx)
    x = torch.tanh(x)
    x = (x * (2 ** qy)).round().clamp(min=-128, max=127)
    return x


def sigmoid_q(x, qx, qy):
    x = x / (2 ** qx)
    x = torch.sigmoid(x)
    x = (x * (2 ** qy)).round().clamp(min=0, max=127)
    return x


def shift_hardware(x, shift):
    y = x.int()
    value = ((y >> (shift - 1)) + 1) >> 1
    return value.float()

def div_hardware(x, div):
    ki = (x / (div / 4)).int()
    kv = torch.floor(ki / 4.0 + 0.5)
    return kv

def sigmoid(x):
    sig = 1 / (1 + np.exp(-x))
    return sig

# def gelu(x):
#     return x*sigmoid(1.702*x)

def gelu(x):
    return 0.5 * x * ( 1 + torch.tanh(x * 0.7978845608 * ( 1 + 0.044715 * x * x)))

# act_input1024 = [i for i in range(512)] + [i - 512 for i in range(512)]
#     act_input1024 = np.array(act_input1024)

def gelu_scale_table1024(x, scale_x, scale_y):
  x = x / scale_x
  x = gelu(x)
  x = (x * scale_y).round().clip(-128, 127)
  return x


def makeNodeScale(input_name, output_name, G):
    node_scale = onnx.helper.make_node("Scale", inputs=[input_name], outputs=[output_name], scale=1 / G)
    return node_scale


def makeNodeRelu(input_name, output_name):
    node_relu = onnx.helper.make_node('Relu', [input_name], [output_name])
    return node_relu


def makeNodeSigmod(input_name, output_name):
    node_sigmoid = onnx.helper.make_node('Sigmoid', [input_name], [output_name])
    return node_sigmoid


def makeNodeTanh(input_name, output_name):
    node_tanh = onnx.helper.make_node('Tanh', [input_name], [output_name])
    return node_tanh


def makeNodeAdd(input_name, output_name):
    node_add = onnx.helper.make_node('Add', input_name, [output_name])
    return node_add

def makeNodeLinear(weight_path, idx, input_name):
    weight = np.load(weight_path)
    weight_t_name = 'weight_tensor' + str(idx)
    bias_t_name = 'bias_tensor' + str(idx)

    weight_t = onnx.helper.make_tensor(weight_t_name, data_type=onnx.TensorProto.FLOAT, dims=np.shape(np.transpose(weight)),
                                  vals=np.transpose(weight).flatten())
    bias_t = onnx.helper.make_tensor(bias_t_name, data_type=onnx.TensorProto.FLOAT, dims=[weight.shape[0]],
                                vals=np.zeros(weight.shape[0]))

    node_conv2d = onnx.helper.make_node('Gemm',
                                   inputs=[input_name, weight_t_name, bias_t_name],
                                   outputs=['linear_out' + str(idx)],
                                   name='linear_' + str(idx)
                                   )
    return node_conv2d, [weight_t, bias_t]

def makeNodeLinear_tmp(weight, idx, input_name):
    # weight = np.load(weight_path)
    weight_t_name = 'weight_tensor' + str(idx)
    bias_t_name = 'bias_tensor' + str(idx)

    weight_t = onnx.helper.make_tensor(weight_t_name, data_type=onnx.TensorProto.FLOAT, dims=np.shape(np.transpose(weight)),
                                  vals=np.transpose(weight).flatten())
    bias_t = onnx.helper.make_tensor(bias_t_name, data_type=onnx.TensorProto.FLOAT, dims=[weight.shape[0]],
                                vals=np.zeros(weight.shape[0]))

    node_conv2d = onnx.helper.make_node('Gemm',
                                   inputs=[input_name, weight_t_name, bias_t_name],
                                   outputs=['linear_out' + str(idx)],
                                   name='linear_' + str(idx)
                                   )
    return node_conv2d, [weight_t, bias_t]


def make_concat(input_name1, input_name2, node_name, axis=1, use_relu=False):
    concat_node = onnx.helper.make_node("Concat", inputs=[input_name1, input_name2], outputs=[node_name + "_out"],
                                        axis=1)
    relu_node = onnx.helper.make_node('Relu', inputs=[node_name + "_out"], outputs=[node_name + "_relu"])

    inits = []
    output_name = None
    if use_relu == False:
        nodes = [concat_node]
        output_name = node_name + "_out"
    else:
        nodes = [concat_node, relu_node]
        output_name = node_name + "_relu"
    return inits, nodes, output_name


def make_tensor(filepath, t_name, tranpose=False, dims=0):
    # seps = str.split(filepath, ".")
    # type = seps[-1]
    # array = None
    # if type == "txt":
    #     array=np.loadtxt(filepath).astype("float32")
    # elif type == "npy":
    #     array=np.load(filepath).astype("float32")
    # if tranpose == True:
    #     array = np.transpose(array)
    # if dims != 0:
    #     dims = dims
    # else:
    #     dims = np.shape(array)
    # tensor_0 = onnx.helper.make_tensor(t_name,
    #                         data_type=onnx.TensorProto.FLOAT,
    #                         dims=dims,
    #                         vals=array.flatten())
    # return tensor_0

    array = filepath
    dims = np.shape(array)
    tensor_0 = onnx.helper.make_tensor(t_name,
                                       data_type=onnx.TensorProto.FLOAT,
                                       dims=dims,
                                       vals=array.flatten())
    return tensor_0


def make_add(input_name1, input_name2, node_name="add", use_relu=False):
    '''
    input_name1,input_name2: 要做add的输入名称
    node: 节点的名称
    use_relu: 是否做relu激活
    '''
    add_node = onnx.helper.make_node('Add',
                                     [input_name1, input_name2],
                                     [node_name + "_out"],
                                     name=node_name
                                     )
    relu_node = onnx.helper.make_node('Relu', inputs=[node_name + "_out"], outputs=[node_name + "_relu"])

    inits = []
    output_name = None
    if use_relu == False:
        nodes = [add_node]
        output_name = node_name + "_out"
    else:
        nodes = [add_node, relu_node]
        output_name = node_name + "_relu"
    return inits, nodes, output_name


# def make_tensor_wb(wb, t_name):
#     array = wb
#     dims = np.shape(array)
#     tensor_0 = onnx.helper.make_tensor(t_name,
#                             data_type=onnx.TensorProto.FLOAT,
#                             dims=dims,
#                             vals=array.flatten())
#     return tensor_0


def make_tdnn(w_file, b_file, gain_scale, offset_nums, offset, inputName, nodeName, isNeedRelu, tranpose=False):
    w_tensor = make_tensor(w_file, nodeName + "_w", tranpose)
    b_tensor = make_tensor(b_file, nodeName + "_b")
    offsets_tensor = onnx.helper.make_tensor(nodeName + "_offset",
                                             data_type=onnx.TensorProto.FLOAT, dims=(offset_nums,), vals=offset)

    tdnn_node = onnx.helper.make_node('Tdnn', inputs=[inputName, nodeName + "_w"],
                                      outputs=[nodeName + "_o"],
                                      time_offsets=offsets_tensor,
                                      bias_params=b_tensor,
                                      scale_params=gain_scale,
                                      name=nodeName
                                      )

    relu_node = onnx.helper.make_node('Relu', inputs=[nodeName + "_o"],
                                      outputs=[nodeName + "_relu"])

    inits = [w_tensor, b_tensor, offsets_tensor]
    outputName = None
    if isNeedRelu == False:
        nodes = [tdnn_node]
        outputName = nodeName + "_o"
    else:
        nodes = [tdnn_node, relu_node]
        outputName = nodeName + "_relu"

    return inits, nodes, outputName


def make_conv(w_file, b_file, kernal, strides, pads, layer_name, gain_scale, inputname, isNeedRelu=True):
    w_tensor = make_tensor(w_file, layer_name + "_w")
    b_tensor = make_tensor(b_file, layer_name + "_b")

    # make node
    conv1_node = onnx.helper.make_node("Conv",
                                       inputs=[inputname, layer_name + "_w", layer_name + "_b"],
                                       outputs=[layer_name + "_o"],
                                       kernel_shape=kernal,
                                       strides=strides,
                                       pads=pads,
                                       name=layer_name)
    conv1_scale_node = onnx.helper.make_node("Scale", inputs=[layer_name + "_o"], outputs=[layer_name + "_scale"],
                                             scale=1.0 / gain_scale)
    conv1_relu_node = onnx.helper.make_node('Relu', [layer_name + "_scale"], [layer_name + "_relu"])

    inits = [w_tensor, b_tensor]
    outputName = None
    if isNeedRelu == False:
        nodes = [conv1_node, conv1_scale_node]
        outputName = layer_name + "_scale"
    else:
        nodes = [conv1_node, conv1_scale_node, conv1_relu_node]
        outputName = layer_name + "_relu"

    return inits, nodes, outputName


def make_linear(w_file, b_file, scale, inputName, nodeName, tranpose=False, isActivate=True, out_scale=1):
    linear_w = make_tensor(w_file, nodeName + "_w", tranpose)
    linear_b = make_tensor(b_file, nodeName + "_b")
    output = nodeName + "_output"

    linear_node = onnx.helper.make_node(
        'Gemm',
        inputs=[inputName, nodeName + "_w", nodeName + "_b"],
        outputs=[output],
        name=nodeName)

    linear_scale = onnx.helper.make_node('Scale', [output],
                                         [nodeName + "_scale"],
                                         scale=scale)

    if isActivate == True:
        linear_relu_node = onnx.helper.make_node('Relu', [nodeName + "_scale"],
                                                 [nodeName + "_relu"])

        inits = [linear_w, linear_b]
        nodes = [linear_node, linear_scale, linear_relu_node]
        return inits, nodes, nodeName + "_relu"
    else:
        inits = [linear_w, linear_b]
        nodes = [linear_node, linear_scale]
        return inits, nodes, nodeName + "_scale"


def make_reshape(input_name, node_name, dim, values):
    output = node_name + "_out"
    reshape_tensor = onnx.helper.make_tensor(node_name + "_reshape",
                                             data_type=onnx.TensorProto.FLOAT,
                                             dims=(dim,),
                                             vals=values)
    reshape_node = onnx.helper.make_node(
        "Reshape",
        inputs=[input_name, node_name + "_reshape"],
        outputs=[output]
    )

    return [reshape_tensor], [reshape_node], output


def make_deconv(w_file, b_file, kernels, strides, pads, gainScale, outpads, inputName, nodeName, isNeedRelu=False):
    dconv_w_tensor = make_tensor(w_file, nodeName + "_w")
    dconv_b_tensor = make_tensor(b_file, nodeName + "_b")

    deconv_node = onnx.helper.make_node(
        "ConvTranspose",
        inputs=[inputName, nodeName + "_w", nodeName + "_b"],
        outputs=[nodeName],
        kernel_shape=kernels,
        strides=strides,
        pads=pads,
        output_padding=outpads,
        name=nodeName)
    deconv_scale_node = onnx.helper.make_node("Scale",
                                              inputs=[nodeName],
                                              outputs=[nodeName + '_scale'],
                                              scale=1.0 / gainScale)

    if isNeedRelu:
        deconv_relu_node = onnx.helper.make_node('Relu', [nodeName + '_scale'],
                                                 [nodeName + '_relu'])

    inits = [dconv_w_tensor, dconv_b_tensor]
    if isNeedRelu:
        nodes = [deconv_node, deconv_scale_node, deconv_relu_node]
        output = nodeName + "_relu"
    else:
        nodes = [deconv_node, deconv_scale_node]
        output = nodeName + "_scale"

    return inits, nodes, output


def make_lstm(w_ifo_file, b_ifo_file, w_c_file, b_c_file, inputname, nodeName, scale_ifo=1024, scale_c=1024,
              shiftBits=[-7, -7]):
    outputName = nodeName + '_lstm5_out'

    lstm_w_ifo_tensor = make_tensor(w_ifo_file, nodeName + "_lstm_ifo_w", tranpose=True)
    lstm_b_ifo_tensor = make_tensor(b_ifo_file, nodeName + "_lstm_ifo_b")
    lstm_w_c_tensor = make_tensor(w_c_file, nodeName + "_lstm_c_w", tranpose=True)
    lstm_b_c_tensor = make_tensor(b_c_file, nodeName + "_lstm_c_b")

    act_input256 = [i for i in range(128)] + [i - 128 for i in range(128)]
    act_input1024 = [i for i in range(512)] + [i - 512 for i in range(512)]
    act_input256 = np.array(act_input256)
    act_input1024 = np.array(act_input1024)

    sigmoid_table_list = sigmoid_q(act_input1024, 6, 7)
    tanh1_table_list = tanh_q(act_input1024, 8, 7)
    tanh2_table_list = tanh_q(act_input256, 7, 7)

    act_table1 = onnx.helper.make_tensor("act_table1",
                                         data_type=onnx.TensorProto.FLOAT,
                                         dims=(1, 1024),
                                         vals=sigmoid_table_list.flatten())
    act_table2 = onnx.helper.make_tensor("act_table2",
                                         data_type=onnx.TensorProto.FLOAT,
                                         dims=(1, 1024),
                                         vals=tanh1_table_list.flatten())
    act_table3 = onnx.helper.make_tensor("act_table3",
                                         data_type=onnx.TensorProto.FLOAT,
                                         dims=(1, 256),
                                         vals=tanh2_table_list.flatten())

    lstm_node = onnx.helper.make_node(
        'Lstm',
        inputs=[
            inputname, nodeName + "_lstm_ifo_w", nodeName + "_lstm_c_w",
                       nodeName + "_lstm_ifo_b", nodeName + "_lstm_c_b"
        ],
        scale_ioft=scale_ifo,
        scale_ct=scale_c,
        activate_type=['sigmoid', 'tanh', 'tanh'],
        activate_table=[act_table1, act_table2, act_table3],
        shift_bits=shiftBits,
        outputs=[outputName],
        name=nodeName
    )

    node5 = [lstm_node]
    initializer = [
        lstm_w_ifo_tensor, lstm_b_ifo_tensor, lstm_w_c_tensor,
        lstm_b_c_tensor
    ]

    return initializer, node5, outputName


def make_batchnorm(bn_weight_file, bn_bias_file, bn_mean_file, bn_var_file, scale_in, scale_out, input_name, node_name,
                   use_relu=False):
    '''
    bn_weight_file, bn_bias_file, bn_mean_file, bn_var_file: 4个相关参数文件
    scale_in：输入数据缩放倍数
    scale_out：输出数据缩放倍数
    input_name：输入名称
    node_name：节点名称
    use_relu：是否做relu激活
    '''
    # x_in = x_in * scale_in
    # x_out = xout / scale_out
    bn_weight = make_tensor(bn_weight_file, node_name + "_w")
    bn_bias = make_tensor(bn_bias_file, node_name + "_b")
    bn_mean = make_tensor(bn_mean_file, node_name + "_mean")
    bn_var = make_tensor(bn_var_file, node_name + "_var")

    linear_bn_node = onnx.helper.make_node('BatchNormalization',
                                           inputs=[input_name, node_name + "_w", node_name + "_b",
                                                   node_name + "_mean", node_name + "_var"],
                                           outputs=[node_name + "_out"],
                                           epsilon=1e-5,
                                           scale_in=scale_in,
                                           scale_out=scale_out,
                                           name=node_name
                                           )
    relu_node = onnx.helper.make_node('Relu', inputs=[node_name + "_out"], outputs=[node_name + "_relu"])

    inits = [bn_weight, bn_bias, bn_mean, bn_var]

    output_name = None
    if use_relu == False:
        nodes = [linear_bn_node]
        output_name = node_name + "_out"
    else:
        nodes = [linear_bn_node, relu_node]
        output_name = node_name + "_relu"

    return inits, nodes, output_name


def make_GRU(gru_weight1_file, gru_weight2_file, gru_weight3_file, gru_bias1_file, gru_bias2_file, gru_bias3_file,
             inputName, nodeName, input_size, output_size):
    """
        生成GRU算子
    """
    gru_weight1 = make_tensor(gru_weight1_file, nodeName + "_weight1", dims=(input_size + output_size, 2 * output_size),
                              tranpose=True)
    gru_weight2 = make_tensor(gru_weight2_file, nodeName + "_weight2", dims=(output_size, output_size), tranpose=True)
    gru_weight3 = make_tensor(gru_weight3_file, nodeName + "_weight3", dims=(input_size, output_size), tranpose=True)
    gru_bias1 = make_tensor(gru_bias1_file, nodeName + "_bias1", dims=(2 * output_size,))
    gru_bias2 = make_tensor(gru_bias2_file, nodeName + "_bias2", dims=(output_size,))
    gru_bias3 = make_tensor(gru_bias3_file, nodeName + "_bias3", dims=(output_size,))
    output_name = nodeName + "_output"

    # 生成激活表，表长1024
    act_input1024 = [i for i in range(512)] + [i - 512 for i in range(512)]
    sigmoid_act_table = np.array(act_input1024)
    sigmoid_table_data = sigmoid_q(sigmoid_act_table, 5, 7)
    tanh_act_table = np.array(act_input1024)
    tanh_table_data = tanh_q(tanh_act_table, 7, 7)
    act_table = np.concatenate((sigmoid_table_data, tanh_table_data), axis=0)
    act_table = np.reshape(act_table, (2, 1024)).astype(np.float32)
    act_table = onnx.helper.make_tensor("act_table",
                                        data_type=onnx.TensorProto.FLOAT,
                                        dims=(2, 1024),
                                        vals=act_table.flatten())

    gru_node = onnx.helper.make_node('Gru',
                                     inputs=[inputName, nodeName + "_weight1", nodeName + "_weight2",
                                             nodeName + "_weight3",
                                             nodeName + "_bias1", nodeName + "_bias2", nodeName + "_bias3"],
                                     scale_zr=4096,
                                     scale_ht=1024,
                                     scale_in=1024,
                                     scale_ones=128,
                                     activate_type=['sigmoid', 'tanh'],
                                     activate_table=act_table,
                                     shift_bits=[-7, -7],
                                     clean_ht=0,
                                     outputs=[output_name],
                                     name=nodeName)

    inits = [gru_weight1, gru_weight2, gru_weight3, gru_bias1, gru_bias2, gru_bias3, act_table]
    nodes = [gru_node]

    return inits, nodes, output_name


def make_gru2array(gru_weight1_file, gru_weight2_file, gru_bias1_file, gru_bias2_file, input_size, output_size,
                   group_num, input_name, node_name):
    gru_weight1 = make_tensor(gru_weight1_file, node_name + "_weight1", dims=(input_size, output_size * 3),
                              tranpose=True)
    gru_weight2 = make_tensor(gru_weight2_file, node_name + "_weight2", dims=(output_size, output_size * 3),
                              tranpose=True)
    gru_bias1 = make_tensor(gru_bias1_file, node_name + "_bias1", dims=(output_size * 3,))
    gru_bias2 = make_tensor(gru_bias2_file, node_name + "_bias2", dims=(output_size * 3,))

    act_input1024 = [i for i in range(512)] + [i - 512 for i in range(512)]
    act_input256 = [i for i in range(128)] + [i - 128 for i in range(128)]
    # sigmoid_act_list = np.array(act_input256)
    # sigmoid_act_list = sigmoid_q(sigmoid_act_list, 5, 7)
    # np.savetxt('256_table_sigmoid.txt',sigmoid_act_list, fmt='%1d')
    sigmoid_act_table = np.array(act_input1024)
    sigmoid_act_table = torch.tensor(sigmoid_act_table)
    sigmoid_table_data = sigmoid_q(sigmoid_act_table, 6, 7)
    # np.savetxt('1024_table_sigmoid.txt',sigmoid_table_data, fmt='%1d')
    tanh_act_table = np.array(act_input1024)
    tanh_act_table = torch.tensor(tanh_act_table)
    tanh_table_data = tanh_q(tanh_act_table, 8, 7)
    act_table = np.concatenate((sigmoid_table_data, tanh_table_data), axis=0)
    act_table = np.reshape(act_table, (2, 1024)).astype(np.float32)
    act_table = onnx.helper.make_tensor("act_table",
                                        data_type=onnx.TensorProto.FLOAT,
                                        dims=(2, 1024),
                                        vals=act_table.flatten())

    gru_node = onnx.helper.make_node('Gru2Array',
                                     inputs=[input_name, node_name + "_weight1", node_name + "_weight2",
                                             node_name + "_bias1", node_name + "_bias2"],
                                     input_offset=0,
                                     hidden_shift_bit=0,  # 输入scle
                                     scale_input=1024,
                                     scale_hidden=1024,
                                     scale_ones=128,
                                     activate_type=['sigmoid', 'tanh'],
                                     activate_table=act_table,
                                     shift_bits=[-7, -7, 0, 0],
                                     clean_ht=0,
                                     outputs=[node_name + "_out"],
                                     groupNum=group_num,
                                     name=node_name)

    output_name = node_name + "_out"
    inits = [gru_weight1, gru_weight2, gru_bias1, gru_bias2, act_table]
    nodes = [gru_node]

    return inits, nodes, output_name


def make_gru2array_2100(gru_weight1_file, gru_weight2_file, gru_bias1_file, gru_bias2_file, input_size, output_size,
                        group_num, input_name, node_name):
    gru_weight1 = make_tensor(gru_weight1_file, node_name + "_weight1", dims=(input_size, output_size * 3),
                              tranpose=False)
    gru_weight2 = make_tensor(gru_weight2_file, node_name + "_weight2", dims=(output_size, output_size * 3),
                              tranpose=False)
    gru_bias1 = make_tensor(gru_bias1_file, node_name + "_bias1", dims=(output_size * 3,))
    gru_bias2 = make_tensor(gru_bias2_file, node_name + "_bias2", dims=(output_size * 3,))

    act_input1024 = [i for i in range(512)] + [i - 512 for i in range(512)]
    # act_input256 = [i for i in range(128)] + [i - 128 for i in range(128)]
    # sigmoid_act_list = np.array(act_input256)
    # sigmoid_act_list = sigmoid_q(sigmoid_act_list, 5, 7)
    # np.savetxt('256_table_sigmoid.txt',sigmoid_act_list, fmt='%1d')
    sigmoid_act_table = np.array(act_input1024)
    sigmoid_table_data = sigmoid_q(sigmoid_act_table, 6, 7)
    # np.savetxt('1024_table_sigmoid.txt',sigmoid_table_data, fmt='%1d')
    tanh_act_table = np.array(act_input1024)
    tanh_table_data = tanh_q(tanh_act_table, 8, 7)
    act_table = np.concatenate((sigmoid_table_data, tanh_table_data), axis=0)
    act_table = np.reshape(act_table, (2, 1024)).astype(np.float32)
    act_table = onnx.helper.make_tensor("act_table",
                                        data_type=onnx.TensorProto.FLOAT,
                                        dims=(2, 1024),
                                        vals=act_table.flatten())

    gru_node = onnx.helper.make_node('Gru',
                                     inputs=[input_name, node_name + "_weight1", node_name + "_weight2",
                                             node_name + "_bias1", node_name + "_bias2"],
                                     scale_zr=1024,
                                     scale_ht=1024,
                                     activate_type=['sigmoid', 'tanh'],
                                     activate_table=act_table,
                                     shift_bits=[-7, -7],
                                     outputs=[node_name + "_out"],
                                     name=node_name)

    output_name = node_name + "_out"
    inits = [gru_weight1, gru_weight2, gru_bias1, gru_bias2, act_table]
    nodes = [gru_node]

    return inits, nodes, output_name


def make_actlut(act_table, act_type, input_name, node_name):
    """
    act_table: 激活表
    act_type: 激活类型
    input_name: 输入名称
    node_name: 节点名称
    """
    act_table = make_tensor(act_table, node_name + "_table")
    act_node = onnx.helper.make_node('ActLut', [input_name], [node_name + "_out"],
                                     act_type=act_type,
                                     table_params=act_table,
                                     name=node_name
                                     )

    inits = [act_table]
    output_name = node_name + "_out"
    nodes = [act_node]

    return inits, nodes, output_name


def make_concat(input_name1, input_name2, node_name, axis=1, use_relu=False):
    """
    input_name1: 输入名称1
    input_name2: 输入名称2
    node_name: 节点名称
    axis: 维度
    use_relu: 是否使用relu激活
    """
    concat_node = onnx.helper.make_node("Concat",
                                        inputs=[input_name1, input_name2],
                                        outputs=[node_name + "_out"],
                                        axis=1,
                                        name=node_name
                                        )
    relu_node = onnx.helper.make_node('Relu', inputs=[node_name + "_out"], outputs=[node_name + "_relu"])

    inits = []
    output_name = None
    if use_relu == False:
        nodes = [concat_node]
        output_name = node_name + "_out"
    else:
        nodes = [concat_node, relu_node]
        output_name = node_name + "_relu"
    return inits, nodes, output_name


def make_maxpool(kernal, stride, pad, input_name, node_name):
    """
    kernel: 卷积核
    pad: padding
    stride: 步长
    scale_in: 输入量化参数
    scale_out: 输出量化参数
    input_name: 输入名称
    node_name: 节点名称
    use_relu: 是否使用relu激活
    """
    maxpool_node = onnx.helper.make_node("MaxPool",
                                         inputs=[input_name],
                                         outputs=[node_name + "_out"],
                                         kernel_shape=kernal,
                                         pads=pad,
                                         strides=stride,
                                         name=node_name
                                         )
    output_name = node_name + "_out"
    inits = []
    nodes = [maxpool_node]

    return inits, nodes, output_name
