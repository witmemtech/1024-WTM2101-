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

import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
import tqdm
import os
from rnnoise import RNNoise


# import h5py


class RNNoiseDataset(torch.utils.data.Dataset):
    def __init__(self,
                 features_file,
                 sequence_length=200):
        self.sequence_length = sequence_length

        # print('Loading data...')
        # with h5py.File(features_file, 'r') as hf:
        #     self.data = hf['data'][:]
        # print('done.')

        self.data = np.memmap(features_file, dtype='float32', mode='r')
        # dim = 98
        # dim = 87
        dim = 65
        # tmp_shape = self.data.shape[0]
        # print(tmp_shape)
        #
        # # self.data = self.data.reshape((50, 200, 87))
        #
        # self.nb_sequences = self.data.shape[0] // self.sequence_length
        # # self.data = self.data[:self.nb_sequences * self.sequence_length * dim]
        # # #
        # # self.data = np.reshape(self.data, (self.nb_sequences, self.sequence_length, dim))
        self.nb_sequences = self.data.shape[0] // self.sequence_length // dim
        self.data = self.data[:self.nb_sequences * self.sequence_length * dim]

        self.data = np.reshape(self.data, (self.nb_sequences, self.sequence_length, dim))
        self.data_tmp = self.data

    def __len__(self):
        return self.nb_sequences

    def __getitem__(self, index):
        # return self.data[index, :, :65].copy(), self.data[index, :, 65:-1].copy(), self.data[index, :, -1:].copy()
        # return self.data[index, :, :42].copy(), self.data[index, :, 42:64].copy(), self.data[index, :, -1:].copy()
        outdata = self.data[index, :, :42].copy(), self.data[index, :, 42:64].copy(), self.data[index, :, -1:].copy()
        return outdata


device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

dataset_data = RNNoiseDataset("./data/training.f32")
# from torch.utils.data import DataLoader
dataloader = torch.utils.data.DataLoader(dataset=dataset_data, batch_size=16, shuffle=True, num_workers=1)


if __name__ == '__main__':
    net_path = './output/checkpoints/denoise_50.pth'

    model = RNNoise()
    model.to(device)
    model.load_state_dict(torch.load(net_path, map_location='cpu'))

    # =====================model val=====================
    model.eval()

    for epoch in range(1, 1 + 1):

        print(f"testing epoch {epoch}...")

        with tqdm.tqdm(dataloader, unit='batch') as tepoch:
            for i, (features, gain, vad) in enumerate(tepoch):
                pred_gain = model(features)
                scale = 4
                features_tmp=features * scale
                value = (features_tmp).round().clip(min=-128, max=127)
                pred_gain_quant_Q = model.forward_quant(value)

                gainQ = pred_gain_quant_Q.reshape(-1)

                cos_sim = torch.cosine_similarity(pred_gain, pred_gain_quant_Q, dim=1)
                print("###########################cosine_similarity################################")
                print("{:.5f}".format(cos_sim.mean().detach().cpu().item()))

