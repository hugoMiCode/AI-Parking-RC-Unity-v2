import sys
import os
import time
import numpy as np
import onnx
import onnxruntime
from onnx import numpy_helper

# This script is used to print the inputs and outputs of the model
# It is useful to know the shape of the inputs and outputs of the model
# For more information on the model, you can use Netron (https://netron.app/)
# But this script is useful to know how to use .onnx model in python


model_dir = "./model"
model = model_dir+"/Park90RC.onnx"

session = onnxruntime.InferenceSession(model, None)


input_details = session.get_inputs()
output_details = session.get_outputs()
input_feed = {}


print("Inputs:")
for input_detail in input_details:
    input_name = input_detail.name
    input_shape = input_detail.shape
    print(f"    Name: {input_name}")
    print(f"    Shape: {input_shape}")
    
    input_shape = [dim if isinstance(dim, int) else 1 for dim in input_shape]
    input_data = np.zeros(input_shape, dtype=np.float32)
    
    input_feed[input_name] = input_data

print("\r\nOutputs:")
for output_detail in output_details:
    output_name = output_detail.name
    output_shape = output_detail.shape
    print(f"    Name: {output_name}")
    print(f"    Shape: {output_shape}")

print("\r\n")


print("Running inference with 0s as input:")
for output_detail in output_details:
    output_name = output_detail.name
    output_shape = output_detail.shape
    result = session.run([output_name], input_feed)
    print(f"    {output_name}: {result}")


# For the inference, you need to use deterministic_continuous_actions
