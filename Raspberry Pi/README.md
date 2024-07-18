# REAL CAR SCRIPTS

This project uses a combination of Unity ML-Agents, ONNX models, and LIDAR data to train and control an autonomous parking system for a Picarx robot.

## Project Overview

### Main Script: `run_model.py`

This script handles the main functionality of reading LIDAR data, processing it, and controlling the Picarx robot based on predictions from an ONNX model.

#### Key Components

- **LIDAR Data Reading**: Reads data from a LIDAR sensor and processes it into a usable format.
- **Model Inference**: Uses an ONNX model to predict actions based on LIDAR data.
- **Robot Control**: Controls the Picarx robot's steering and throttle based on model predictions.

## Additional Scripts

### LIDAR Data Visualization: `read_lidar_fast.py`

This script reads LIDAR data and plots it on a polar graph. It helps visualize the LIDAR readings and ensures that the data processing is consistent with the main script.


This script will continuously read LIDAR data and periodically save polar plot images.

### Model Inputs and Outputs: `print_model_io.py`

This script prints the input and output details of the ONNX model. It is useful for understanding the model's expected input shapes and output formats. It also demonstrates how to use the ONNX model in Python for inference.

This script will print the names and shapes of the model's inputs and outputs, and perform a sample inference using zero inputs.

## Model Analysis with Netron

Netron is a tool for visualizing neural network models. It supports ONNX models and provides a graphical interface to inspect the layers and structure of the model.

### Using Netron

1. Visit [Netron](https://netron.app/).
2. Upload the ONNX model (`Park90RC.onnx`) located in the `./model` directory.
3. Explore the model structure, inputs, and outputs.

## Conclusion

This project integrates various technologies to create an autonomous parking system for a Picarx robot. By understanding the code and using the provided scripts, you can further develop and optimize the system for different scenarios and environments. For detailed model analysis, tools like Netron are invaluable.