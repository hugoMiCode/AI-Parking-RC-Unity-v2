# AI-Parking-RC-Unity

This project utilizes Unity's ML-Agents toolkit to train AI models for parking tasks in a simulated environment. The training configurations are specified in the `RC.yaml` file for two environments: `Park90RC` and `ParkParallelRC`.

## Getting Started

### Training the Model

To train the AI model, use the following command:

```bash
mlagents-learn Config/RC.yaml --run-id=run1 --initialize-from=run0 --resume --force
```

Here's a breakdown of the flags used:

- `--run-id=<run-id>`: Unique identifier for the training run. Replace `<run-id>` with a unique name for each training session.
- `--initialize-from=<run-id>`: Initializes the model from a previous run. Replace `<run-id>` with the identifier of the previous run (optional).
- `--resume`: Resumes training from the last checkpoint (optional).
- `--force`: Overwrites the previous run data if it exists (optional).

### Understanding the Configuration (RC.yaml)

The `RC.yaml` file contains the configuration for the training environments. Below is a brief explanation of the key components:

#### General Structure

```yaml
behaviors:
  Park90RC:
    trainer_type: ppo
    hyperparameters:
      batch_size: 1024
      buffer_size: 5120
      learning_rate: 0.0003
      beta: 0.0025
      epsilon: 0.3
      lambd: 0.95
      num_epoch: 5
      learning_rate_schedule: linear

    network_settings:
      normalize: true
      hidden_units: 128
      num_layers: 2

    reward_signals:
      extrinsic:
        gamma: 0.99
        strength: 0.8

    keep_checkpoints: 15
    checkpoint_interval: 500000
    time_horizon: 264
    max_steps: 50000000
    summary_freq: 20000
    threaded: true

  ParkParallelRC:
    # Similar structure with slight variations
```

#### Explanation of Components

- `trainer_type`: Specifies the type of trainer. Here, `ppo` (Proximal Policy Optimization) is used.
- `hyperparameters`: Contains parameters like `batch_size`, `buffer_size`, `learning_rate`, etc., which control the training process.
- `network_settings`: Defines the neural network architecture, including `normalize`, `hidden_units`, and `num_layers`.
- `reward_signals`: Specifies the type and parameters of reward signals. `extrinsic` is used here with `gamma` and `strength`.
- `keep_checkpoints`: Number of checkpoints to keep.
- `checkpoint_interval`: Interval for saving checkpoints.
- `time_horizon`: Defines the time horizon for the agent's experiences.
- `max_steps`: Maximum number of steps for training.
- `summary_freq`: Frequency of writing training summaries.
- `threaded`: Enables multi-threading for training.

### GAIL (Generative Adversarial Imitation Learning)

GAIL is a training method that enables the agent to learn from demonstrations. In the configuration file, GAIL can be activated by uncommenting the respective section:

```yaml
reward_signals:
  extrinsic:
    gamma: 0.99
    strength: 0.8
  gail:
    strength: 0.3
    demo_path: Demos/DemoRC.demo
    use_actions: true

# behavioral_cloning:
#   demo_path: Demos/DemoRC.demo
#   steps: 750000
#   strength: 0.3
```

#### Explanation

- `gail`: The `gail` block within `reward_signals` enables GAIL.
  - `strength`: Determines the influence of the GAIL signal.
  - `demo_path`: Path to the demonstration file.
  - `use_actions`: If set to `true`, the agent uses actions from the demonstration.

To use GAIL, ensure the demonstration file exists at the specified path and contains the necessary data for the agent to learn from.

## Conclusion

This README provides an overview of setting up and running the AI-Parking-RC-Unity project. By following the instructions and understanding the configuration, you can train and optimize AI models for parking tasks in Unity. For more detailed information, refer to the [official ML-Agents documentation](https://github.com/Unity-Technologies/ml-agents/tree/main/docs).

---

Feel free to customize the README further based on specific needs or additional details relevant to your project.