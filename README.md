# WOA-MPC Lane-Centering Controller Optimization

This repository contains a MATLAB/Simulink workflow for tuning a lane-centering controller using the **Whale Optimization Algorithm (WOA)**. The optimization loop tests different controller gains and MPC weights by running the Simulink driving scenario, evaluating each candidate controller, and saving the best-performing parameter set.

The general goal is to optimize controller parameters for a torque-based lane-centering control system.

---

## Repository Structure

```text
WOA_MPC/
│
├── DrivingScenario.slx              # Main Simulink model used for optimization
├── Plant_RRSim.slx                  # Vehicle/plant model
├── busDefinitions.mat               # Simulink bus definitions
├── WOA_MPC.prj                      # Main project file to run MATLAB project
│
├── LCC/
│   ├── LCC_TorqueControl_2025b.slx
│   ├── LCC_TrajectoryPlanning_2025b.slx
│   └── LCC_pathprediction_2025b.slx
|
├── Scripts/
│   ├── LoadValues.m                   # Loads saved best values to workspace
│   ├── LoadDrivingScenario.m          # Loads bus definitions and scenario setup variables
│   └── RunWOA.m                       # Main script used to run WOA-based controller tuning
|
├── DrivingScenarios/
│   ├── OvalTrack.mat                  # Large oval track road scenario
│   └── leftCurve.mat                  # Simple left curve road scenario
│
├── WOA/
│   ├── WOA.m                        # General WOA implementation
│   ├── WOA_simulink.m               # WOA implementation for Simulink-based cost evaluation
│   └── initialization.m             # Search-agent initialization function
│
└── Results/
    ├── best_whale_lookAhead10.mat   # Saved optimized result
    └── best_whale_lookAhead3.mat    # Saved optimized result

```

---

## Requirements

This project is intended to be run in MATLAB with Simulink.

Recommended software:

- MATLAB R2025b or newer
- Simulink
- Parallel Computing Toolbox, optional but recommended
- Vehicle Dynamics Blockset

---

## Opening the Project

### 1. Clone the repository

From a terminal or Git Bash window, run:

```bash
git clone https://github.com/ballsc/WOA_MPC.git
```

Then move into the repository folder:

```bash
cd WOA_MPC
```

---

### 2. Open MATLAB in the repository folder

Start MATLAB and set the current folder to the cloned repository location.

In MATLAB, this can be done with:

```matlab
cd('path/to/WOA_MPC')
```

For example:

```matlab
cd('C:/Users/YourName/Documents/MATLAB/WOA_MPC')
```

---

### 3. Open Project Folder

Double click WOA_MPC.prj to open the MATLAB project. Verify this adds all main folders to the project path. If not added, right click on each main folder and hit the option to add to project path.

---

### 4. Load the driving scenario setup

Run:

```matlab
LoadDrivingScenario
```

This script loads the required setup variables, including bus definitions and simulation parameters.

---

### 5. Open the main Simulink model

Open the main simulation model with:

```matlab
open_system('DrivingScenario.slx')
```

Alternatively, double-click `DrivingScenario.slx` from the MATLAB Current Folder browser.

This allows you to view the main model being optimized using WOA.

---

## Running the WOA Optimization

The optimization is best run with the simulink model closed for performance purposes.

The main script for running the controller-weight optimization is:

```matlab
RunWOA.m
```

Run it from the MATLAB Command Window:

```matlab
RunWOA
```

This script defines the WOA search settings and the tunable controller variables.

The optimized variables are:

| Variable | Description |
|---|---|
| `Kp` | Proportional gain |
| `Ki` | Integral gain |
| `Qy` | MPC lateral-error weight |
| `Qyaw` | MPC yaw-error weight |
| `Rang` | Steering-angle magnitude penalty |
| `Sang` | Steering-angle-rate penalty |

The default search bounds in `MPC_PID_sweep.m` are:

```matlab
lb = [0 0 0 0 1 1];
ub = [4 3 5 5 15 15];
```

The default WOA settings are:

```matlab
Max_iteration = 100;
SearchAgents_no = 8;
```

---

## Using Parallel Simulation

The project is structured to support parallel Simulink runs using `parsim`.

Parallel execution is useful because each whale/search agent requires a separate Simulink simulation. Running these simulations in parallel can significantly reduce optimization time.

By default, a variable in RunWOA called parflag is set to 1. This enables parallel execution.

If parallel execution causes issues, close the parallel pool:

```matlab
delete(gcp('nocreate'))
```

Then modify the RunWOA script to unset parflag.

---

## How the Optimization Works

The optimization process follows these steps:

1. Initialize a population of whales, where each whale represents one candidate set of controller parameters.
2. Assign each whale's parameter values to the Simulink model using `Simulink.SimulationInput`.
3. Run the driving scenario simulation for each whale.
4. Evaluate the performance of each whale using the simulation output.
5. Update the leader whale according to the Whale Optimization Algorithm.
6. Repeat until the maximum number of iterations is reached.
7. Return the best controller parameters and convergence history.

The Simulink-specific WOA implementation is contained in:

```matlab
WOA/WOA_simulink.m
```

The general WOA implementation is contained in:

```matlab
WOA/WOA.m
```

---

## Outputs

After the optimization finishes, the script produces:

- The best controller parameter set found by WOA
- The best fitness value
- A convergence curve showing optimization progress over iterations
- Optional saved `.mat` result files

Saved examples are located in the `Results/` folder.

Example result files include:

```text
Results/best_whale_lookAhead10.mat
Results/best_whale_lookAhead3.mat
```
To save resutls, simply select the run results from the workspace, and export to a .mat file.

---


## Modifying the Optimization

### Change the number of whales

In `MPC_PID_sweep.m`, edit:

```matlab
SearchAgents_no = 8;
```

A larger number of whales gives broader search coverage but requires more simulations per iteration.

---

### Change the number of iterations

Edit:

```matlab
Max_iteration = 100;
```

More iterations give the optimizer more time to converge but increase total runtime.

---

### Change the tuned variables

The tuned variable names are stored in:

```matlab
var_list = ["Kp", "Ki", "Qy", "Qyaw", "Rang", "Sang"];
```

To tune different variables, update `var_list`, `lb`, and `ub` together.

For example, if adding another variable, all three arrays must be expanded consistently:

```matlab
var_list = ["Kp", "Ki", "Qy", "Qyaw", "Rang", "Sang", "NewVariable"];
lb = [0 0 0 0 1 1 0];
ub = [4 3 5 5 15 15 10];
```

The corresponding Simulink block parameters should use these variable names rather than hard-coded numeric values.

---

## Important Notes

- The Simulink model must be able to access all variables listed in `var_list`.
- If a Constant block is being tuned, its value should be set to the variable name, such as `Kp`, rather than a fixed number.
- The variable names in `var_list` must match the variable names used inside the Simulink model.
- The repository folder and subfolders should be on the MATLAB path before running the optimization.
- If a referenced model cannot be found, verify that `addpath(genpath(pwd))` has been run from the repository root.
- If bus-related errors occur, run `LoadDrivingScenario.m` before opening or simulating the model.

---

## Troubleshooting

### MATLAB cannot find a function or model

Run:

```matlab
addpath(genpath(pwd))
savepath
```

Then try again.

---

### Simulink reports undefined variables

Make sure `LoadDrivingScenario.m` has been run:

```matlab
LoadDrivingScenario
```

Also verify that all tuned variables in `var_list` are either initialized in the workspace or assigned through `Simulink.SimulationInput`.

---

### `parsim` fails to run

Try starting a parallel pool manually:

```matlab
parpool
```

If errors continue, test the model with a single manual simulation first:

```matlab
sim('DrivingScenario')
```

This helps determine whether the issue is with the model itself or with parallel execution.

---

### SimulationInput variables do not affect the model

Check that the relevant Simulink blocks use workspace variables as their values.

For example, a Constant block should have:

```matlab
Kp
```

not:

```matlab
1.5
```

`setVariable` can only change the value used by the model if the model references that variable.

---

## Citation

The Whale Optimization Algorithm implementation is based on the original WOA method by Seyedali Mirjalili and Andrew Lewis:

> Mirjalili, S., & Lewis, A. (2016). The Whale Optimization Algorithm. Advances in Engineering Software, 95, 51–67.

---

