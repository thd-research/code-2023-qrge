# code-2023-qrge


# Here is the RaiSim deployment algorithm in docker

## 1. Obtaining a License:

You need to obtain a RaiSim license, which is tied to the hardware ID of the machine.

Please go to [LINK](https://docs.google.com/forms/d/e/1FAIpQLSc1FjnRj4BV9xSTgrrRH-GMDsio_Um4DmD0Yt12MLNAFKm12Q/viewform), fill in all the fields, and select the Academic license.

Rename the obtained license file to **_activation.raisim_** and place it in `/home/<YOUR-USERNAME>/.raisim`

The original tutorial for installation can be found [HERE](https://raisim.com/sections/Installation.html)


## 2. Clone the Repository:

Clone the repository with Docker source code to `/home/<YOUR-USERNAME>/`
```
cd /home/$USER && git clone git@github.com:thd-research/code-2023-study-quadruped-gait.git
```
## 3. Build Docker:

In the repository, there is a script for installing Docker itself:  _**install_docker.bash**_

Run the Docker installation script:
```
bash install_docker.bash -n
```
Run the script to clone repositories and make changes in them (**** Here, the license key is cloned into Docker):
```
bash initial.bash
```
Run the Docker build script:
```
bash build_docker.sh -n
```
And run the Docker launch script:
```
bash run_docker.sh -n
```
# Further actions are performed exclusively in Docker


## 4. Project Build:

In Docker, in the directory **_raisim_workspace_** , you will find the scripts **_build.bash_** and _**rebuild.bash**_. 

For the **_initial_** project build, run the **_build.bash_**script, which moves the license key to the necessary directory and creates the required build and project build folders.

For subsequent builds, run the **_rebuild.bash_** script.

## 5. Working with the Project:

The main working directories in Docker are:

```
/raisim_workspace/raisimLib/raisimGymTorch/data - Location of trained models
/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_anymal - Location of executable files
```

### 5.1 Running Training


Running the working code is done in the directory  `/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_anymal`.

To start training, execute:
```
cd /raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_anymal
python3 runner.py
```
In this directory, the main file for making changes and configuring the simulation is **_Environment.hpp_**.

The _**cfg.yaml**_ file sets punishment and reward coefficients.

### 5.2 Policy Retraining

To start policy retraining, execute:
```
cd /raisim_workspace/raisimLib/raisimGymTorch/
python3 raisimGymTorch/env/envs/rsg_anymal/runner.py --mode retrain --weight data/uni_locomotion/***/full_**.pt
```

### 5.3 Running a Trained Policy

To run a pre-trained policy, execute:
```
cd /raisim_workspace/raisimLib/raisimGymTorch/
python3 raisimGymTorch/env/envs/rsg_anymal/tester.py --weight data/uni_locomotion/***/full_**.pt
```