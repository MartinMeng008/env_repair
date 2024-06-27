# env_relax_repair

## Overview

This repository provides the code associated with the paper [Automated Robot Recovery from Assumption Violations of High-level Specifications]()

env_relax_repair is the first framework that enables robot to automatically recover from environment assumption violations of high-level specifications during task execution. We create a monitor to detect assumption violations during the task execution, relax the violated assumptions to admit observed environment behaviors, and obtain new robot skills for task completion.

<!-- ![overview](workflow.png) -->
<img src=workflow.png alt="overview" width="700">


## Setup

This code requires python > 3 and has been tested on python 3.8.10 and Ubuntu 20.04

### Required repositories

- This [fork](https://github.com/MartinMeng008/synthesis_based_repair) of synthesis_based_repair and its dependencies should be installed and added to the PATH:
  - [BDD package](https://github.com/tulip-control/dd)

  ```shell
  git clone -b env_relax_repair https://github.com/MartinMeng008/synthesis_based_repair.git
  export PATH=[parent_dir_of_synthesis_based_repair]/synthesis_based_repair/synthesis_based_repair
  git clone https://github.com/tulip-control/dd.git
  cd dd
  pip install cython
  python setup.py install --fetch --cudd
  ```

- This [fork](https://github.com/MartinMeng008/slugs) of slugs. The path to the src and parser directory should be added to PATH.

  ```shell
  git clone https://github.com/MartinMeng008/slugs.git
  export PATH=[parent_dir_of_slugs]/slugs/src:$PATH
  export PATH=[parent_dir_of_slugs]/slugs/tools/StructuredSlugsParser:$PATH
  ```
- ROS. This code has been tested on ROS Noetic. Follow the installation [instructions](http://wiki.ros.org/ROS/Installation).
- This [repo](https://github.com/MartinMeng008/cs4750) containing helper functions should be installed in ROS catkin_ws directory:

   ```shell
  cd [parent_dir_of_catkin]/catkin_ws/src
  git clone https://github.com/MartinMeng008/cs4750.git
  ```

- Hello Robot Stretch ROS [package](https://github.com/hello-robot/stretch_ros):

   ```shell
  cd [parent_dir_of_catkin]/catkin_ws/src
  git clone https://github.com/hello-robot/stretch_ros.git
   ```

### Required python packages
- argparse
- matplotlib
- networkx
- numpy
- shapley
- z3

```shell
pip install argparse matplotlib networkx numpy shapley z3
```

### Installation
Clone the repository to the catkin_ws directory:

```shell
cd [parent_dir_of_catkin]/catkin_ws/src
git clone https://github.com/MartinMeng008/env_relax_repair.git
cd ..
catkin build
source devel/setup.bash
```

## Usage

### Inputs

#### Workspace

#### 


## Reference
If you use this repository or find this project interesting, please kindly cite:

```bib
Todo
```
