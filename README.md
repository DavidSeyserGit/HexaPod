# **Hexapod Project**

This repository contains the code and documentation for our university (FHTW) project to control a hexapod robot using the Robot Operating System (ROS).

## **Table of Contents**

- [About](#about)
- [Installation](#installation)
- [Features](#features)
- [License](#license)

## **About**

Our project aims to revive a hexapod robot using ROS, enabling it to perform simple forwards movment. This README provides an overview of the project structure, setup instructions, and guidelines for usage.

## **Installation**

### Cloning the Repository
To clone the repository and its submodules, use the following command:

    git clone --recurse-submodules https://github.com/DavidSeyserGit/HexaPod.git

### Setting Up the Submodule
After the repository is cloned, the submodules need to be initialized:

    cd HexaPod
    git submodule init
    git submodule update

### compiling the code with catkin_make

After the repository is successfully cloned and the submodules initialized, the devel and build folder must be removed.
If you are on Linux use the following commands:

    rm -rf devel && rm -rf build

The repository is now ready to be compiled.
    
    catkin_make
    
## **Features**

- **ROS Integration**: Utilizing ROS for communication, control, and coordination of the hexapod robot.
