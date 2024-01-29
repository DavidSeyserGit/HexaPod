# Hexapod Project

This repository contains the code and documentation for our university (FHTW) project to control a hexapod robot using the Robot Operating System (ROS).

## About


## Installation

### Cloning the Repository
To clone the repository and its submodules, use the following command:

    git clone --recurse-submodules https://github.com/DavidSeyserGit/HexaPod.git

### Setting Up the Submodule
After the repository is cloned, the submodules need to be initialized:

    git submodule init
    git submodule update

### compiling the code with catkin_make

After the repository is successfully cloned and the submodules initialized, the devel and build folder must be removed.
If you are on Linux use the following commands:

    cd Hexpaod 
    rm -rf devel && rm -rf build

The repository is now ready to be compiled.
    
    catkin_make

## Features

## Voraussetzungen

Um das Projekt auszuführen, sind folgende Voraussetzungen erforderlich:

- ROS1 Noetic

## Verwendung

## Lizenz

Dieses Projekt ist unter der [Lizenz] lizenziert - siehe die [LICENSE.md](LICENSE.md)-Datei für weitere Details.

