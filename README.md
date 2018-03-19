# cartographer_hokuyo3d

## Package Summary

The package provides a quick demonstration of 3-D SLAM using hokuyo3d (driver node for Hokuyo YVT series) and Google Cartographer.

## Build and Run

Running in docker container is recommended to avoid linux installation related problems.
This requires Docker and docker-compose installed. (see https://docs.docker.com/install/ and https://docs.docker.com/compose/install/)

Provided docker-compose is for Intel HD and Radeon graphic boards. NVIDIA requires nvidia-docker and nvidia-docker-compose installed and some changes on docker-compose.yml.

1. Build the docker image
    ```
    $ docker-compose build
    ```
2. Connect YVT sensor having the IP address of 192.168.0.10 (sensor default)
3. Run
    ```
    $ docker-compose up
    ```
    
Resultant data including bag file, trajectory node pbstream, and pointcloud in pcd format will be exported to your `~/.slam-results`. The exported files will be overwritten by running it.
