# cartographer_hokuyo3d

## Package Summary

The package provides a quick demonstration of 3-D SLAM using hokuyo3d (driver node for Hokuyo YVT series) and Google Cartographer.

## Run the demo

Running in docker container is recommended to avoid linux installation related problems.
This requires Docker and docker-compose installed. (see https://docs.docker.com/install/ and https://docs.docker.com/compose/install/)

Provided docker-compose is for Intel HD and Radeon graphic boards. NVIDIA requires nvidia-docker and nvidia-docker-compose installed and some changes on docker-compose.yml.

1. Connect YVT sensor having the IP address of 192.168.0.10 (sensor default)
2. Run
    ```console
    $ xhost +  # Allow showing rviz from docker container (needed to run for each login)
    $ docker-compose up
    ```

Keep in mind that it will download about 1GB of the docker image at the first time of the run.
To update docker image, do:
```console
$ docker pull atwat/cartographer_hokuyo3d
```

Resultant data including bag file, trajectory node pbstream, and pointcloud in pcd format will be exported to your `~/.slam-results`. The exported files will be overwritten by running it.

## Build the image from source

```console
$ docker build -t atwat/cartographer_hokuyo3d:latest .
```

## Apply local changes

To apply manually updated parameters into the docker image, add following two lines to the `volumes:` section in the docker-compose.yaml.
Now, your local change is bound to the container.

```yaml
      - "${PWD}/configuration_files:/opt/ros/kinetic/share/cartographer_hokuyo3d/configuration_files"
      - "${PWD}/launch:/opt/ros/kinetic/share/cartographer_hokuyo3d/launch"
```
