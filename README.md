# Limo robot Simulation Project (ROS Noetic)

## Steps to deploy using Docker (Linux OS)

1. Install docker

    Follow steps from: [Docker installation page](https://docs.docker.com/engine/install/)

2. Install nvidia container toolkit (optional: if you want to use GPU acceleration)

    Follow steps from: [Nvidia Container toolkit installer](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
    
    `* Note: you have to get Nvidia drivers installed first`

3. Build Docker image

    Run from inside the Repo's home directory:
    ```bash
    docker compose build 
    ```
    `* Check: docker image is created: `
    ```bash
    docker image list
    ```

4. Run Docker container for your system

    Run from inside the home directory::
    ```bash
    docker compose up -d
    ```
    `* Check: docker container is running:`
    ```bash
    docker ps
    ```

5. Access your docker container from any terminal

    Run the following to enter inside the docker container's bash:
    ```bash
    docker exec -it noetic bash
    ```
    `* Note: to exit from container, simply type:` `exit`