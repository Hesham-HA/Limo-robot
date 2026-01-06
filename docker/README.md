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


## Steps to deploy in WSL (Windows OS) 

1. Install WSL 2 (Windows Subsystem for Linux)

    `* Note: This only works for Windows 10 Pro/Enterprise/Home version 19044 or above`
    
    - Open Windows Power Shell as administrator
    - Run the following command:
        ```powershell
        wsl --install
        ```
    - This command will automatically download and install the latest version of WSL, the Linux kernel, and the Ubuntu distribution for you.
    - Restart your computer when prompted.

    ____________________________________
    *For older Windows versions, use manual installation instead:*
    - *Enable Virtual machine platform:*
        - *Open PowerShell as Administrator and run this command:*
            ```powershell
            dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
            ```
        - *Restart your computer for updates to take place.*
    - *Install the Linux Kernel Update:*
        - [*Direct download link for x86_64 systems*](https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi)
        - *Run the installer*
        - *Set WSL 2 as your default version:*
            ```powershell
            wsl --set-default-version 2
            ```
    - *Install the Linux kernel of your choice from Windows App Store by searching its name.*
    _________________________________________