# Limo robot Simulation Project (ROS Noetic)

## Steps to deploy using Docker (Windows OS)

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
    _______________________________________
    

2. Install Docker Desktop App

    - [Direct download link for x86_64 systems](https://desktop.docker.com/win/main/amd64/Docker%20Desktop%20Installer.exe?utm_source=docker&utm_medium=webreferral&utm_campaign=docs-driven-download-win-amd64)
    - Run the installer, it will guide you through the process.
    - Ensure you leave the "Use WSL 2 instead of Hyper-V" option checked.
    - Follow the installation prompts and restart your computer one more time.
