# UMI RTX Robotic Arm Demos

## Background
This is a bare bone keyboard controller for the UMI RTX Robotic Arm (Yumi). It uses the driver provided by [Physar](https://github.com/physar/umi-rtx), and two nodes from [GARDE & MASSA](https://github.com/gardegu/LAB42_RTX_control), namely the nodeArm and nodeInverseKinematics nodes.

I created this repo with modifications to GARDE & MASSA's code as their project requires the use of a Realsense camera and a GPU. The start_arm\_control.sh script runs the driver, the nodes that interact with the driver, and the controller node which interprets the keyboard inputs and publishes the commands to the /target_grip and /target_pose topics.

Please note, this project is not meant to provide a polished controller for the robotic arm. It is meant to allow others to install the minimum packages neccesary to get their robitic arm up and moving. It is provided only so that others (including myself) may build upon it. Hopefully the controllers available will give you a few idea of how you wish to control the arm.

## Installation

To use this porject, you will need to use Docker. To learn more about Docker, what it is, and why it's useful, see this [video](https://youtu.be/DQdB7wFEygo).

### Docker Installation

To install Docker follow the steps on their [website](https://docs.docker.com/engine/install/).

For installing using the `apt` repository on Ubuntu do the following:

1. Set up Docker's `apt` repository.
```
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

2. Install the Docker packages.
```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

3. Verify that the installation is successful by running the `hello-world` image:
```
sudo docker run hello-world
```

This command downloads a test image and runs it in a container. When the container runs, it prints a confirmation message and exits.

You have now successfully installed and started Docker Engine.

### Run using prebuilt Docker container

If you wish to use the arm without ---------------------

However, this is not recommended as this project is meant to provide a foundation on top of which you build your own means of control. To iterate on and change the code in this project you'll need to build and run your own containers which is explained in the next section.

### Build and run custom Docker containers

This is the recommended means of working with this project.
The usefulness of building and running your own containers is that you can add your own workspace, and packages.

1. Build your image
To build your Docker image, you need a Dockerfile, which contains the instructions for Docker. In the same directory as your `Dockerfile`, run:
```bash
docker build -t my-image-name:latest .
```

`.` means "build context is the current directory" so if you really want, you can replace this with the location to the aformentioned directory.

2. Check your image
To list your local images, from any directory, run:
```bash
docker images
```

You should see `my-image-name` in the list.

3. Run a container from your image
To run a container from your image, from any directory, run:
```bash
docker run -it my-image-name:latest
```

You may need to run it with elevated permissions as I did:
```bash
sudo docker run -it my-image-name:latest --name yumi
```

When working on your own pachages, you may wish to make your own repository and change the following line in the `Dockerfile` to clone from your repository.
`RUN git clone https://github.com/gardegu/LAB42_RTX_control.git`
Or you may wish to do as I did during development, which was to [mount](https://docs.docker.com/engine/storage/bind-mounts/) your local directory as follows, and then create a repository when you're finished developing.
```bash
sudo docker run -it \
    --name yumi \
    -v ~/Desktop/JUNO/LAB42_RTX_control/:/home/Stage/LAB42_RTX_control/ \
    my-image-name
```

3. Check your container
You should now be able to see a list of running containers by running:
```bash
sudo docker ps
```
and a list of all running and stopped containers by running:
```bash
sudo docker ps -a
```

0. Notes on containers/useful commands
When containers are stopped (by entering `exit` in the container terminal), any changes made in the container persist unless you remove the container by running:
```bash
sudo docker rm yumi
```
Said changes do not transfer to any new containers unless you use volumes or mounts, in which case the data persists independently of the container lifesycle.

You can restart a container with all of it's local changes by running:
```bash
sudo docker start <container_id>
```
This does not "put you in" the terminal, this simply runs the container in the background (detached mode). To get inside the container in interactive mode run
```bash
sudo docker exec -it <container_id> /bin/bash
```
or
```bash
sudo docker exec -it <container_name> /bin/bash
```

## Running 

### Check that the UMI RTX Robotic Arm is connected

As it sands, the 


## Notes on the UMI RTX Driver

The driver 













## Installation

This project is built and tested with **Ubuntu 22.04** and **ROS2 Iron**.

1. **ROS2 Installation**

    First, ensure you have ROS2 Iron installed. If not, follow the instructions provided [here](https://docs.ros.org/en/iron/Installation.html) for installation. 
    After installation, set up your environment by sourcing ROS2. If you're not using bash, adjust accordingly by replacing ".bash" with your shell type in the following command:
    
    ```bash
    source /opt/ros/iron/setup.bash
    ```
    To avoid sourcing it every time, consider adding this line to the end of your `~/.bashrc` file.

2. **Install Dependencies**

    Start by updating your package lists:
    ```bash
    sudo apt update
    ```
   Then, install the necessary dependencies including Pinocchio for inverse kinematics or xacro:
    ```bash
    sudo apt install ros-iron-pinocchio -y
    sudo apt install ros-iron-xacro -y
    ```

3. **Colcon installation**

    Install Colcon, the build tool required for this project:
    ```bash
    sudo apt udpate
    sudo apt install python3-colcon-common-extensions
   ```
4. **Autocompletion**

    Enable autocompletion for Colcon:
    ```bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    ```

   To avoid sourcing it every time, add this line to the end of your `~/.bashrc` file.

### Build the Package

Navigate to your ROS workspace directory:

```bash
cd ROS_ws
```

Build the package using Colcon:

```bash
colcon build
```

Once built, source it to set up your environment:
```bash
source install/setup.bash
```

### Run simulation

To run the simulation, execute the following command:
```bash
ros2 launch umi_rtx_controller simu.launch.py
```

## Docker

### 1. Docker installation

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-docker2 nvidia-container-toolkit
sudo systemctl daemon-reload
sudo systemctl restart docker
```

### 2. Build the docker image

```bash
# Place yourself in umi_rtx_demos
docker build -t "name" .
```

Be careful to replace "name" with the name you want, and everything is ready !

### 3. Run the image into a container

```bash
# Give the permission to use the screen
xhost +

# Launch the container (replace "name" with the name you chose)
docker run --gpus all -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --rm "name":latest
```

### 4. Run the program

Start by running these commands:

```bash
cd ROS_ws/
colcon build
source install/setup.bash
cd ..
```

Then, if you want to launch only the simulation, run this:

```bash
ros2 launch umi_rtx_controller simu.launch.py
```

If you want to use the arm, run this:

```bash
./start_arm.sh
```


