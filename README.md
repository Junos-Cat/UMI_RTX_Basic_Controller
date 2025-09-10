# UMI RTX Robotic Arm Controller

## Background
This is a bare-bones keyboard controller for the UMI RTX Robotic Arm (Yumi). It uses the driver provided by [Physar](https://github.com/physar/umi-rtx), and two nodes from [GARDE & MASSA](https://github.com/gardegu/LAB42_RTX_control), namely the nodeArm and nodeInverseKinematics nodes.

I created this repo with modifications to GARDE & MASSA's code as their project requires the use of a Realsense camera and a GPU. The start_arm\_control.sh script runs the driver, the nodes that interact with the driver, and the controller node which interprets the keyboard inputs and publishes the commands to the /target_grip and /target_pose topics.

Please note, this project is not meant to provide a polished controller for the robotic arm. It is meant to allow others to install the minimum packages necessary to get their robotic arm up and moving. It is provided only so that others (including myself) may build upon it. Hopefully the controllers available will give you a few ideas of how you wish to control the arm.

For an example of another method of control, check out [nameguin](https://github.com/nameguin/umi_rtx_demos)'s project.

## Installation

To use this project, you will need to use Docker. To learn more about Docker, what it is, and why it's useful, see this [video](https://youtu.be/DQdB7wFEygo).

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

You may need to run it with elevated permissions as I did:
```bash
sudo docker build -t my-image-name:latest .
```

`.` means "build context is the current directory" so if you really want, you can replace this with the location to the aforementioned directory.

This will take a while the first time around but subsequent builds will be shorter.

2. Check your image
To list your local images, from any directory, run:
```bash
docker images
```

You should see `my-image-name` in the list.

3. Run a container from your image
To run a container from your image, from any directory, run:
```bash
sudo docker run -it my-image-name --name yumi
```

When working on your own packages, you may wish to make your own repository and change the following line in the `Dockerfile` to clone from your repository.
`RUN git clone https://github.com/gardegu/LAB42_RTX_control.git`
Or you may wish to do as I did during development, which was to [mount](https://docs.docker.com/engine/storage/bind-mounts/) your local directory as follows, and then create a repository when you're finished developing.
```bash
sudo docker run -it \
    --name yumi \
    -v ~/Desktop/JUNO/LAB42_RTX_control/:/home/Stage/LAB42_RTX_control/ \
    my-image-name
```
This means you won't have to rebuild the Docker image as often.

4. Check your container
You should now be able to see a list of running containers by running:
```bash
sudo docker ps
```
and a list of all running and stopped containers by running:
```bash
sudo docker ps -a
```

#### Notes on containers/useful commands
When containers are stopped (by entering `exit` in the container terminal), any changes made in the container persist unless you remove the container by running:
```bash
sudo docker rm yumi
```
Said changes do not transfer to any new containers unless you use volumes or mounts, in which case the data persists independently of the container lifecycle.

You can restart a container with all of its local changes by running:
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

## Running the UMI RTX Robotic Arm

### Run the Docker container

The Docker container needs access to the USB port the arm is plugged into. To view the USB devices recognised by your computer run:
```bash
ls /dev/ttyUSB*
```
Note the USB number (e.g. /dev/ttyUSB0) as this is the name you need to pass in order to give the Docker container access.

Now run the container, giving it access to the USB port:
```bash
sudo docker run -it \
    --name yumi \
    --device=/dev/ttyUSB0 \
    my-image-name
```

If you did not remove the previous container and don't wish to restart it with `start` but create a new one with `run`, then run:
```bash
sudo docker rm -f yumi
sudo docker run -it \
    --name yumi \
    --device=/dev/ttyUSB0 \
    my-image-name
```

### Start the arm and controller

To start the arm run:
```bash
./start_arm_control.sh
```
Take a look at what happens in this script as you will need to edit it to make your own control mechanisms.
