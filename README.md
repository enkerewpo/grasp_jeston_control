# GRASP JETSON CONTROL

Author: wheatfox

## QUICK START

1. Clone the repository recursively:

```bash
git clone --recursive https://github.com/enkerewpo/grasp_jeston_control
cd grasp_jeston_control
```

2. create a .env file in the root directory with the following variables:

```ini
DOCKERKEY_PERM=tskey-auth-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

where tskey-auth-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx is your Tailscale auth key, please generate one from https://login.tailscale.com/admin/settings/keys

3. run the script:

```bash
./run.sh -b # to build the docker image, this may take very long time since we will install ROS2+CUDA+PyTorch+etc.
```

4. after enter the container, find your container's tailscale IP address and modify the `shared/fast.xml` file, replace the "container" IP to your container's tailscale IP.

```xml
<locator>
    <udpv4>
        <address>xxx.xxx.xxx.xxx</address> <!-- Debian Docker IP -->
    </udpv4>
</locator>
```

This folder is mapped into the container so feel free to modify it anytime and the container will use it immediately.

5. in the container:

```bash

cd ~/src

./install_graspnet.sh
mv /opt/miniconda3/envs/env1/lib/libstdc++.so.6 /opt/miniconda3/envs/env1/lib/libstdc++.so.6.bak # to fix ROS2 under conda

./test1.sh # to run the official ROS1 demo
./test2.sh # to run the actual graspnet that use Jeston's camera topic input
```

## IP ADDRESSES BACKUP
Jetson IP:              10.7.61.73
Jetson Tailscale IP:    100.84.5.76 (old) 100.87.172.93 (new)
Debian IP:              100.117.99.116
Debian Docker IP:       100.99.31.104
