# ros1-image-stream-encoder
a simple ros Noetic package that allows you to convert an image stream to various media files/outputs using ffmpeg. This was for use on a jetson nano for storing multiple image streams to the disk

### Tested on:
- `Ubuntu: 20.04`
- `ROS: Noetic`
- `GPU: 1070ti & Jetson Nano `

note : *it should work on other ros 1 versions and other linux distros without issue.*


# Install FFmpeg

## Without Hardware Acceleration

```bash
sudo apt install ffmpeg
```

## With NVidea hardware Acceleration

to improve encoding speed on a device with a dedicated nvidia gpu, you can build the FFMpeg library with the nvidia codecies
1. clone and build `ffnvcodec`
```bash
    git clone https://git.videolan.org/git/ffmpeg/nv-codec-headers.git
    cd nv-codec-headers && sudo make install && cd -
```
2. clone and build the `FFMpeg` public git repo with `cuda-nvcc` (NVENC/NVDEC) enabled 
```bash
git clone https://git.ffmpeg.org/ffmpeg.git ffmpeg/

sudo apt-get install build-essential yasm cmake libtool libc6 libc6-dev unzip wget libnuma1 libnuma-dev

./configure --enable-nonfree --enable-cuda-nvcc --enable-libnpp --extra-cflags=-I/usr/local/cuda/include --extra-ldflags=-L/usr/local/cuda/lib64 --disable-static --enable-shared

make -j 4
```

```bash
sudo make install
```


# Build and run the package

1. install the package
```bash
mkdir ~/catkin_ws/src -p && cd ~/catkin_ws/src

git clone {this package} 
cd ..
```

2. install dependencies with [rosdep](http://wiki.ros.org/rosdep)
```bash
rosdep install --from-paths src --ignore-src -r -y
```
3. build & source the package
```
catkin_make && . devel/setup.bash
```



## Example

The current example uses the `usb_cam` package to quickly create an image feed, it can be run via the following lines

```bash
    roslaunch image_stream_encoder example.launch
```
- note: *you will have to have a usb webcam attached and install the usb-cam package via the following line `sudo apt install ros-${ROS_DISTRO}-usb-cam`*