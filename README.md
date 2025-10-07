# ros-image-stream-encoder
a simple ros Noetic package that allows you to convert an image topic to various media files/outputs using the `ffmpeg` library and hardware acceleration. This was designed for use on a jetson nano for storing multiple image streams to the disk.

### Tested on:
- `Ubuntu: 20.04`
- `ROS: Noetic`
- `GPU: 1070ti & Jetson Nano `

> [!note]
>  *it should work on other ros 1 versions and other linux distros without issue.*


# Install FFmpeg

## Without Hardware Acceleration

```bash
sudo apt install ffmpeg
```


## With NVidea hardware Acceleration on Jetson

This Section uses a patched version of FFMpeg created by [jocover](https://github.com/jocover/jetson-ffmpeg) and extended by LinusCDE and can be found [here](https://github.com/LinusCDE/mad-jetson-ffmpeg). Theoretically the installation of ffmpeg should result in the same across the two packages, however I only found success with LinusCDE's package and thus I use it here despite its larger build time.
1. Ensure that ffmpeg is uninstalled 
```bash
    sudo apt remove ffmpeg
```
1. Install dependencies:
```bash
    sudo apt install build-essential cmake pkg-config bzip2 fontconfig libfribidi{0,-dev} gmpc{,-dev} gnutls-bin lame libass{9,-dev} libavc1394-{0,dev} libbluray{2,-dev} libdrm{2,-dev} libfreetype6{,-dev} libmodplug{1,-dev} libraw1394-{11,dev} librsvg2{-2,-dev} libsoxr{0,-dev} libtheora{0,-dev} libva{2,-dev} libva-drm2 libva-x11-2 libvdpau{1,-dev} libvorbisenc2 libvorbis{0a,-dev} libvpx{6,-dev} libwebp{6,-dev} libx11{-6,-dev} libx264-{155,dev} libx265-{179,dev} libxcb1{,-dev} libxext{6,-dev} libxml2{,-dev} libxv{1,-dev} libxvidcore{4,-dev} libopencore-amr{nb0,nb-dev,wb0,wb-dev} opus-tools libsdl2-dev speex v4l-utils zlib1g{,-dev} libopenjp2-7{,-dev} libssh-{4,dev} libspeex{1,-dev} libgmp-dev libgnutls28-dev libladspa-ocaml-dev libmp3lame{0,-dev} libopus{0,-dev} libv4l-{0,dev}
```
2. Build library
```bash
    git clone https://github.com/LinusCDE/mad-jetson-ffmpeg.git -b release/6.0
    cd mad-jetson-ffmpeg
    ./configure \
  --prefix=/usr/local \
  --extra-cflags="-I/usr/local/include" \
  --extra-ldflags="-L/usr/local/lib" \
  --disable-debug \
  --disable-stripping \
  --enable-lto \
  --enable-fontconfig \
  --enable-gmp \
  --enable-gnutls \
  --enable-gpl \
  --enable-ladspa \
  --enable-libass \
  --enable-libbluray \
  --enable-libdrm \
  --enable-libfreetype \
  --enable-libfribidi \
  --enable-libmodplug \
  --enable-libmp3lame \
  --enable-libopencore_amrnb \
  --enable-libopencore_amrwb \
  --enable-libopenjpeg \
  --enable-libopus \
  --enable-libpulse \
  --enable-librsvg \
  --enable-libsoxr \
  --enable-libspeex \
  --enable-libssh \
  --enable-libtheora \
  --enable-libv4l2 \
  --enable-libvorbis \
  --enable-libvpx \
  --enable-libwebp \
  --enable-libx264 \
  --enable-libx265 \
  --enable-libxcb \
  --enable-libxml2 \
  --enable-libxvid \
  --enable-version3 \
  --enable-nonfree \
  --enable-nvmpi \
  --enable-nvv4l2dec \
  --extra-libs="-L/usr/lib/aarch64-linux-gnu/tegra -lnvbuf_utils" \
  --extra-cflags="-I /usr/src/jetson_multimedia_api/include/" \
  --enable-shared

    # build
    make -j$(nproc)
```
2. Install
```bash
    sudo make install
```


## With hardware Acceleration on x86_64 (Hasn't been tested fully yet)

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
- note: *you will have to have a usb webcam attached and install the usb-cam package via the following line `sudo apt install ros-${ROS_DISTRO}-usb-cam ros-${ROS_DISTRO}-image-view`*

## Supported Codecies / Encoders
- Node defaults to `h264_nvenc`  which utilises the `NVENC` cores on the jetson's GPU.
- By default, all are available to work with, you can find a list of them by running `ffmpeg -codecs` or `ffmpeg -encoders`


## Common Errors:

#### FFMPEG Relocation Against Symbol
```bash
/usr/bin/ld: /usr/local/lib/libavutil.a(tx_float_neon.o): relocation R_AARCH64_ADR_PREL_PG_HI21 against symbol `ff_tx_tab_2048_float' which may bind externally can not be used when making a shared object; recompile with -fPIC

```
[Solution](https://stackoverflow.com/questions/13812185/how-to-recompile-with-fpic):
- Disable assembler optimisations (add --disable-asm to the configure line)
- Compile dynamic libraries
