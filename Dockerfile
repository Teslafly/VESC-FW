FROM gcc:11.2

RUN apt-get update \
    && apt-get -y install git 
# scons bzr lib32z1 lib32ncurses5

# Set up a toolchain dev directory
WORKDIR /home/devtools

# get and extract the gcc-arm-none-eabi toolchain
RUN wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2 \
    && tar xvf gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2 \
    && rm gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2

# Set up the compiler path
ENV PATH $PATH:/home/devtools/gcc-arm-none-eabi-6-2017-q2-update/bin

WORKDIR /usr/project
COPY . /usr/project

#todo: hot to debug with gcc inside container connected to external openocd adapter. start with vscode