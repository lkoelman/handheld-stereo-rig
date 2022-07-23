#!/bin/bash

set -e

readonly linux_pkgs=(
    python3
    python3-pip
    udev
    cmake
    git
    python3-numpy
)

readonly ubuntu_pkgs=(
    ${linux_pkgs[@]}
    # https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html
    build-essential
    libgtk2.0-dev
    pkg-config
    libavcodec-dev
    libavformat-dev
    libswscale-dev
    python3-dev
    libtbb2
    libtbb-dev
    libjpeg-dev
    libpng-dev
    libtiff-dev
    libdc1394-22-dev
    # https://stackoverflow.com/questions/55313610
    ffmpeg
    libsm6
    libxext6
    libgl1-mesa-glx
    python3-pyqt5
    python3-pyqt5.qtquick
    qml-module-qtquick-controls2
    qml-module-qt-labs-platform
    qtdeclarative5-dev
    qml-module-qtquick2
    qtbase5-dev
    qtchooser
    qt5-qmake
    qtbase5-dev-tools
    qml-module-qtquick-layouts
    qml-module-qtquick-window2
)

readonly ubuntu_arm_pkgs=(
    "${ubuntu_pkgs[@]}"
    # https://stackoverflow.com/a/53402396/5494277
    libhdf5-dev
    libhdf5-dev
    libatlas-base-dev
    libjasper-dev
    # https://github.com/EdjeElectronics/TensorFlow-Object-Detection-on-the-Raspberry-Pi/issues/18#issuecomment-433953426
    libilmbase-dev
    libopenexr-dev
    libgstreamer1.0-dev
)

readonly fedora_pkgs=(
    ${linux_pkgs[@]}
    gtk2-devel
    # Fedora uses pkgconf instead of pkg-config
    tbb-devel
    libjpeg-turbo-devel
    libpng-devel
    libtiff-devel
    libdc1394-devel
)

print_action () {
    green="\e[0;32m"
    reset="\e[0;0m"
    printf "\n$green >>$reset $*\n"
}
print_and_exec () {
    print_action $*
    $*
}


if [ -f /etc/os-release ]; then
    # shellcheck source=/etc/os-release
    source /etc/os-release

    if [[ "$ID" == "ubuntu" || "$ID" == "debian" || "$ID_LIKE" == "ubuntu" || "$ID_LIKE" == "debian" || "$ID_LIKE" == "ubuntu debian" ]]; then
        if [[ ! $(uname -m) =~ ^arm* ]]; then
            apt-get update
            apt-get install -y "${ubuntu_pkgs[@]}"
            python3 -m pip install --upgrade pip
        elif [[ $(uname -m) =~ ^arm* ]]; then
            apt-get update
            apt-get install -y "${ubuntu_arm_pkgs[@]}"
            python3 -m pip install --upgrade pip
        fi

        OS_VERSION=$(lsb_release -r |cut -f2)
        if [ "$OS_VERSION" == "21.04" ]; then
            echo -e "\033[33mThere are known issues with running our demo script on Ubuntu 21.04, due to package \"python3-pyqt5.sip\" not being in a correct version (>=12.9)\033[0m"
            echo -e "\033[33mWe recommend installing the updated version manually using the following commands\033[0m"
            echo -e "\033[33m$ wget http://mirrors.kernel.org/ubuntu/pool/universe/p/pyqt5-sip/python3-pyqt5.sip_12.9.0-1_amd64.deb\033[0m"
            echo -e "\033[33m$ dpkg -i python3-pyqt5.sip_12.9.0-1_amd64.deb\033[0m"
            echo ""
        fi
    else
        echo "ERROR: Distribution not supported"
        exit 99
    fi

    # Allow all users to read and write to Myriad X devices
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules
    # udevadm control --reload-rules && udevadm trigger
else
    echo "ERROR: Host not supported"
    exit 99
fi
