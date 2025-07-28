# Active-Searcher

## Quick Start

This project has been tested on Ubuntu 18.04(ROS Melodic) and 20.04(ROS Noetic).

Firstly, you should install __nlopt v2.7.1__:
```
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

Next, you can run the following commands to install other required tools:
```
sudo apt-get install libarmadillo-dev
```

After that, you need to install __LKH-3__(LKH-3.0.6 version is recommended) with the following commands. Please make sure the executable file `LKH` is correctly placed at `/usr/local/bin`.

```
wget http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz
tar xvfz LKH-3.0.6.tgz
cd LKH-3.0.6
make
sudo cp LKH /usr/local/bin
```

```
  set(CUDA_NVCC_FLAGS 
    -gencode arch=compute_61,code=sm_61;
  ) 
``` 

Then simply clone and compile our package (using ssh here):

```
cd ${YOUR_WORKSPACE_PATH}/src
git clone https://github.com/Chai8226/Active-Searcher.git
cd ../ 
catkin_make
```

After compilation you can start a search demo.

```
sh shfiles/search.sh
```