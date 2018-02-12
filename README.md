KinectFusionApp
===============

This is a sample application using the [KinectFusionLib](https://github.com/chrdiller/KinectFusionLib). It implements 
cameras (for data acquisition from recordings as well as from a live depth sensor) as data sources. The resulting fused volume 
can then be exported into a pointcloud or a dense surface mesh.

Dependencies
------------
* **GCC 5** as higher versions don't work with current nvcc (as of 2017).
* **CUDA 8.0** or higher. In order to provide real-time reconstruction, this library relies on graphics hardware.
Running it exclusively on CPU is not possible.
* **OpenCV 3.0** or higher. This library heavily depends on the GPU features of OpenCV that have been refactored in the 3.0 release.
Therefore, OpenCV 2 is not supported.
* **Eigen3** for efficient matrix and vector operations.
* **OpenNI2** for data acquisition with a live depth sensor.

Prerequisites
-------------
* Adjust CUDA architecture: Set the CUDA architecture version to that of your graphics hardware
```cmake
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -gencode arch=compute_52,code=sm_52)
```
Tested with a nVidia GeForce 970, compute capability 5.2, maxwell architecture
* Set custom opencv path (if built from source):
```cmake
SET("OpenCV_DIR" "/opt/opencv/usr/local/share/OpenCV")
```

Usage
-----
Setup the data sources in main.cpp. Then, start the application.

Use the following keys to perform actions:
* 'p': Export all camera poses known so far
* 'm': Export a dense surface mesh
* ' ': Export nothing, just end the application
* 'a': Save all available data

License
-------
This library is licensed under MIT.
