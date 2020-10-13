# ofxPointCloudLibrary
`ofxPointCloudLibrary` is an openFrameworks addon for the [Point Cloud Library](http://pointclouds.org/).

**Warning: Work in progress!**

## Versions

PointCloudLibrary v1.9.1, Windows x64  

Tested with:
  - openFramewoks v0.10.1 release
  - Windows Visual Studio 2017 x64

## Initial Setup (Required)

### **OF / Boost Library Fix**

PointCloudLibrary requires some Boost libraries that aren't included in openFrameworks by default.  

  - **Download and run the Boost v1.64 Visual Studio 2017 installer** from [Boost](https://dl.bintray.com/boostorg/release/1.64.0/binaries/):  
  **[ `boost_1_64_0-msvc-14.1-64.exe`](https://dl.bintray.com/boostorg/release/1.64.0/binaries/boost_1_64_0-msvc-14.1-64.exe)**  

    (you can install Boost anywhere on your hard drive)

  - **Copy the installed Boost library files into openFrameworks:**

    - **Boost headers**:  
      copy `boost_1_64_0/boost/` directory    
      overwrite `of_v0.10.1_vs2017_release/libs/boost/include/boost/`  

    - **Boost static libraries (*.lib)**  
      copy `boost_1_64_0/lib64-msvc-14.1/*.lib` files  
      overwrite `of_v0.10.1_vs2017_release/libs/boost/lib/vs/x64/*.lib`  

## Using with openFrameworks

Use the OF Project Generator to generate a Visual Studio 2017 project.

