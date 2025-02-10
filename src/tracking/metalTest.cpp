#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
using namespace std;
int main()
{

    if (cv::ocl::haveOpenCL()) {
        std::cout << "OpenCL is available!" << std::endl;
        cv::ocl::setUseOpenCL(true);
    } else {
        std::cout << "OpenCL is not available on this system." << std::endl;
    }
   cv::ocl::setUseOpenCL(true);
   cv::ocl::Context context;
    if (context.create(cv::ocl::Device::TYPE_ALL)) {

    cout << context.ndevices() << " GPU devices are detected." << endl; 
    for (int i = 0; i < context.ndevices(); i++)
    {
        cv::ocl::Device device = context.device(i);
        cout << "name:              " << device.name() << endl;
        cout << "available:         " << device.available() << endl;
        cout << "imageSupport:      " << device.imageSupport() << endl;
        cout << "OpenCL_C_Version:  " << device.OpenCL_C_Version() << endl;
        cout << endl;
        } //this works & i can see my video card name & opencl version
        cv::ocl::Device(context.device(0));
    }else{
        std::cout << "Failed to create OpenCL context." << std::endl;

    }
}