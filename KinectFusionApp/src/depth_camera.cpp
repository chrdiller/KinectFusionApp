
#include <depth_camera.h>

#include <iostream>
#include <fstream>
#include <iomanip>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Weffc++"
#include <PS1080.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#pragma GCC diagnostic pop

// ### Pseudo ###
PseudoCamera::PseudoCamera(const std::string& _data_path) :
        data_path{_data_path}, cam_params{}, current_index{0}
{
    std::ifstream cam_params_stream { data_path + "seq_cparam.txt" };
    if (!cam_params_stream.is_open())
        throw std::runtime_error{"Camera parameters could not be read"};
    cam_params_stream >> cam_params.image_width >> cam_params.image_height;
    cam_params_stream >> cam_params.focal_x >> cam_params.focal_y;
    cam_params_stream >> cam_params.principal_x >> cam_params.principal_y;
};

InputFrame PseudoCamera::grab_frame () const
{
    std::stringstream depth_file;
    depth_file << data_path << "seq_depth" << std::setfill('0') << std::setw(5) << current_index << ".png";
    std::stringstream color_file;
    color_file << data_path << "seq_color" << std::setfill('0') << std::setw(5) << current_index << ".png";

    InputFrame frame {};
    cv::imread(depth_file.str(), -1).convertTo(frame.depth_map, CV_32FC1);

    if (frame.depth_map.empty()) {  // When this happens, we reached the end of the recording and have to
                                    // start at 0 again
        current_index = 0;
        depth_file = std::stringstream {};
        color_file = std::stringstream {};
        depth_file << data_path << "seq_depth" << std::setfill('0') << std::setw(5) << current_index << ".png";
        color_file << data_path << "seq_color" << std::setfill('0') << std::setw(5) << current_index << ".png";
        frame.depth_map = cv::imread(depth_file.str(), -1);
    }

    frame.color_map = cv::imread(color_file.str());

    ++current_index;

    return frame;
}

CameraParameters PseudoCamera::get_parameters() const
{
    return cam_params;
}

// ### Asus Xtion PRO LIVE
XtionCamera::XtionCamera() :
        device{}, depthStream{}, colorStream{}, depthFrame{},
        colorFrame{}, cam_params{}
{
    openni::OpenNI::initialize();

    openni::Array<openni::DeviceInfo> deviceInfoList;
    openni::OpenNI::enumerateDevices(&deviceInfoList);

    std::cout << deviceInfoList.getSize() << std::endl;
    for (int i = 0; i < deviceInfoList.getSize(); ++i) {
        std::cout << deviceInfoList[i].getName() << ", "
                  << deviceInfoList[i].getVendor() << ", "
                  << deviceInfoList[i].getUri() << ", "
                  << std::endl;
    }

    auto ret = device.open(openni::ANY_DEVICE);
    if (ret != openni::STATUS_OK)
        throw std::runtime_error{"OpenNI device could not be opened"};

    openni::VideoMode depthMode;
    depthMode.setResolution(640, 480);
    depthMode.setFps(30);
    depthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);

    openni::VideoMode colorMode;
    colorMode.setResolution(640, 480);
    colorMode.setFps(30);
    colorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

    depthStream.create(device, openni::SENSOR_DEPTH);
    depthStream.setVideoMode(depthMode);
    depthStream.start();

    colorStream.create(device, openni::SENSOR_COLOR);
    colorStream.setVideoMode(colorMode);

    openni::CameraSettings *cameraSettings = colorStream.getCameraSettings();
    cameraSettings->setAutoExposureEnabled(true);
    cameraSettings->setAutoWhiteBalanceEnabled(true);
    cameraSettings = colorStream.getCameraSettings();


    if (cameraSettings != nullptr) {
        std::cout << "Camera Settings" << std::endl;
        std::cout << " Auto Exposure Enabled      : " << cameraSettings->getAutoExposureEnabled() << std::endl;
        std::cout << " Auto WhiteBalance Enabled  : " << cameraSettings->getAutoWhiteBalanceEnabled() << std::endl;
        std::cout << " Exposure                   : " << cameraSettings->getExposure() << std::endl;
        std::cout << " Gain                       : " << cameraSettings->getGain() << std::endl;
    }

    colorStream.start();

    if (device.setDepthColorSyncEnabled(true) != openni::STATUS_OK) {
        std::cout << "setDepthColorSyncEnabled is disabled" << std::endl;
    }
    if (device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) != openni::STATUS_OK) {
        std::cout << "setImageRegistrationMode is disabled" << std::endl;
    }

    double pixelSize;
    depthStream.getProperty<double>(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE, &pixelSize);

    // pixel size @ VGA = pixel size @ SXGA x 2
    pixelSize *= 2.0; // in mm

    // focal length of IR camera in pixels for VGA resolution
    int zeroPlaneDistance; // in mm
    depthStream.getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE, &zeroPlaneDistance);

    double baseline;
    depthStream.getProperty<double>(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE, &baseline);
    baseline *= 10.0;

    // focal length from mm -> pixels (valid for 640x480)
    double depthFocalLength_VGA = (int) (static_cast<double>(zeroPlaneDistance) / pixelSize);

    CameraParameters cp {};
    cp.image_width = depthStream.getVideoMode().getResolutionX();
    cp.image_height = depthStream.getVideoMode().getResolutionY();
    cp.focal_x = cp.focal_y = (float) depthFocalLength_VGA;
    cp.principal_x = cp.image_width / 2 - 0.5f;
    cp.principal_y = cp.image_height / 2 - 0.5f;

    cam_params = cp;
}

InputFrame XtionCamera::grab_frame() const
{
    depthStream.readFrame(&depthFrame);
    colorStream.readFrame(&colorFrame);

    if (!depthFrame.isValid() || depthFrame.getData() == nullptr ||
        !colorFrame.isValid() || colorFrame.getData() == nullptr) {
        throw std::runtime_error{"Frame data retrieval error"};
    } else {
        cv::Mat depthImg16U { depthStream.getVideoMode().getResolutionY(),
                              depthStream.getVideoMode().getResolutionX(),
                              CV_16U,
                              static_cast<char*>(const_cast<void*>(depthFrame.getData())) };
        cv::Mat depth_image;
        depthImg16U.convertTo(depth_image, CV_32FC1);
        cv::flip(depth_image, depth_image, 1);

        cv::Mat color_image { colorStream.getVideoMode().getResolutionY(),
                              colorStream.getVideoMode().getResolutionX(),
                              CV_8UC3,
                              static_cast<char*>(const_cast<void*>(colorFrame.getData())) };
        cv::cvtColor(color_image, color_image, cv::COLOR_BGR2RGB);
        cv::flip(color_image, color_image, 1);

        return InputFrame { depth_image, color_image };
    }
}

CameraParameters XtionCamera::get_parameters() const
{
    return cam_params;
}


// ### Kinect ###
/*
KinectCamera::KinectCamera()
{

}

KinectCamera::~KinectCamera()
{

}

InputFrame KinectCamera::grab_frame() const
{

}

CameraParameters KinectCamera::get_parameters() const
{

}
*/
