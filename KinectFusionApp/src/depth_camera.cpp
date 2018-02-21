
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
#include <opencv/cv.hpp>

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

// ### Intel RealSense
RealSenseCamera::RealSenseCamera() : pipeline{}
{
    // Explicitly enable depth and color stream, with these constraints:
    // Same dimensions and color stream has format BGR 8bit
    rs2::config configuration {};
    configuration.disable_all_streams();
    configuration.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    configuration.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    // Use the first detected device, if any
    rs2::context ctx {};
    auto devices = ctx.query_devices();

    if(devices.size() == 0)
        throw std::runtime_error { "No RealSense device detected" };

    {
        auto device = devices[0]; // The device handle defined here is invalid after pipeline.start()

        // Print info about the device
        std::cout << "Using this RealSense device:" << std::endl;
        for ( int info_idx = 0; info_idx < static_cast<int>(RS2_CAMERA_INFO_COUNT); ++info_idx ) {
            auto info_type = static_cast<rs2_camera_info>(info_idx);
            std::cout << "  " << std::left << std::setw(20) << info_type << " : ";
            if ( device.supports(info_type))
                std::cout << device.get_info(info_type) << std::endl;
            else
                std::cout << "Not supported" << std::endl;
        }
    }

    pipeline.start(configuration);

    // Get depth sensor intrinsics
    auto streams = pipeline.get_active_profile().get_streams();
    for(const auto& stream : streams) {
        if(stream.stream_type() == RS2_STREAM_DEPTH) {
            auto intrinsics = stream.as<rs2::video_stream_profile>().get_intrinsics();
            cam_params.focal_x = intrinsics.fx;
            cam_params.focal_y = intrinsics.fy;
            cam_params.image_height = intrinsics.height;
            cam_params.image_width = intrinsics.width;
            cam_params.principal_x = intrinsics.ppx;
            cam_params.principal_y = intrinsics.ppy;
        }
    }

    // Get depth scale which is used to convert the measurements into millimeters
    depth_scale = pipeline.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
}

RealSenseCamera::RealSenseCamera(const std::string& filename) : pipeline{}
{
    rs2::config configuration {};
    configuration.disable_all_streams();
    configuration.enable_device_from_file(filename);
    pipeline.start(configuration);

    auto streams = pipeline.get_active_profile().get_streams();
    for(const auto& stream : streams) {
        if(stream.stream_type() == RS2_STREAM_DEPTH) {
            auto intrinsics = stream.as<rs2::video_stream_profile>().get_intrinsics();
            cam_params.focal_x = intrinsics.fx;
            cam_params.focal_y = intrinsics.fy;
            cam_params.image_height = intrinsics.height;
            cam_params.image_width = intrinsics.width;
            cam_params.principal_x = intrinsics.ppx;
            cam_params.principal_y = intrinsics.ppy;
        }
    }

    // Get depth scale which is used to convert the measurements into millimeters
    depth_scale = pipeline.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
}

InputFrame RealSenseCamera::grab_frame() const
{
    auto data = pipeline.wait_for_frames();
    auto depth = data.get_depth_frame();
    auto color = data.get_color_frame();

    cv::Mat depth_image { cv::Size { cam_params.image_width,
                                     cam_params.image_height },
                          CV_16UC1,
                          const_cast<void*>(depth.get_data()),
                          cv::Mat::AUTO_STEP};

    cv::Mat converted_depth_image;
    depth_image.convertTo(converted_depth_image, CV_32FC1, depth_scale * 1000.f);

    cv::Mat color_image { cv::Size { cam_params.image_width,
                                     cam_params.image_height },
                          CV_8UC3,
                          const_cast<void*>(color.get_data()),
                          cv::Mat::AUTO_STEP};

    return InputFrame {
            converted_depth_image,
            color_image
    };
}

CameraParameters RealSenseCamera::get_parameters() const
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
