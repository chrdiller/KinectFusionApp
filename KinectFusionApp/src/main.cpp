
#include <kinectfusion.h>
#include <depth_camera.h>
#include <util.h>

#include <iostream>
#include <fstream>
#include <iomanip>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Weffc++"
#include <opencv2/highgui.hpp>
#pragma GCC diagnostic pop

int main()
{
    const std::string data_path = "/media/christian/Barracuda/IDP/dataset_new/";
    const std::string recording_name = "yunus2";

    std::stringstream source_path {};
    source_path << data_path << "source/" << recording_name << "/";

    PseudoCamera camera = PseudoCamera { source_path.str() };
    //XtionCamera camera = XtionCamera {};

    cv::namedWindow("Pipeline Output");

    kinectfusion::GlobalConfiguration configuration;
    configuration.voxel_scale = 2.f;
    configuration.triangles_buffer_size *= 25;
    configuration.init_depth = 1000.f;
    configuration.distance_threshold = 5.f;
    configuration.angle_threshold = 20.f;
    configuration.icp_iterations = std::vector<int>{4, 5, 10};
    configuration.volume_size.x = 512;
    configuration.volume_size.y = 512;
    configuration.volume_size.z = 512;
    configuration.depth_cutoff_distance = 1500.f;
    configuration.truncation_distance = 25.f;
    kinectfusion::Pipeline pipeline { camera.get_parameters(), configuration };

    for (bool end = false; !end;) {
        //1 Get frame
        InputFrame frame = camera.grab_frame();

        //2 Process frame
        bool success = pipeline.process_frame(frame.depth_map, frame.color_map);
        if (!success)
            std::cout << "Frame could not be processed" << std::endl;

        //3 Display the output
        cv::imshow("Pipeline Output", pipeline.get_last_model_frame());

        switch (cv::waitKey(1)) {
            case 'a': { // Save all available data
                std::cout << "Saving all ..." << std::endl;
                std::cout << "Saving poses ..." << std::endl;
                auto poses = pipeline.get_poses();

                for (size_t i = 0; i < poses.size(); ++i) {
                    std::stringstream file_name {};
                    file_name << data_path << "poses/" << recording_name << "/seq_pose" << std::setfill('0')
                              << std::setw(5) << i << ".txt";
                    std::ofstream { file_name.str() } << poses[i] << std::endl;
                }

                std::cout << "Extracting mesh ..." << std::endl;
                auto mesh = pipeline.extract_mesh();
                std::cout << "Saving mesh ..." << std::endl;
                std::stringstream file_name {};
                file_name << data_path << "meshes/" << recording_name << ".ply";
                kinectfusion::export_ply(file_name.str(), mesh);
                end = true;
                break;
            }
            case 'p': { // Save poses only
                std::cout << "Saving poses ..." << std::endl;
                auto poses = pipeline.get_poses();

                for (size_t i = 0; i < poses.size(); ++i) {
                    std::stringstream file_name {};
                    file_name << data_path << "poses/" << recording_name << "/seq_pose" << std::setfill('0')
                              << std::setw(5) << i << ".txt";
                    std::ofstream { file_name.str() } << poses[i] << std::endl;
                }
                end = true;
                break;
            }
            case 'm': { // Save mesh only
                std::cout << "Extracting mesh ..." << std::endl;
                auto mesh = pipeline.extract_mesh();
                std::cout << "Saving mesh ..." << std::endl;
                std::stringstream file_name {};
                file_name << data_path << "meshes/" << recording_name << ".ply";
                kinectfusion::export_ply(file_name.str(), mesh);
                end = true;
                break;
            }
            case ' ': // Save nothing
                end = true;
                break;
            default:
                break;
        }
    }

    return EXIT_SUCCESS;
}
