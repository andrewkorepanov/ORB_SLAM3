/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

int main(int argc, char **argv)
{  
    if(argc < 4 || argc > 5)
    {
        cerr << endl << "Usage: ./mono_video path_to_vocabulary path_to_settings path_to_videofile (trajectory_file_name)" << endl;
        return 1;
    }
    
    string voc_file_name;
    voc_file_name = string(argv[1]);
    cout << "Vocabulary Filename: " << voc_file_name << std::endl;
    
    string settings_file_name;
    settings_file_name = string(argv[2]);
    cout << "Settings Filename: " << settings_file_name << std::endl;
    
    string video_file_name;
    video_file_name = string(argv[3]);
    cout << "Video Filename: " << video_file_name << std::endl;

    string file_name;
    bool bFileName = false;
    if (argc == 5)
    {
        file_name = string(argv[argc-1]);
        bFileName = true;
    }


    cout << "Opening video: " << video_file_name << "...";
    auto video = cv::VideoCapture(video_file_name);
    if (!video.isOpened()) {
        std::cerr << "Unable to open the video." << std::endl;
        return 1;
    }
    const unsigned int video_frame_count = static_cast<unsigned int>(video.get(cv::CAP_PROP_FRAME_COUNT));

    int fps = 20;
    float dT = 1.f/fps;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file_name,settings_file_name,ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    cv::Mat frame;
    const unsigned int frame_skip = 1;
    unsigned int num_frame = 0;
    double tframe_prev = -1;

    bool is_not_end = true;
    while (is_not_end) {
        is_not_end = video.read(frame);
        if (!is_not_end) {
            break;
        }

        if (!frame.empty() && (num_frame % 100 == 0)) {
            std::cout << "Processed: " << num_frame << " of " << video_frame_count << " frames = " << 100.0 * num_frame / video_frame_count << "%" << std::endl;
        } 

        if (!frame.empty() && (num_frame % frame_skip == 0)) {

            cv::Mat im = frame;
            double tframe = video.get(cv::CAP_PROP_POS_MSEC);

            if (tframe>tframe_prev) { // Only allow incremental timestamps

                if(imageScale != 1.f)
                {
    #ifdef REGISTER_TIMES
        #if defined(COMPILEDWITHC11) || defined(COMPILEDWITHC14)
                    std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
        #else
                    std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
        #endif
    #endif
                    int width = im.cols * imageScale;
                    int height = im.rows * imageScale;
                    cv::resize(im, im, cv::Size(width, height));
    #ifdef REGISTER_TIMES
        #if defined(COMPILEDWITHC11) || defined(COMPILEDWITHC14)
                    std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
        #else
                    std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
        #endif
                    t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                    SLAM.InsertResizeTime(t_resize);
    #endif
                }

        #if defined(COMPILEDWITHC11) || defined(COMPILEDWITHC14)
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
                std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif

                // Pass the image to the SLAM system
                // cout << "tframe = " << tframe << endl;
                SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial

        #if defined(COMPILEDWITHC11) || defined(COMPILEDWITHC14)
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
                std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif

    #ifdef REGISTER_TIMES
                t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
                SLAM.InsertTrackTime(t_track);
    #endif

            }
            tframe_prev = tframe;
        }
        ++num_frame;
    }


    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryTUM(f_file);
        SLAM.SaveKeyFrameTrajectoryTUM(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    return 0;
}
