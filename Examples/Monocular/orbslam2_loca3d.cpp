//#include <iostream>
//#include "System.h"

//int main() {
//    std::cout << "Hello, World!" << std::endl;
//    return 0;
//}

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


// standard libs
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include <unistd.h>

// non-standard libs to be installed as third-party into system (apt / vcpkg)
#include<opencv2/core/core.hpp>

// ORB-SLAM lib, to be installed by YOU.
#include "System.h"

// spdlog lib, to be installed by you from https://github.com/gabime/spdlog (or try vcpkg?)
// Ubuntu apt-get install libspdlog-dev DOESNT WORK due to OLDNESS.
// vcpkg probably will have problems with fmtlib NON-BUNDLED to spdlog and therefore PROBLEMS (same as in Ubuntu).
#include<spdlog/spdlog.h>
#include <include/Converter.h>

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


namespace spd = spdlog;

std::shared_ptr<spd::logger> console_logger;
std::shared_ptr<spd::logger> processing_times_logger;
std::shared_ptr<spd::logger> processing_rt_logger;

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./orbslam2_loca3d path_to_vocabulary path_to_settings path_to_sequence log_directory_name" << endl;
        return 1;
    }

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    auto datetime_string = ss.str();

    std::string logging_directory =  "logs/";
    logging_directory += argv[4];
    console_logger = spd::stdout_color_mt("console");
    processing_rt_logger = spd::basic_logger_mt("orb_slam_rt",  logging_directory + "/frame_transformation_" + datetime_string + ".txt");
    processing_times_logger = spd::basic_logger_mt("orb_slam_time", logging_directory + "/frame_processing_times_" + datetime_string + ".txt");
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    unsigned long nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<double> vTimesTrack;
    vTimesTrack.resize(nImages);

    processing_times_logger->info("Starting sequence, using data-folder: {}", string(argv[3]));
    console_logger->info("Starting sequence, using data-folder: {}", string(argv[3]));
    processing_rt_logger->info("Starting sequence, using data-folder: {}", string(argv[3]));

    processing_times_logger->info("Valid images number, found in this data-folder: {}", nImages);
    console_logger->info("Valid images number, found in this data-folder: {}", nImages);

    //    cout << endl << "-------" << endl;
//    cout << "Start processing sequence ..." << endl;
//    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            processing_times_logger->critical("Failed to load image at: {}", vstrImageFilenames[ni]);
            console_logger->critical("Failed to load image at: {}", vstrImageFilenames[ni]);
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
//        auto frame = SLAM.TrackMonocular(im,tframe);
        auto outTcw = SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        processing_times_logger->info("Frame {}: tracking time is {} seconds", ni, ttrack);

        // try to save on-the-fly Rotation and Translation, if any

        if (!outTcw.empty() && outTcw.rows > 0 && outTcw.cols > 0) {
            cv::Mat R = outTcw.rowRange(0,3).colRange(0,3).t();
            vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
            cv::Mat Rcw = outTcw.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = outTcw.rowRange(0,3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat Ow = -Rwc*tcw;

            processing_rt_logger->info("{}: {:03.7f} {:03.7f} {:03.7f} {:03.7f} {:03.7f} {:03.7f} {:03.7f} T",
                                       tframe, Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2), q[0], q[1],
                                       q[2], q[3]);
        } else {
            processing_rt_logger->info("{}: 0 0 0 0 0 0 1 F", tframe);

        }

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep(static_cast<__useconds_t>((T - ttrack) * 1e6));

        /*if (SLAM.GetPauseRequestedAndToggle()) {
            usleep(static_cast<__useconds_t>(10e6));
        }*/
    }


    usleep(static_cast<__useconds_t>(20e6));

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    processing_times_logger->info("Median tracking time: {}", vTimesTrack[nImages/2]);
    console_logger->info("Median tracking time: {}", vTimesTrack[nImages/2]);
    processing_times_logger->info("Mean tracking time: {}", totaltime/nImages);
    console_logger->info("Mean tracking time: {}", totaltime/nImages);

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/../time_passed.txt";
    fTimes.open(strPathTimeFile.c_str());
    if (!fTimes) {
        console_logger->critical(strPathTimeFile.c_str());
        console_logger->critical("Problem with the timestamps file: not opened or empty. Maybe, you have lost it, or path is incorrect?");
        exit(-1);
    }

    string s;
    int counter = 0;
    while(!fTimes.eof())
    {
        counter ++;
        if (!getline(fTimes,s) && !fTimes.eof()) {
            processing_times_logger->error("Problem with the timestamps file: not really correct string found in the file (probably around line {})", counter);
            console_logger->error("Problem with the timestamps file: not really correct string found in the file (probably around line {})", counter);

//            cerr << "Problem while reading times.txt file" << endl;
            break;
        }

        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/"; //"/image_0/";

    unsigned long nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(4) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
