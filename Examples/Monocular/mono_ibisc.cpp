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

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./orbslam2_loca3d path_to_vocabulary path_to_settings path_to_sequence log_directory_name" << endl;
        return 1;
    }

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    auto datetime_string = ss.str();

    console_logger = spd::stdout_color_mt("console");
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    unsigned long nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    console_logger->info("Starting sequence, using data-folder: {}", string(argv[3]));

    console_logger->info("Valid images number, found in this data-folder: {}", nImages);

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            console_logger->critical("Failed to load image at: {}", vstrImageFilenames[ni]);
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        auto outTcw = SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(cv::waitKey(30) >= 0){
            break;
        }

        if(ttrack<T)
            usleep(static_cast<__useconds_t>((T - ttrack) * 1e6));
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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
