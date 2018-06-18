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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

#include"System.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_webcam path_to_vocabulary path_to_settings " << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing webcam input ..." << endl;
    //cout << "Images in the sequence: " << nImages << endl << endl;

    //Cam init
    cv::VideoCapture cap(1); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cv::Mat im, frame;
    //cv::namedWindow( "window", cv::WINDOW_AUTOSIZE );

    // Init VideoTimeStamp
    std::chrono::steady_clock::time_point startVideo = std::chrono::steady_clock::now();

    // Main loop
    int ni=0;
    while(true)
    {
        // Read image from webcam
        cap >> im;

        // Get Time of the frame
        std::chrono::steady_clock::time_point currentFrame = std::chrono::steady_clock::now();
        double time = std::chrono::duration_cast<std::chrono::nanoseconds>(currentFrame - startVideo).count();
        double tframe = time/1000000;

        if(im.empty())
        {
            cerr << endl << "Failed to read Camera" << endl;
            return 1;
        }

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

        if(cv::waitKey(30) >= 0){
            break;
        }

        //Wait for next frame
        std::chrono::steady_clock::time_point nextFrame = std::chrono::steady_clock::now();
        double time2 = std::chrono::duration_cast<std::chrono::microseconds>(nextFrame - currentFrame).count();
        if(time2<33333){
            usleep(33333-time2);
        }

//        usleep(100);

        ni++;
        if(cv::waitKey(30)>=0){break;}
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");


    return 0;
}
