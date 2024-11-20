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
#include<opencv2/core/core.hpp>

#include "sys/types.h"
#include "sys/sysinfo.h"

#include<System.h>
using namespace std;
namespace ORB_SLAM2{
    using Seconds = double;
}

void LoadImages(const string &pathToSequence, vector<string> &imageFilenames, vector<ORB_SLAM2::Seconds> &timestamps);
std::string paddingZeros(const std::string& number, const size_t numberOfZeros = 5);

void removeSubstring(std::string& str, const std::string& substring) {
    size_t pos;
    while ((pos = str.find(substring)) != std::string::npos) {
        str.erase(pos, substring.length());
    }
}

int main(int argc, char **argv)
{

    // ORB_SLAM2 PLUS inputs
    bool activateVisualization{true};
    string path_to_vocabulary;
    string path_to_sequence;
    string path_to_output;
    string experimentIndex{"0"};
    for (int i = 0; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.find("Vis:") != std::string::npos) {
            removeSubstring(arg, "Vis:");
            activateVisualization = bool(std::stoi(arg));
            std::cout << "Activate Visualization = " << activateVisualization << std::endl;
            continue;
        }
        if (arg.find("Voc:") != std::string::npos) {
            removeSubstring(arg, "Voc:");
            string path_to_vocabulary_folder = arg;
            path_to_vocabulary =  path_to_vocabulary_folder + "/ORBvoc.txt";
            std::cout << "Path to vocabulary = " << path_to_vocabulary << std::endl;
            continue;
        }
        if (arg.find("sequence_path:") != std::string::npos) {
            removeSubstring(arg, "sequence_path:");
            path_to_sequence =  arg;
            std::cout << "Path to sequence = " << path_to_sequence << std::endl;
            continue;
        }
        if (arg.find("exp_folder:") != std::string::npos) {
            removeSubstring(arg, "exp_folder:");
            path_to_output =  arg;
            std::cout << "Path to output = " << path_to_output << std::endl;
            continue;
        }
        if (arg.find("exp_id:") != std::string::npos) {
            removeSubstring(arg, "exp_id:");
            experimentIndex =  arg;
            std::cout << "Exp id = " << experimentIndex << std::endl;
            continue;
        }
    }

    string path_to_settings = path_to_sequence + "/calibration.yaml";

    // Retrieve paths to images
    vector<string> imageFilenames{};
    vector<ORB_SLAM2::Seconds> timestamps{};
    LoadImages(path_to_sequence, imageFilenames, timestamps);

    size_t nImages = imageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings,
                           ORB_SLAM2::System::MONOCULAR,
                           activateVisualization);

    // Vector for tracking time statistics
    vector<ORB_SLAM2::Seconds> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(size_t ni = 0; ni < nImages; ni++)
    {
        // Read image from file
        im = cv::imread(imageFilenames[ni],cv::IMREAD_UNCHANGED);
        ORB_SLAM2::Seconds tframe = timestamps[ni];

        // Pass the image to the SLAM system
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        SLAM.TrackMonocular(im,tframe);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        ORB_SLAM2::Seconds ttrack = std::chrono::duration_cast<std::chrono::duration<ORB_SLAM2::Seconds> >(t2 - t1).count();
        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        ORB_SLAM2::Seconds T = 0.0;
        if(ni < nImages-1)
            T = timestamps[ni+1] - tframe;
        else if(ni > 0)
            T = tframe - timestamps[ni-1];

        if(ttrack < T)
            usleep((T-ttrack)  * 1e6);

    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    ORB_SLAM2::Seconds totaltime = 0.0;
    for(int ni = 0; ni < nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    string resultsPath_expId = path_to_output + "/" + paddingZeros(experimentIndex);
    SLAM.SaveKeyFrameTrajectoryTUM(resultsPath_expId + "_" + "KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &pathToSequence, vector<string> &imageFilenames, vector<ORB_SLAM2::Seconds> &timestamps)
{

    ifstream times;
    string pathToTimeFile = pathToSequence + "/rgb.txt";
    times.open(pathToTimeFile.c_str());

    string s0;
    while(!times.eof())
    {
        string s;
        getline(times,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            ORB_SLAM2::Seconds t;
            string sRGB;
            ss >> t;
            timestamps.push_back(t);
            ss >> sRGB;
            imageFilenames.push_back(pathToSequence + "/" +  sRGB);
        }
    }
}

std::string paddingZeros(const std::string& number, const size_t numberOfZeros){
    std::string zeros{};
    for(size_t iZero{}; iZero < numberOfZeros - number.size(); ++iZero)
        zeros += "0";
    return (zeros + number);
}
