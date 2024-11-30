/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "sys/sysinfo.h"
#include "sys/types.h"

#include <System.h>
using namespace std;
namespace ORB_SLAM2 {
using Seconds = double;
}

// void LoadImages(const string &pathToSequence, const string &rgb_txt,
//                 vector<string> &imageFilenames,
//                 vector<ORB_SLAM2::Seconds> &timestamps);

void LoadImages(const string &pathToSequence, const string &rgb_txt,
                vector<string> &leftImageFilenames,
                vector<string> &rightImageFilenames,
                vector<ORB_SLAM2::Seconds> &timestamps);

std::string paddingZeros(const std::string &number,
                         const size_t numberOfZeros = 5);

void removeSubstring(std::string &str, const std::string &substring) {
  size_t pos;
  while ((pos = str.find(substring)) != std::string::npos) {
    str.erase(pos, substring.length());
  }
}

int main(int argc, char **argv) {

  // ORB_SLAM2 PLUS inputs
  string sequence_path;
  string calibration_yaml;
  string rgb_txt;
  string exp_folder;
  string exp_id{"0"};
  string settings_yaml{"orbslam2_settings.yaml"};
  bool verbose{true};

  string vocabulary{"Vocabulary/ORBvoc.txt"};
  cout << endl;
  for (int i = 0; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.find("sequence_path:") != std::string::npos) {
      removeSubstring(arg, "sequence_path:");
      sequence_path = arg;
      std::cout << "[stereo_vslamlab.cpp] Path to sequence = " << sequence_path
                << std::endl;
      continue;
    }
    if (arg.find("calibration_yaml:") != std::string::npos) {
      removeSubstring(arg, "calibration_yaml:");
      calibration_yaml = arg;
      std::cout << "[stereo_vslamlab.cpp] Path to calibration.yaml = "
                << calibration_yaml << std::endl;
      continue;
    }
    if (arg.find("rgb_txt:") != std::string::npos) {
      removeSubstring(arg, "rgb_txt:");
      rgb_txt = arg;
      std::cout << "[stereo_vslamlab.cpp] Path to rgb_txt = " << rgb_txt
                << std::endl;
      continue;
    }
    if (arg.find("exp_folder:") != std::string::npos) {
      removeSubstring(arg, "exp_folder:");
      exp_folder = arg;
      std::cout << "[stereo_vslamlab.cpp] Path to exp_folder = " << exp_folder
                << std::endl;
      continue;
    }
    if (arg.find("exp_id:") != std::string::npos) {
      removeSubstring(arg, "exp_id:");
      exp_id = arg;
      std::cout << "[stereo_vslamlab.cpp] Exp id = " << exp_id << std::endl;
      continue;
    }
    if (arg.find("settings_yaml:") != std::string::npos) {
      removeSubstring(arg, "settings_yaml:");
      settings_yaml = arg;
      std::cout << "[stereo_vslamlab.cpp] Path to settings_yaml = "
                << settings_yaml << std::endl;
      continue;
    }
    if (arg.find("verbose:") != std::string::npos) {
      removeSubstring(arg, "verbose:");
      verbose = bool(std::stoi(arg));
      std::cout << "[stereo_vslamlab.cpp] Activate Visualization = " << verbose
                << std::endl;
      continue;
    }
    if (arg.find("vocabulary:") != std::string::npos) {
      removeSubstring(arg, "vocabulary:");
      vocabulary = arg;
      std::cout << "[stereo_vslamlab.cpp] Path to vocabulary = " << vocabulary
                << std::endl;
      continue;
    }
  }

  // Retrieve paths to images
  vector<string> leftImageFilenames{};
  vector<string> rightImageFilenames{};
  vector<ORB_SLAM2::Seconds> timestamps{};
  LoadImages(sequence_path, rgb_txt, leftImageFilenames, rightImageFilenames,
             timestamps);

  size_t nImages = leftImageFilenames.size();

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(vocabulary, calibration_yaml, settings_yaml,
                         ORB_SLAM2::System::STEREO, verbose);

  // Vector for tracking time statistics
  vector<ORB_SLAM2::Seconds> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat im_left, im_right;
  for (size_t ni = 0; ni < nImages; ni++) {
    // Read image from file
    im_left = cv::imread(leftImageFilenames[ni], cv::IMREAD_UNCHANGED);
    im_right = cv::imread(rightImageFilenames[ni], cv::IMREAD_UNCHANGED);
    ORB_SLAM2::Seconds tframe = timestamps[ni];

    // Pass the image to the SLAM system
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // SLAM.TrackMonocular(im,tframe);
    SLAM.TrackStereo(im_left, im_right, tframe);    
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    ORB_SLAM2::Seconds ttrack =
        std::chrono::duration_cast<std::chrono::duration<ORB_SLAM2::Seconds>>(
            t2 - t1)
            .count();
    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    ORB_SLAM2::Seconds T = 0.0;
    if (ni < nImages - 1)
      T = timestamps[ni + 1] - tframe;
    else if (ni > 0)
      T = tframe - timestamps[ni - 1];

    if (ttrack < T)
      usleep((T - ttrack) * 1e6);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  ORB_SLAM2::Seconds totaltime = 0.0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  string resultsPath_expId = exp_folder + "/" + paddingZeros(exp_id);
  SLAM.SaveKeyFrameTrajectoryTUM(resultsPath_expId + "_" +
                                 "KeyFrameTrajectory.txt");

  return 0;
}

void LoadImages(const string &pathToSequence, const string &rgb_txt,
                vector<string> &leftImageFilenames,
                vector<string> &rightImageFilenames,
                vector<ORB_SLAM2::Seconds> &timestamps) {

  ifstream times;
  times.open(rgb_txt.c_str());

  string s0;
  while (!times.eof()) {
    string s;
    getline(times, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;

      ORB_SLAM2::Seconds t;
      string sLeft;
      string sRight;
      ss >> t;
      timestamps.push_back(t);
      ss >> sLeft;
      ss >> sRight;
      leftImageFilenames.push_back(pathToSequence + "/" + sLeft);
      rightImageFilenames.push_back(pathToSequence + "/" + sRight);
    }
  }
}

std::string paddingZeros(const std::string &number,
                         const size_t numberOfZeros) {
  std::string zeros{};
  for (size_t iZero{}; iZero < numberOfZeros - number.size(); ++iZero)
    zeros += "0";
  return (zeros + number);
}
