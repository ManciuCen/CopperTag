/**************************************************************************
 * Copyright 2023 Youibot Robotics Co., Ltd. All Rights Reserved.
 * Contact: wenzhaochen (robotmanciu@gmail.com)
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *************************************************************************/

#include <cmath>

#include "pose_esti/pose_estimation.h"
#include "yaml-cpp/yaml.h"

namespace CopperTag {
    PoseEstimation::PoseEstimation() : isInit_(false){
      //
    }

    PoseEstimation::~PoseEstimation() {
        isInit_ = false;
    }

    void PoseEstimation::load_camera_parameters(CameraParameters& input_params) {
        // not to be implemented
    }

    void PoseEstimation::load_camera_parameters(std::string filename) {
        try {
            YAML::Node config = YAML::LoadFile(filename);

            int width, height;
            float fx, fy, cx, cy;
            float k1, k2, p1, p2, k3;

            width   = config["camera_params"]["width"].as<int>();
            height  = config["camera_params"]["height"].as<int>();
            fx      = config["camera_params"]["fx"].as<float>();
            fy      = config["camera_params"]["fy"].as<float>();
            cx      = config["camera_params"]["cx"].as<float>();
            cy      = config["camera_params"]["cy"].as<float>();
            k1      = config["camera_params"]["k1"].as<float>();
            k2      = config["camera_params"]["k2"].as<float>();
            p1      = config["camera_params"]["p1"].as<float>();
            p2      = config["camera_params"]["p2"].as<float>();
            k3      = config["camera_params"]["k3"].as<float>();

            curCameraParams_.width  = width;
            curCameraParams_.height = height;
            curCameraParams_.intrinsic_matrix.fx = fx;
            curCameraParams_.intrinsic_matrix.fy = fy;
            curCameraParams_.intrinsic_matrix.cx = cx;
            curCameraParams_.intrinsic_matrix.cy = cy;
            curCameraParams_.distortion.k1       = k1;
            curCameraParams_.distortion.k2       = k2;
            curCameraParams_.distortion.p1       = p1;
            curCameraParams_.distortion.p2       = p2;
            curCameraParams_.distortion.k3       = k3;

            cameraMatrix_ = (cv::Mat_<double>(3,3) << fx,0,cx,0,fy,cy,0,0,1);
            distCoeffs_ = (cv::Mat_<double>(1,5) << k1,k2,p1,p2,k3);

            // std::cout << width << " " << height << " "
            //           << fx << " " << fy << " " << cx << " " << cy << " "
            //           << k1 << " " << k2 << " " << p1 << " " << p2 << " " << k3 << std::endl;

        } catch (const YAML::Exception& e) {
            std::cerr << "Error reading YAML file: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown error occurred while reading YAML file." << std::endl;
        }
        isInit_ = true;
    }

    PoseResult PoseEstimation::calculate_pose(bool need_polish,
                                              const std::vector<cv::Point3f>& objPoints,
                                              const std::vector<cv::Point2f>& imgPoints) {
        PoseResult curPoseRes;
        curPoseRes.valid = false;
        if (!isInit_)
        {
            std::cerr << "camera parameters is not initialized" << std::endl;
            return curPoseRes;
        }

        if (objPoints.size() != 4 || imgPoints.size() != 4) {
            std::cerr << "only support 4 elements in current PnP problem" << std::endl;
            return curPoseRes;
        }

        cv::Mat rvec1, tvec1;
        cv::Mat rvec2, tvec2;
        cv::Mat better_rvec, better_tvec;
        float reprojErr1, reprojErr2, reprojErrBetter;

        float size = sqrtf(powf((objPoints[0].x - objPoints[1].x), 2) + 
                           powf((objPoints[0].y - objPoints[1].y), 2));
        ippeSolver_->solveSquare(size, imgPoints, cameraMatrix_, distCoeffs_, rvec1, tvec1, reprojErr1, rvec2, tvec2, reprojErr2);

        // choose the result with the smaller reprojection error
        if (reprojErr1 <= reprojErr2) {
            better_rvec = rvec1;
            better_tvec = tvec1;
            reprojErrBetter = reprojErr1;
        } else {
            better_rvec = rvec2;
            better_tvec = tvec2;
            reprojErrBetter = reprojErr2;
        }

        // polish the result with LM method
        if (need_polish) {
            float err = solvePNP_LM_IMPL<float>(objPoints, imgPoints, cameraMatrix_, distCoeffs_, better_rvec, better_tvec);
            std::cout << "the polish error from LM method is: " << err << std::endl;
        }

        curPoseRes.repj_err = reprojErrBetter;
        curPoseRes.x        = better_tvec.at<float>(0,0);
        curPoseRes.y        = better_tvec.at<float>(0,1);
        curPoseRes.z        = better_tvec.at<float>(0,2);
        curPoseRes.rx       = better_rvec.at<float>(0,0);
        curPoseRes.ry       = better_rvec.at<float>(0,1);
        curPoseRes.rz       = better_rvec.at<float>(0,2);

        return curPoseRes;
    }
} // namespace CopperTag