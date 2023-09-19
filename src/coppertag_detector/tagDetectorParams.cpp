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

#include "coppertag/tagDetectorParams.h"
#include "yaml-cpp/yaml.h"

namespace CopperTag {
    DetectorParams::DetectorParams() {
        // not to be implemented
    }

    bool DetectorParams::load_from_yaml(std::string filename) {
        try {
            YAML::Node config = YAML::LoadFile(filename);

            LINE_ERROR_                 = config["detector_params"]["line_error"].as<double>();
            CORNER_SWAY_THRESH_         = config["detector_params"]["corner_sway_thresh"].as<double>();
            EST_CORNER_THRESH_          = config["detector_params"]["est_corner_thresh"].as<double>();
            MAX_DIST_K_                 = config["detector_params"]["max_dist_k"].as<double>();
            MIN_LINE_LEN_               = config["detector_params"]["min_line_len"].as<double>();
            MIN_DP_CONTOUR_NUM_         = config["detector_params"]["min_dp_contour_num"].as<double>();
            CORNER_LINE_DIST_           = config["detector_params"]["corner_line_dist"].as<double>();
            CORNER_LINE_ANGLE_          = config["detector_params"]["corner_line_angle"].as<double>();
            CORNER_REFINE_SCAN_DIST_    = config["detector_params"]["corner_refine_scan_dist"].as<int>();
            CORNER_REFINE_THRESH_       = config["detector_params"]["corner_refine_thresh"].as<double>();
        } catch (const YAML::Exception& e) {
            std::cerr << "Error reading YAML file: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown error occurred while reading YAML file." << std::endl;
        }
        return true;
    }
} // namespace CopperTag