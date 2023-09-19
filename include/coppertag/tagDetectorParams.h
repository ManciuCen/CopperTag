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

#ifndef _COPPER_TAG_DETECTOR_PARAMS_H_
#define _COPPER_TAG_DETECTOR_PARAMS_H_

#include <string>
#include "common/cyber_common_macros.h"

namespace CopperTag {

    class DetectorParams {
      public:
        /**
         * @brief load detector parameters from yaml file
         * 
         * @param filename .yaml file name
         * 
         * @return true: succeed; false: failed
         */
        bool load_from_yaml(std::string filename);

        inline double LINE_ERROR() {
            return LINE_ERROR_;
        }

        inline double CORNER_SWAY_THRESH() {
            return CORNER_SWAY_THRESH_;
        }

        inline double EST_CORNER_THRESH() {
            return EST_CORNER_THRESH_;
        }

        inline double MAX_DIST_K() {
            return MAX_DIST_K_;
        }

        inline double MIN_LINE_LEN() {
            return MIN_LINE_LEN_;
        }

        inline double MIN_DP_CONTOUR_NUM() {
            return MIN_DP_CONTOUR_NUM_;
        }

        inline double CORNER_LINE_DIST() {
            return CORNER_LINE_DIST_;
        }

        inline double CORNER_LINE_ANGLE() {
            return CORNER_LINE_ANGLE_;
        }
        
        inline double CORNER_REFINE_SCAN_DIST() {
            return CORNER_REFINE_SCAN_DIST_;
        }

        inline double CORNER_REFINE_THRESH() {
            return CORNER_REFINE_THRESH_;
        }
      private:
        /**
         * @brief default deconstruct function
         */
        ~DetectorParams() = default;

        double LINE_ERROR_;
                
        double CORNER_SWAY_THRESH_;

        double EST_CORNER_THRESH_;

        double MAX_DIST_K_;
        
        int MIN_LINE_LEN_;

        int MIN_DP_CONTOUR_NUM_;

        double CORNER_LINE_DIST_;

        double CORNER_LINE_ANGLE_;

        int CORNER_REFINE_SCAN_DIST_;

        double CORNER_REFINE_THRESH_;

        // declare current class as a singleton
        DECLARE_SINGLETON(DetectorParams)

        // declare an external operation instance
        #define PARAMS CopperTag::DetectorParams::Instance()

    }; // class DetectorParams

} // namespace CopperTag

#endif // _COPPER_TAG_DETECTOR_PARAMS_H_