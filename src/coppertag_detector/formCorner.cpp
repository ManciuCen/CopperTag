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

#include "coppertag/tagDetector.h"

namespace CopperTag {
  void Detector::find_corners(std::vector<std::vector<CopperLine>> lines, 
                              std::vector<std::vector<CopperCorner>> &corners) {
    std::vector<CopperCorner> cornerGroup; 
    for(int i = 0; i < lines.size(); i++) { // loop all line groups

      int lineGroupSize = lines.at(i).size();
      if (lineGroupSize < 2) {
        continue;
      }

      for(int j = 0; j < (lineGroupSize == 2 ? 1 : lineGroupSize); j++) {
        int curIdx = j, nextIdx = (j + 1) % lineGroupSize;
        // check if line angle good

        double angle_diff = line_angle_diff(lines.at(i).at(curIdx).angle, 
                                            lines.at(i).at(nextIdx).angle);
        if(angle_diff < PARAMS->CORNER_LINE_ANGLE()) {
          continue;
        }

        double d1 = cv::norm(lines.at(i).at(curIdx).endPt - lines.at(i).at(nextIdx).startPt);
        double d2 = cv::norm(lines.at(i).at(nextIdx).endPt - lines.at(i).at(curIdx).startPt);
        
        if(d1 <= d2 && d1 < PARAMS->CORNER_LINE_DIST()) {
          if(cross(cv::Point2d(lines.at(i).at(curIdx).endPt - lines.at(i).at(curIdx).startPt), 
                   cv::Point2d(lines.at(i).at(nextIdx).endPt - lines.at(i).at(nextIdx).startPt)) > 0) {
            continue;
          }
          // form corner; 
          cv::Point2d cornerLoc; 
          if (intersect(lines.at(i).at(curIdx), lines.at(i).at(nextIdx), cornerLoc)) {
            cornerGroup.push_back(CopperCorner(cornerLoc, 
                                               lines.at(i).at(curIdx), 
                                               lines.at(i).at(nextIdx), 
                                               lines.at(i).at(curIdx).segmentID)); 
          }
        } else if (d2 < d1 && d2 < PARAMS->CORNER_LINE_DIST()) {
          // check corner orientation;
          if(cross(cv::Point2d(lines.at(i).at(nextIdx).endPt - lines.at(i).at(nextIdx).startPt), 
                   cv::Point2d(lines.at(i).at(curIdx).endPt - lines.at(i).at(curIdx).startPt)) > 0) {
            continue;
          }
          // form corner; 
          cv::Point2d cornerLoc; 
          if (intersect(lines.at(i).at(nextIdx), lines.at(i).at(curIdx), cornerLoc)) {
            cornerGroup.push_back(CopperCorner(cornerLoc, 
                                               lines.at(i).at(nextIdx), 
                                               lines.at(i).at(curIdx), 
                                               lines.at(i).at(nextIdx).segmentID)); 
          }
        }      
      }

      if (cornerGroup.size() > 0){
        corners.push_back(cornerGroup);
        cornerGroup.clear();
      }
    }
  }
  
  double Detector::line_angle_diff(double l1Angle, double l2Angle) {
    double dir_diff = abs(l2Angle - l1Angle);
    double oppo_diff = 180 - dir_diff; 

    return std::min(dir_diff, oppo_diff);
  }

  bool Detector::intersect(CopperLine line1, CopperLine line2, cv::Point2d &rst) {
      // parallel 
      if (line1.m == line2.m || 
          (abs(line1.m) == INFINITY && 
          abs(line2.m) == INFINITY)){
          rst = cv::Point2d(0, 0);
          return false; 
      }
      
      double x, y;   
      // if 1 verticle 
      if (abs(line1.m) == INFINITY){
          x = line1.startPt.x; 
          y = line2.m * x + line2.b; 
      }else if (abs(line2.m) == INFINITY){
          x = line2.startPt.x; 
          y = line1.m * x + line1.b; 
      }else{
          // no verticle
          x = (line2.b - line1.b) / (line1.m - line2.m);
          y = line1.m * x + line1.b;
      }
      
      rst = cv::Point2d(x, y);
      return true; 
  }

} // namespace CopperTag