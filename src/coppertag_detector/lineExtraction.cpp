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
  
  bool Detector::are_lines_collinear(const CopperLine& line1, const CopperLine& line2) {
    double includedAngle = atan((line1.m - line2.m) / (1 + line1.m * line2.m)) * 180.0 / M_PI;
    // TODO: degree 10 is best?
    if (std::fabs(includedAngle) > 10) {
      return false;
    } else {
      double d1 = distance_of_2point(line1.startPt, line2.startPt);
      double d2 = distance_of_2point(line1.startPt, line2.endPt);
      double d3 = distance_of_2point(line1.endPt, line2.startPt);
      double d4 = distance_of_2point(line1.endPt, line2.endPt);
      if (( d1 < 10) || ( d2 < 10) || (d3 < 10) || (d4 < 10)) {
        return true;
      } else {
        return false;
      }
    }
  }

  void Detector::extract_lines(const std::vector<std::vector<cv::Point>> &edgeSegments,
                               const std::vector<std::vector<int>> &edgeClusters,
                               std::vector<std::vector<CopperLine>> &lines, 
                               std::vector<std::vector<cv::Point>> &convexQuadContours) {
    std::vector<int> segmentLineCount(edgeSegments.size(), 0);
    lines.clear();
    lines.resize(edgeSegments.size());
    for(int i = 0; i < edgeSegments.size(); i++){
      bool closeContour = cv::norm(edgeSegments[i][edgeSegments[i].size()-1] - 
                                    edgeSegments[i][0]) > 2 ? false : true;

      std::vector<cv::Point> dpContour; 
      cv::approxPolyDP(edgeSegments[i], dpContour, PARAMS->LINE_ERROR(), closeContour);
      if (closeContour) {
        if (dpContour.size() == 4 && cv::isContourConvex(dpContour)) {
          convexQuadContours.push_back(dpContour);
        }
      }

      std::vector<CopperLine> lineGroup;
      filter_lines(edgeSegments[i], dpContour, lineGroup, i, closeContour);
  
      if(lineGroup.size() != 0) {
        lines[i] = lineGroup;
      }
    }

    for (int i = 0; i < edgeClusters.size(); ++i) {
      if (edgeClusters[i].size() <= 1) {
        continue;
      }
    
      std::vector<CopperLine> auxLineGroup, finalLineGroup;
      for (int j = 0; j < edgeClusters[i].size(); ++j) {
        if (lines[edgeClusters[i][j]].empty()) {
          continue;
        } else {
          for (int newLineIdx = 0; newLineIdx < lines[edgeClusters[i][j]].size(); ++newLineIdx) {
            bool collinear = false;
            for (auto& existLine : auxLineGroup) {          
              collinear = are_lines_collinear(lines[edgeClusters[i][j]][newLineIdx], existLine);
              if (collinear) {
                break;
              }
            }
            if (collinear) {
              continue;;
            } else {
              auxLineGroup.push_back(lines[edgeClusters[i][j]][newLineIdx]);
            }
          }
        }
      }
      if (auxLineGroup.size() >= 2) {
        // if we get more than 4 lines into one group, we only extract the longest 4 lines
        auto line_compare = [](const CopperLine& line1, const CopperLine& line2) {
          if (line1.len <= line2.len) {
            return false;
          } else {
            return true;
          }
        };
        std::sort(auxLineGroup.begin(), auxLineGroup.end(), line_compare);
        
        // now we extract the longest four lines (or more than 4), we need to sort them in a anticlockwise order
        finalLineGroup.push_back(auxLineGroup[0]);
        int validLineNums = auxLineGroup.size() > 4 ? 4 : auxLineGroup.size();
        int curIdxSaved = 0;
        std::vector<bool> used(validLineNums, false);
        used[0] = true;
        
        for (int times = 1; times < validLineNums; ++times) {
          double minDist = 1000;
          cv::Point2d curEndPoint = finalLineGroup.back().endPt;
          for (int j = 0; j < validLineNums; ++j) {
            if (used[j]) {
              continue;
            } else {
              double dist = distance_of_2point(curEndPoint, auxLineGroup[j].startPt);
              if (dist < minDist) {
                minDist = dist;
                curIdxSaved = j;
              }
            }
          }
          used[curIdxSaved] = true;
          finalLineGroup.push_back(auxLineGroup[curIdxSaved]);
        }

        lines.push_back(finalLineGroup);
      }
    }
  }


  void Detector::filter_lines(std::vector<cv::Point> edgeSegment, 
                              std::vector<cv::Point> dpContour, 
                              std::vector<CopperLine> &lines, 
                              int segmentID, 
                              bool closeContour) {
    // if (dpContour.size() <= 2) return;  
    int maxSize =  closeContour ? dpContour.size() : dpContour.size()-1;
    std::vector<int> dpOriginIdx;
    dp_Index(edgeSegment, dpContour, dpOriginIdx);
    // std::cout << "current dpOriginIdx size is: " << dpOriginIdx.size() << std::endl;
    for (int dpIdx = 0; dpIdx < maxSize; dpIdx++) {
      // prepare dpIdx and dpNext
      int dpNext = (dpIdx + 1) % dpContour.size();
      //dp contour len
      double len = cv::norm(dpContour[dpNext] - dpContour[dpIdx]);
      if(len > PARAMS->MIN_LINE_LEN()){
        cv::Point2d startPt = dpContour[dpIdx];
        int startPtEdgeIdx = dpOriginIdx[dpIdx]; 

        cv::Point2d endPt = dpContour[dpNext];
        int endPtEdgeIdx = dpOriginIdx[dpNext]; 

        // if line too short and do corner refinement might cause trouble; 
        if(len > 3 * PARAMS->CORNER_REFINE_SCAN_DIST()){ 
          line_refinement(edgeSegment, startPtEdgeIdx, startPt, true); 

          line_refinement(edgeSegment, endPtEdgeIdx, endPt, false); 
        }

        std::vector<int>sampleIdx = {(startPtEdgeIdx+2) % int(edgeSegment.size()), 
                                     (startPtEdgeIdx+4) % int(edgeSegment.size()), 
                                     (startPtEdgeIdx+6) % int(edgeSegment.size())};

        std::vector<cv::Point2d> sampledVect;
        sample_points(edgeSegment, sampleIdx, sampledVect);
        correct_direction(startPt, endPt, sampledVect);
        lines.push_back(CopperLine(startPt, endPt, segmentID)); 
                                          //  dpOriginIdx.at(dpIdx), 
                                          //  dpOriginIdx.at(dpNext))
      }

    }
  }


  void Detector::correct_direction(cv::Point2d &startPt,
                                   cv::Point2d &endPt, 
                                   std::vector<cv::Point2d> &sampled_points) {
    cv::Point2d lineDir = endPt - startPt; 
    int c1 = cross(lineDir, sampled_points[0]);
    int c2 = cross(lineDir, sampled_points[1]);
    int c3 = cross(lineDir, sampled_points[2]);

    if(c1 / abs(c1) + c2/abs(c2) + c3/abs(c3) > 0){
      cv::Point2d tmp = startPt; 
      startPt = endPt; 
      endPt = tmp; 
    }  
  }

  void Detector::sample_points(std::vector<cv::Point> &edgeSegment,
                               std::vector<int> &sampleIdx, 
                               std::vector<cv::Point2d> & sampledPoints) {
    cv::Point pt1 = edgeSegment.at(sampleIdx[0]); 
    sampledPoints.push_back(cv::Point2d(gradX_.at<short>(pt1), 
                                        gradY_.at<short>(pt1)));

    cv::Point pt2 = edgeSegment.at(sampleIdx[1]); 
    sampledPoints.push_back(cv::Point2d(gradX_.at<short>(pt2), 
                                        gradY_.at<short>(pt2)));

    cv::Point pt3 = edgeSegment.at(sampleIdx[2]); 
    sampledPoints.push_back(cv::Point2d(gradX_.at<short>(pt3), 
                                        gradY_.at<short>(pt3)));
  }
  

  void Detector::dp_Index(std::vector<cv::Point> &edgeSegment, 
                          std::vector<cv::Point> &dpContour,
                          std::vector<int> & rst) {
    for(int dpIdx = 0; dpIdx < dpContour.size(); dpIdx ++) {
      for(int i = 0; i < edgeSegment.size(); i++) {
        if(edgeSegment[i].x == dpContour[dpIdx].x && 
           edgeSegment[i].y == dpContour[dpIdx].y) {
          rst.push_back(i);
          break;
        }
      }
    }
  }
  
  void Detector::line_refinement(std::vector<cv::Point> &edgeSegments, 
                                 int &idx, 
                                 cv::Point2d &cornerPt, 
                                 bool forward) {
    /**
     * forward: idx . .(possible new cornerPt) . -> 
     * 
     * !forward: <- . (possible new cornerPt) . . idx
     * 
     */
    if(idx >= edgeSegments.size()) {
      DEBUG_ERROR("the input corner index is out of input edgeSegment size");
      return; 
    }

    std::vector<cv::Point> shortSegment;
    int refineRange = PARAMS->CORNER_REFINE_SCAN_DIST() / 1;
    if (forward) {
      int dist = edgeSegments.size() - idx;
      DEBUG_INFO("forward situation: edgeSegments.size() is " << edgeSegments.size()
                 << ", idx is: " << idx << ", dist is: " << dist);
      // Check if reaching end pt

      
      
      if(refineRange > dist) {
        shortSegment = std::vector<cv::Point>(edgeSegments.end() - dist, 
                                              edgeSegments.end()); 
        shortSegment.insert(shortSegment.end(), 
                            edgeSegments.begin(),
                            edgeSegments.begin() + (refineRange - dist)); 
      } else {
        shortSegment = std::vector<cv::Point>(edgeSegments.begin() + idx,
                                              edgeSegments.begin()+idx + refineRange);
      }
    } else {
      DEBUG_INFO("backward situation: edgeSegments.size() is " << edgeSegments.size() << ", idx is: " << idx);
      // Check if reach beginning
      if(refineRange > idx){
        shortSegment = std::vector<cv::Point>(edgeSegments.end() - (refineRange - idx) + 1, 
                                              edgeSegments.end()); 
        shortSegment.insert(shortSegment.end(), 
                            edgeSegments.begin(), 
                            edgeSegments.begin() + idx + 1); 
      }else{
        shortSegment = std::vector<cv::Point>(edgeSegments.begin() + (idx - refineRange) + 1, 
                                              edgeSegments.begin() + idx + 1);
      }
    }

    std::vector<cv::Point2d> shortDP; 
    cv::approxPolyDP(shortSegment, shortDP, PARAMS->CORNER_REFINE_THRESH(), false); 
    if(shortDP.size() <= 2) {
      // all good, nothing need to be done;
      return;
    }

    if(forward){ 
      for(int i = shortSegment.size()-1; i >= 0; i--){
        if(shortDP.at(shortDP.size()-2).x == shortSegment.at(i).x &&
           shortDP.at(shortDP.size()-2).y == shortSegment.at(i).y)
        {
          idx += i; 
          idx = idx % (edgeSegments.size()); 
          cornerPt = shortDP.at(shortDP.size()-2); 
          break;
        }
      }
    }else{
      for(int i = 0; i < shortSegment.size(); i++){
        if(shortDP.at(1).x == shortSegment.at(i).x &&
           shortDP.at(1).y == shortSegment.at(i).y)
        {
          idx += i; 
          idx -= (shortSegment.size()-1); 
          idx = idx % (edgeSegments.size()); 
          cornerPt = shortDP.at(1); 
          break;
        }
      }

    }
  }

} // namespace CopperTag