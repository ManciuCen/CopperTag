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

// three main function:
// 1. get_candidate() -> use corners to form the quad for the first time
// 2. fit_singleton_corner() -> use the singleton corners to form the quad again
// 3. candidate_filter() -> remove some useless candidate by rules

#include "coppertag/tagDetector.h"

namespace CopperTag {

void Detector::get_candidates(std::vector<cv::RotatedRect> ellipses, 
                              std::vector<std::vector<CopperCorner>> &corners, 
                              std::vector<CopperQuad> &allQuads) {
    // we can remove those real small corners here
    for (auto& corner : corners) {
      for (auto it = corner.begin(); it != corner.end();) {
        if (it->l1.len < 20 && it->l2.len < 20) {
          corner.erase(it);
        } else {
          ++it;
        }
      }
    }

    auto corner_group_center = [](std::vector<CopperCorner>& inputCorners) {
      if (inputCorners.empty()) {
        return cv::Point2f(0, 0);
      }
      float x = 0;
      float y = 0;
      for (auto corner : inputCorners) {
        x += corner.loc.x;
        y += corner.loc.y;
      }
      x /= inputCorners.size();
      y /= inputCorners.size();
      return cv::Point2f(x, y);
    };

    std::vector<bool> checkedCornerGroup(corners.size(), false);

    // the quad that can be formed with 3 or 4 corners
    // calcaulate all these first to prevent extra calculations;  
    std::unordered_map<std::pair<int, int>, CopperQuad, pairhash> quad3N4;
    
    for(int cgi = 0; cgi < corners.size(); cgi++) { // cgi <=> cornerGroupIndex
      if (corners.at(cgi).size() < 3) {
        continue;;
      }
      for (int cornerIdx = 0; cornerIdx < corners.at(cgi).size(); cornerIdx++) {
        int i1 = cornerIdx;
        int i2 = (i1 + 1) % corners.at(cgi).size();
        int i3 = (i1 + 2) % corners.at(cgi).size();
        int i4 = (i1 + 3) % corners.at(cgi).size();
        CopperQuad quad;
        bool fix3n4Res = process_3n4_corners(corners.at(cgi), {i1, i2, i3, i4}, quad, true);
        bool fix3n4ResIsValid = valid_quad(quad.cornerLoc);
        
        if (fix3n4Res && fix3n4ResIsValid) {
          DEBUG_INFO("corner group[" << cgi << "] with [" << corners.at(cgi).size() << "] form a valid quad!");
          quad3N4[std::make_pair(cgi, i1)] = quad;
          // TODO: if one of the corner in this corner group can form a quad, we can skip that maybe?
          break;
        } else {
          // TODO: if one quad formed by this corner group (size <= 4) is invalid, we can skip that maybe?
          if (corners.at(cgi).size() <= 4 && !fix3n4ResIsValid) {
            break;
          }
          DEBUG_WARNING("corner group[" << cgi << "] with [" << corners.at(cgi).size() 
                        << "] form quad failed, fix3n4Res[" << fix3n4Res 
                        << "], fix3n4ResIsValid[" << fix3n4ResIsValid << "]");
        }
      }
    }
    
    // traversal all ellipse
    centerRefined_.clear();
    refineEllipseCenter_.clear();
    for (const auto& e : ellipses) {
      centerRefined_.push_back(false);
      refineEllipseCenter_.push_back(cv::Point2d(e.center.y, e.center.x));
    }
    for(int ellipseIdx = 0; ellipseIdx < ellipses.size(); ellipseIdx++){

      /********* display the ellipse currently being processed **********/
#if DISDEBUG
      cv::Mat showCurEllipse;
      cv::cvtColor(imgGray, showCurEllipse, cv::COLOR_GRAY2BGR);
      cv::flip(showCurEllipse, showCurEllipse, 1);
      cv::rotate(showCurEllipse, showCurEllipse, cv::ROTATE_90_COUNTERCLOCKWISE);     
      cv::ellipse(showCurEllipse, ellipses[ellipseIdx], cv::Scalar(0, 0, 255), 1);  
      cv::flip(showCurEllipse, showCurEllipse, 1);
      cv::rotate(showCurEllipse, showCurEllipse, cv::ROTATE_90_COUNTERCLOCKWISE);    
      cv::imshow("showCurEllipse", showCurEllipse);
      cv::waitKey();
#endif

      cv::RotatedRect ellipse = ellipses.at(ellipseIdx); 

      double majorAxis;  
      double minorAxis;  
      if (ellipse.size.width > ellipse.size.height) {
        majorAxis = ellipse.size.width; 
        minorAxis = ellipse.size.height; 
      } else {
        majorAxis = ellipse.size.height; 
        minorAxis = ellipse.size.width; 
      }

      // check if ellipse too small
      if (minorAxis < 30) {
        DEBUG_ERROR("current ellipse majorAxis is: " << majorAxis
                      << " and minorAxis is " << minorAxis 
                      << ", it is too small(threshold 30), skip it");
        continue;
      }
      
      // check if ellipse is too close to the image edge
      double disClose2Edge = std::sqrt(pow(majorAxis / 2, 2) + pow(minorAxis / 2, 2)) * 0.7;
      cv::Point2d center(ellipse.center.y, ellipse.center.x);
      
      if(center.x < disClose2Edge ||
         center.x + disClose2Edge > width_ ||
         center.y < disClose2Edge ||
         center.y + disClose2Edge > height_ ) {
        DEBUG_ERROR("current ellipse disClose2Edge is: " << disClose2Edge
                      << " and the ellipse center is " << center 
                      << ", image width is: " << width_ << ", height: " << height_);
        continue;
      }

      // the maxDist is used to remove those corners are within the ellipse but too close to the center
      double maxDist = majorAxis / PARAMS->MAX_DIST_K();
      // the singleton corners of for ellipse
      std::vector<CopperCorner> ellipseSingleCorner;

      // current ellipse ROI area, we only consider those corners that are locate within this area
      float areaRatio = 0.8;
      float areaMinX = std::max(1., center.x - areaRatio * ellipse.size.width);
      float areaMaxX = std::min(double(width_), center.x + areaRatio * ellipse.size.width);
      float areaMinY = std::max(1., center.y - areaRatio * ellipse.size.height);
      float areaMaxY = std::min(double(height_), center.y + areaRatio * ellipse.size.height);

      // save the quad formed by sameside corners temporary
      std::vector<CopperQuad> samesideQuad;

      // cgIdx = cornerGroupIndex
      for(int cgIdx = 0; cgIdx < corners.size(); cgIdx++) {
        // skip the corner group that are far away from the ellipse area
        cv::Point2f curCornersCenter = corner_group_center(corners[cgIdx]);
        if ((curCornersCenter.x < areaMinX) || (curCornersCenter.x > areaMaxX) ||
            (curCornersCenter.y < areaMinY) || (curCornersCenter.y > areaMaxY)) {
          continue;
        }

        // skip if group is done
        if (checkedCornerGroup.at(cgIdx)) {
          continue;
        }

        std::vector<CopperCorner> curCornerGroup = corners.at(cgIdx); 

        //TODO: maybe we can add some rules to filter the real far corner group quickly

        /********* display all corners in curCornerGroup **********/
#if DISDEBUG
        cv::Mat cornersWithEllipse;
        cv::cvtColor(imgGray, cornersWithEllipse, cv::COLOR_GRAY2BGR);
        for (auto corner : curCornerGroup) {
          draw_corner(cornersWithEllipse, corner, true);
          cv::circle(cornersWithEllipse, curCornersCenter, 3, cv::Scalar(0, 0, 255), -1);
          cv::imshow("cornersWithEllipse", cornersWithEllipse);
          cv::waitKey();
        }
        DEBUG_INFO("current is curCornerGroup[" << cgIdx << "], the size is: " << curCornerGroup.size());
        cv::waitKey();
#endif

        /*********************************************************************************
        * we divided all scenarios into 3 categories for processing
        * 1. cornerGroup.size() = 1 => save it to the ellipseSingleCorner
        *                           => skip it
        * 2. cornerGroup.size() = 2 => use center point to restore the quad
        *                           => save the valid one to the ellipseSingleCorner
        * 3. cornerGroup.size() = 3 => filter to get the best corners
        *                           => size() >= 3, try to form a quad
        *                           => size() = 2, processed as the above mentioned
        *                           => save the valid one to the ellipseSingleCorner
        **********************************************************************************/

        // store the distance between each corner loc and ellipse
        std::vector<double> disToEllipse(curCornerGroup.size(), -100);

        // extract the best corners from the curCornerGroup
        std::vector<CopperCorner> curGoodCornerGroup;

        // the best corners original index in curCornerGroup
        std::vector<int> curGoodCornerIdx;

        for (int cornerIdx = 0; cornerIdx < curCornerGroup.size(); ++cornerIdx) {
          // we don't need those corners that are:
          // 1. outside of the ellipse
          // 2. not facing to center
          // 3. too close to center
          double dis2Ellipse = point_to_ellipse_dist(ellipse, curCornerGroup[cornerIdx].loc);
          // the distance threshold for point to center should be the 1/3 of the length of ellipse diagonal
          double tooCloseThres = std::sqrt(pow(majorAxis / 2, 2) + pow(minorAxis / 2, 2)) * 0.33;
          double dis2Center = distance_of_2point(curCornerGroup[cornerIdx].loc, center);
          bool isFacing = face_point(center, curCornerGroup[cornerIdx]);

          DEBUG_INFO("curCornerGroup[" << cgIdx << "]_corner[" << curCornerGroup[cornerIdx].loc
                      << "]: l1[" << curCornerGroup[cornerIdx].l1.len << "], l2[" << curCornerGroup[cornerIdx].l2.len
                      << "], minorAxis is[" << minorAxis << "], dis2Ellipse[" << dis2Ellipse << "], maxDist [" << maxDist  
                      << "], dis2Center[" << dis2Center << "]" << ", center[" << center << "], tooCloseThres["
                      << tooCloseThres << "], isFacing[" << isFacing << "]");

          // condition-1: outside of ellipse or within but too far away from the ellipse border
          if (dis2Ellipse < 0.1 || dis2Ellipse >= maxDist) {
            continue;
          // condition-2: is not facing to the center
          } else if (!isFacing) {
            continue;
          // condition-3: small and close to the center
          } else if (std::min(curCornerGroup[cornerIdx].l1.len, curCornerGroup[cornerIdx].l2.len) < minorAxis * 0.5 && dis2Center <= tooCloseThres) {
            continue;
          }
          
          else {
            curGoodCornerGroup.push_back(curCornerGroup[cornerIdx]);
            curGoodCornerIdx.push_back(cornerIdx);
            disToEllipse.push_back(dis2Ellipse);
          }         
        }

        DEBUG_INFO("current good corner group size is: " << curGoodCornerGroup.size());

        // if we can extract at least one corner from curCornerGroup, that means this group must be processed
        // if (!curGoodCornerGroup.empty()) {
        //   checkedCornerGroup[cgIdx] = true;
        // }

        /****************** display the current good corner group ******************/
#if DISDEBUG
        cv::Mat curGoodCornerGroupImg;
        cv::cvtColor(imgGray, curGoodCornerGroupImg, cv::COLOR_GRAY2BGR);
        for (auto goodCorner: curGoodCornerGroup) {
          draw_corner(curGoodCornerGroupImg, goodCorner, true);
          cv::imshow("curGoodCornerGroupImg", curGoodCornerGroupImg);
          cv::waitKey();
        }
#endif

        // indicate the state of each corner in curGoodCornerGroup
        // state-(0): initialized and standby
        // state-(-1): invalid corner(may be too far, too small, or included angle too big)
        // state-(1): valid and used to form a quad
        // state-(2): valid but can't form a quad, ready to save to ellipseSingleCorner
        std::vector<int> checkedCorner(curGoodCornerGroup.size(), 0);

        CopperQuad curQuad;
        // for size over 4, we need to figure it out what would happen?
        if (curGoodCornerGroup.size() > 4) {
          std::fill(checkedCorner.begin(), checkedCorner.end(), 0);
          DEBUG_WARNING("there are still " << curGoodCornerGroup.size() << " corners in curGoodCornerGroup"
                        << ", hard to deal with this suitation, let fit_singleton_corners() deal with it, quit!");
        // for size 3-4, we check whether they form a quad before, if not, form it now
        } else if (curGoodCornerGroup.size() <= 4 && curGoodCornerGroup.size() > 2) {
          // we just need to check once in quad3N4 map
          auto quadIterator = quad3N4.find(std::make_pair(cgIdx, curGoodCornerIdx[0]));
          if (quadIterator != quad3N4.end()) {
            if (near_ellipse(ellipse, quadIterator->second.cornerLoc) &&
                corner_center_inline(ellipse, quadIterator->second.cornerLoc)) {
              curQuad = quadIterator->second;
              curQuad.ellipseId = ellipseIdx;
              allQuads.push_back(curQuad);
              quad3N4.erase(quadIterator);
              std::fill(checkedCorner.begin(), checkedCorner.end(), 1);
              DEBUG_INFO("corners[" << cgIdx << "]_process[3-1]: we can find the cornerGroup["
                          << cgIdx << "]_corner[" << curGoodCornerIdx[0] << "] had formed a quad before");
            }
          // if we can't find any quad in quad3N4 map, may be we can try once again with only best corners
          } else {
            int i1 = 0;
            int i2 = (i1 + 1) % curGoodCornerGroup.size();
            int i3 = (i1 + 2) % curGoodCornerGroup.size();
            int i4 = (i1 + 3) % curGoodCornerGroup.size();
            bool fix3n4Res = process_3n4_corners(curGoodCornerGroup, {i1, i2, i3, i4}, curQuad, true, ellipse, true);
            if (fix3n4Res) {
              curQuad.ellipseId = ellipseIdx;
              allQuads.push_back(curQuad);
              std::fill(checkedCorner.begin(), checkedCorner.end(), 1);
              DEBUG_INFO("corners[" << cgIdx << "]_process[3-2]: the corner[" << curGoodCornerIdx[0]
                         << "] and its memeber can form a valid quad");
            }
          }
        // for size-2 suitation: we use process_2_sameside_corners() to check whether they can form a quad
        } else if (curGoodCornerGroup.size() == 2) {
          if (process_2_sameside_corners(ellipse, ellipseIdx, curGoodCornerGroup, curQuad)) {
            allQuads.push_back(curQuad);
            samesideQuad.push_back(curQuad);
            std::fill(checkedCorner.begin(), checkedCorner.end(), 1);
            DEBUG_INFO("corners[" << cgIdx << "]_process[2-1]: corner["
                           << curGoodCornerGroup.at(0).loc << "] and corner["
                           << curGoodCornerGroup.at(1).loc << "] form a quad!");
          }
        }

        for (int i = 0; i < curGoodCornerGroup.size(); ++i) {
          // state = 0 means this corner was not used to form a quad successfully
          if (checkedCorner[i] == 0) {
            bool fix1Res = process_1_corners(ellipse, curGoodCornerGroup.at(i), ellipseSingleCorner);
            DEBUG_INFO("corners[" << cgIdx << "]_process[1-1]: corner["
                           << curGoodCornerGroup.at(i).loc << "] are try to save to the ellipseSingleCorner"
                           << ", the result is: " << fix1Res);
          }
        }
      }

      // merge the quad from samesideQuad to a new quad
      // TODO: size = 2 is a good condition?
      if (samesideQuad.size() == 2) {
        std::vector<cv::Point2d> tmpCorners({samesideQuad[0].cornerLoc[0],
                                             samesideQuad[0].cornerLoc[1],
                                             samesideQuad[1].cornerLoc[0],
                                             samesideQuad[1].cornerLoc[1]});
        DEBUG_INFO("detect 2 sameside quad, try to merge it");
        CopperQuad quad(tmpCorners, std::vector<bool>({1, 1, 1, 1}), ellipseIdx);
        allQuads.push_back(quad);
#if DISDEBUG
        cv::Mat samesideMergeImg;
        cv::cvtColor(imgGray, samesideMergeImg, cv::COLOR_GRAY2BGR);
        draw_quad(samesideMergeImg, quad);
        cv::imshow("samesideMergeImg", samesideMergeImg);
        cv::waitKey();
#endif
      }

      /********* display all single corners in current ellipse **********/
#if DISDEBUG
      cv::Mat singleCornersInEllipse;
      cv::cvtColor(imgGray, singleCornersInEllipse, cv::COLOR_GRAY2BGR);
      for (auto corner : ellipseSingleCorner) {
        //draw_corner(imgCandidateTest, c, true);
        cv::circle(singleCornersInEllipse, cv::Point(corner.loc), 2, cv::Scalar(0, 0, 255), -1);
        cv::imshow("singleCornersInEllipse", singleCornersInEllipse);
        DEBUG_INFO("the single corner[" << corner.loc << "] l1_len is: " << corner.l1.len
          << ", l2_len is: " << corner.l2.len << ", l1-l2 angle is: " << line_angle_diff(corner.l1.angle, corner.l2.angle));
        cv::waitKey();
      }
#endif
      
      bool singleRes = fit_singleton_corners(ellipse, ellipseIdx, ellipseSingleCorner, allQuads);

      DEBUG_INFO("the singleton corners merge result of ellipse[" << ellipseIdx << "] is: " << singleRes);

    } // traversal all ellipses
    
    // for those quads without any ellipse nearby, we still save it as a candidate
    for(auto quadIterator : quad3N4){
      allQuads.push_back(quadIterator.second); 
    }
  } // get_candidate()

  bool Detector::fit_singleton_corners(const cv::RotatedRect& ellipse,
                                       int curEllipseID,
                                       std::vector<CopperCorner> ellipseCorner, 
                                       std::vector<CopperQuad> &allQuads) {
    // a helper function to find the most outer points from the input ellipseCorner
    auto reorder_corners = [&]() {
      std::vector<CopperCorner> outputPoints;
      
      // if (ellipseCorner.size() <= 4) {
      //   return outputPoints;
      // }

      std::vector<cv::Point> inputPoints;
      for (const auto& corner : ellipseCorner) {
        inputPoints.push_back(corner.loc);
      }

      // find convex hull
      std::vector<int> convexHull;
      cv::convexHull(inputPoints, convexHull, true);

      // for (int i = 0; i < convexHull.size(); ++i) {
      //   std::cout << convexHull[i] << ", loc is:  " << inputPoints[convexHull[i]] << std::endl;;
      // }
      // std::cout << std::endl;

      int hullcount = (int)convexHull.size();
      int currentIdx = 0;
      int nextIdx = (currentIdx + 1) % hullcount;
      int nextNextIdx = (nextIdx + 1) % hullcount;
      bool init = true;

      // TODO: remove this damn shit while
      while(currentIdx != 0 || init) {
        init = false;

        outputPoints.push_back(ellipseCorner[convexHull[currentIdx]]);
        if (nextIdx == 0) {
          break;
        }
        
        while (strict_in_line(inputPoints[convexHull[currentIdx]], inputPoints[convexHull[nextIdx]], inputPoints[convexHull[nextNextIdx]], 100)) {
          // DEBUG_INFO("they are in_line, update the nextIdx and nextNextIdx");
          nextIdx = nextNextIdx;
          nextNextIdx = (nextNextIdx + 1) % hullcount;
          if (nextIdx == 0) {
            break;
          }
        }
        currentIdx = nextIdx;
        nextIdx = nextNextIdx;
        nextNextIdx = (nextIdx + 1) % hullcount;
        DEBUG_INFO("they are not in_line, update the currentIdx to: " << currentIdx <<  ", nextIdx and nextNextIdx");
      }

      return outputPoints;
    };

    // a helper function to check whether there are some singleton corners can replace the fake corners in newest quad
    auto replace_fake_corners = [&](int closeFactor = 15) {
      if (allQuads.empty()) {
        return -1;
      } else {
        if (curEllipseID != allQuads.back().ellipseId) {
          return -1;
        } else {
          float closeThresh = allQuads.back().averageSideLen / closeFactor;
          std::vector<bool> hadUsed(ellipseCorner.size(), false);
          for (size_t i = 0; i < 4; ++i) {
            // we only compare those corners that are restore by rules
            if (!allQuads.back().foundCorners[i]) {
              int bestOneIdx = -1;
              float bestDis = 1000;
              for (size_t j = 0; j < ellipseCorner.size(); ++j) {
                if (!hadUsed[j]) {
                  double dis = distance_of_2point(allQuads.back().cornerLoc[i], ellipseCorner[j].loc);
                  if (dis < closeThresh && dis < bestDis) {
                    // we need to check if this two corner angle are similar
                    int iPrev, iNext;
                    if (i == 0) {
                      iPrev = 3;
                    } else {
                      iPrev = i - 1;
                    }
                    iNext = (i + 1) % 4;
                    CopperLine templine1(allQuads.back().cornerLoc[iPrev], allQuads.back().cornerLoc[i]);
                    double anglel1diff = line_angle_diff(templine1.angle, ellipseCorner[j].l1.angle);
                    CopperLine templine2(allQuads.back().cornerLoc[i], allQuads.back().cornerLoc[iNext]);
                    double anglel2diff = line_angle_diff(templine2.angle, ellipseCorner[j].l2.angle);
                    DEBUG_INFO("this two corner is close enough, and the angle_l1_diff[" << anglel1diff
                               << "], angle_l2_diff[" << anglel2diff << "]");
                    if (std::max(anglel1diff, anglel2diff) < 22.5) {
                      bestDis = dis;
                      bestOneIdx = j;
                    } 
                  }
                }
              }
              if (bestOneIdx != -1 && bestDis < closeThresh) {
                allQuads.back().cornerLoc[i] = ellipseCorner[bestOneIdx].loc;
                hadUsed[bestOneIdx] = true;
                DEBUG_INFO("we use singleton corner[" << ellipseCorner[bestOneIdx].loc << "] to replace the fake corner["
                           << allQuads.back().cornerLoc[i] << "]");
              }
            }
          }
          int usedCornerNums = 0;
          for (auto res: hadUsed) {
            if (res)
              ++usedCornerNums;
          }
          return usedCornerNums;
        }
      }
    };
    
    DEBUG_INFO("Ready to do the fit_singleton_corners() process! the ellipseCorner size is: " << ellipseCorner.size());

    // if we have only singleton corner, we check if this corner can group with the allQuads.back() quad in replace_fake_corners
    if (ellipseCorner.size() == 1) {
      if (allQuads.empty()) {
        DEBUG_WARNING("singleton[1-0]: there are only one singleton corner and no existed quad, quit!");
        return false;
      }
    }

    // if we have two singleton corners, they are probably "same_side" or "diagonal"
    if (ellipseCorner.size() == 2) {
      // we check diagonal situation first
      std::vector<CopperQuad> diagonalQuads;
      bool diagonalRes = process_2_diagonal_corners(ellipse, curEllipseID, ellipseCorner, diagonalQuads);
      if (diagonalRes) {
        for (auto& quad : diagonalQuads) {
          allQuads.push_back(quad);
        }
        return true;
      }

      // if diagonal failed, we check same_side situation
      CopperQuad quad;
      bool samesideRes = process_2_sameside_corners(ellipse, curEllipseID, ellipseCorner, quad);
      if (samesideRes) {
        allQuads.push_back(quad);
        return true;
      }

      DEBUG_WARNING("we can't use these two singleton corners to form a valid quad");

      // if all diagonal or sameside failed, we can check if these corners can be used to replace fake corners
      int replaceCornerNums = replace_fake_corners(5);
      DEBUG_INFO("try to use replace_fake_corners() after process_2_diagonal_corners(),"
                  << " the number of replaced corners is: " << replaceCornerNums);
      if (replaceCornerNums <= 0) {
        return false;
      } else {
        return true;
      }
    }
    
    // if we have more than three singleton corners, reorder it and then filter the most outer four corners
    if (ellipseCorner.size() >= 3) {
      // reorder the points and discard most of the inner points
      std::vector<CopperCorner> newSingleCorners = reorder_corners();

      /**************** display the new singleton corners after ordering ***************/
#if DISDEBUG
      cv::Mat newSingleCornersImg;
      cv::cvtColor(imgGray, newSingleCornersImg, cv::COLOR_GRAY2BGR);      
      for (auto corner : newSingleCorners) {
        draw_corner(newSingleCornersImg, corner, true);
        cv::imshow("newSingleCornersImg", newSingleCornersImg);
        cv::waitKey();
      }
#endif

      // if we form at least one quad using these new singleton corners, we assume the result is true
      bool formOneQuadAtLeast = false;

      // these corners are used to process as a 3n4 group first
      for (int cornerIdx = 0; cornerIdx < newSingleCorners.size(); ++cornerIdx) {
        int i1 = cornerIdx;
        int i2 = (i1 + 1) % newSingleCorners.size();
        int i3 = (i1 + 2) % newSingleCorners.size();
        int i4 = (i1 + 3) % newSingleCorners.size();
        CopperQuad quad;
        bool p3n4Res = process_3n4_corners(newSingleCorners, {i1, i2, i3, i4}, quad, false, ellipse, true);
        if (p3n4Res) {
          //formOneQuadAtLeast = true;
          allQuads.push_back(quad);
        }
      }

      // if all 3n4 group are failed, try to process as diagonal group and sameside group
      if (!formOneQuadAtLeast) {
        std::vector<CopperQuad> diagonalQuads;
        // diagonal situation
        bool diagonalRes = process_2_diagonal_corners(ellipse, curEllipseID, newSingleCorners, diagonalQuads);
        if (diagonalRes) {
          formOneQuadAtLeast = true;
          for (auto& quad : diagonalQuads) {
            allQuads.push_back(quad);
            int replaceCornerNums = replace_fake_corners(15);
            DEBUG_INFO("try to use replace_fake_corners() after process_2_diagonal_corners(),"
                       << " the number of replaced corners is: " << replaceCornerNums);
          }
        }

        // sameside group situation
        std::vector<CopperQuad> samesideQuads;
        bool samesideGroupRes = process_2_sameside_corners_group(ellipse, curEllipseID, newSingleCorners, samesideQuads);
        if (samesideGroupRes && samesideQuads.size() != 0) {
          formOneQuadAtLeast = true;
          for (auto& quad : samesideQuads) {
            allQuads.push_back(quad);
          }
        }
      }

      return formOneQuadAtLeast;
    }

    int replaceCornerNums = replace_fake_corners();
    if (replaceCornerNums <= 0) {
      DEBUG_WARNING("replace_fake_corners() method failed!");
      return false;
    } else {
      DEBUG_INFO("replace_fake_corners() succeed! the number of replaced corners is: " << replaceCornerNums);
      return true;
    }
  } // fit_singleton_corners()

  void Detector::candidates_filter(std::vector<CopperQuad> &inputQuads,
                                   std::vector<CopperQuad> &outputQuads) {
    auto quaddd_is_close = [](CopperQuad& quad1, CopperQuad& quad2) {
      int matchedCornerCount = 0;
      if (quad1.cornerLoc.size() != 4 || quad2.cornerLoc.size() != 4) {
        return false;
      }
      std::vector<bool> matched(4, false);
      double sizeLength = distance_of_2point(quad1.cornerLoc[0], quad1.cornerLoc[1]);
      for (int i = 0; i < 4; ++i) {
        for (int j = 0, k = 0; j < 4; ++j) {
          if (!matched[k] && 
              distance_of_2point(quad1.cornerLoc[i], quad2.cornerLoc[k]) < sizeLength / 16.f) {
            matched[k] = true;
            ++matchedCornerCount;
            break;
          }
          k = (k + 1) % 4;
        }
      }
      if (matchedCornerCount >= 3) {
        return true;
      } else {
        return false;
      }
    };

    if (inputQuads.empty()) {
      return;
    }

    std::vector<bool> toBeRemove(inputQuads.size(), false);
    outputQuads.clear();

    // remove bad quads according to three criteria:
    // 1. their corners distance (only two quads close enough will used for comparison)
    // 2. their validCornerCount (more physical corners is better)
    // 3. we prefer small perimeter quads because adaptivethreshold method 
    //    will create both inner and outer quads while the inner one is more precise (deleted)
    // TODO: maybe we should choose the bigger perimeter one...
    for (int i = 0; i < inputQuads.size(); ++i) {
      for (int j = i + 1; j < inputQuads.size(); ++j) {
        if (!toBeRemove[j] && quaddd_is_close(inputQuads[i], inputQuads[j])) {
          if (inputQuads[i].validCornerCount < inputQuads[j].validCornerCount) {
            toBeRemove[i] = true;
          } else if (inputQuads[i].validCornerCount > inputQuads[j].validCornerCount) {
            toBeRemove[j] = true;
          } else if (inputQuads[i].validCornerCount == inputQuads[j].validCornerCount) {
            // if (inputQuads[i].perimeter >= inputQuads[j].perimeter) {
            //   //TODO: update delete object here
            //   // delete small one: tobeRemove[j] = true; delete big one: tobeRemove[i] = true
            //   toBeRemove[j] = true;
            // } else {
            //   toBeRemove[i] = true;
            // }
          }
        }
      }
    }
    
    // push the good quads to the output vector
    for (int i = 0; i < inputQuads.size(); ++i) {
      if (!toBeRemove[i]) {
        outputQuads.push_back(inputQuads[i]);
      }
    }

    // corner upsample and subpixel refinement
    // default the image was resized to half, so the factor is 2
    auto change_point2d_to_point2f = [](const std::vector<cv::Point2d>& in) {
      std::vector<cv::Point2f> out;
      for (auto corner : in) {
        out.push_back(cv::Point2f(corner.x, corner.y));
      }
      return out;
    };

    auto update_quad_corner = [](const std::vector<cv::Point2f>& newCorners, CopperQuad& inputQuad) {
      inputQuad.cornerLoc.clear();
      for (auto& corner : newCorners) {
        inputQuad.cornerLoc.push_back(corner);
      }
    };

    // the default upsamleFactor should be 2, correspond to the default downsampleFactor
    int upsampleFactor = 2;
    // the cornerSubPix method searching windows size should up to the quad side length
    // halfSize = quad.averageSideLen / 13;
    int halfSize;
    
    for (auto& quad : outputQuads) {
      // determine the current searching window size
      halfSize = std::max(3, int(quad.averageSideLen / 15));
      // upsample corner from current image to original image
      for (auto& corner : quad.cornerLoc) {
        corner *= upsampleFactor;
      }

      // chaneg corner type from point2d to point2f for corberSubPix() method
      std::vector<cv::Point2f> refineCorner = change_point2d_to_point2f(quad.cornerLoc);
      cv::cornerSubPix(imgOrigin_, refineCorner, cv::Size(halfSize, halfSize), cv::Size(-1,-1),
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001 ));
      // use refine new corners replace the orginal corners
      update_quad_corner(refineCorner, quad);
    }    
  } // candidates_filter()

} // namespace CopperTag