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

#include <algorithm>

#include "coppertag/tagDetector.h"

namespace CopperTag {
  bool Detector::process_1_corners(const cv::RotatedRect& ellipse, CopperCorner corner,
                                   std::vector<CopperCorner> &ellipseCorner) {
    // TODO: maybe this little helper function can integrated into line_angle_diff
    auto strict_line_angle_diff = [](const CopperLine& l1, const CopperLine& l2) {
      cv::Point vector1 = l1.startPt - l1.endPt;
      cv::Point vector2 = l2.endPt - l2.startPt;

      double dotProduct = vector1.x * vector2.x + vector1.y * vector2.y;
      double magnitude1 = cv::norm(vector1);
      double magnitude2 = cv::norm(vector2);

      double angle = std::acos(dotProduct / (magnitude1 * magnitude2));

      return angle / M_PI * 180.0;
    };

    // step-1. check the included angle between l1 and l2
    // TODO: the included angle calculation can be done by CopperCorner constructor
    // for those big corner (has long line), we can ignore its included angle
    double minorAxis = std::min(ellipse.size.width, ellipse.size.height);
    bool isBigCorner = std::min(corner.l1.len, corner.l2.len) > minorAxis * 0.5;

    if (!isBigCorner) {
      double includedAngle = line_angle_diff(corner.l1.angle, corner.l2.angle);
      double includeAngle2 = strict_line_angle_diff(corner.l1, corner.l2);
      double includeAngleGap = std::fabs(includeAngle2 - includedAngle);
      if (includedAngle < 30 || includeAngleGap > 70) {
        DEBUG_WARNING("corner[" << corner.loc << "] included angle is invalid. the angle is: "
                      << includedAngle << ", and the angle2 is: " << includeAngle2);
        return false;
      }

      // step-2. check if this corner is a really small corner (with two short lines)
      // TODO: maybe 20 is not the best parameters here
      if (corner.l1.len < 20 && corner.l2.len < 20) {
        DEBUG_WARNING("corner[" << corner.loc << "] two lines are too short");
        return false;
      }
    }
    
    // step-3. check if this corner is facing to the ellipse center
    int isFacingCenter = false;
    cv::Point2d ellipseCenter(ellipse.center.y, ellipse.center.x);
    isFacingCenter = face_point(ellipseCenter, corner);
    if (!isFacingCenter) {
      DEBUG_WARNING("corner[" << corner.loc << "] is not facing to the ellipse center");
      return false;
    }

    // step-4. check if this corner is too close to the ellipse center
    if (!isBigCorner) {
      double tooCloseThres = std::sqrt(pow(ellipse.size.width / 2, 2) + pow(ellipse.size.height / 2, 2)) * 0.33;
      double currentDis = distance_of_2point(ellipseCenter, corner.loc);
      if (currentDis < tooCloseThres) {
        DEBUG_WARNING("corner[" << corner.loc << "] is too close the the ellipse center");
        return false;
      }
    }
  
    // step-5. check if a similar point already exists
    // TODO: maybe 5 is not the best parameters here
    int alreadyExist = false;
    for (auto existCorner : ellipseCorner) {
      if (distance_of_2point(corner.loc, existCorner.loc) < 5) {
        alreadyExist = true;
        break;
      }
    }
    if (alreadyExist) {
      DEBUG_WARNING("corner[" << corner.loc << "] find a similarity one in existCorners");
      return false;
    }

    if (isFacingCenter && !alreadyExist) {
      ellipseCorner.push_back(corner);
      return true;
    } else {
      DEBUG_WARNING("corner[" << corner.loc << "] process_1_corner failed, isFacingCenter["
                    << isFacingCenter << "], alreadyExist[" << alreadyExist << "]");
      return false;
    }
  }

  bool Detector::process_2_diagonal_corners(const cv::RotatedRect& ellipse,
                                            int curEllipseID,
                                            const std::vector<CopperCorner>& corners, 
                                            std::vector<CopperQuad> &quads) {
    // step-0: the input corners should include at least two corners
    if (corners.size() < 2) {
      return false;
    }
    
    // step-1. refine the ellipse center if it's never been processed
    cv::Point2d ellipseCenter;
    if (!centerRefined_[curEllipseID]) {
      ellipseCenter = refine_center(ellipse, InnerCircleMode::incomplete);
      DEBUG_INFO("refine_center: before[" << cv::Point2f(ellipse.center.y, ellipse.center.x) << "]" << ", after[" << ellipseCenter << "]");
      centerRefined_[curEllipseID] = true;
      refineEllipseCenter_[curEllipseID] = ellipseCenter;
      /**************** display the original center and refinement center ***************/
#if DISDEBUG
      cv::Mat diagonalImg;
      cv::cvtColor(imgGray, diagonalImg, cv::COLOR_GRAY2BGR);
      cv::circle(diagonalImg, cv::Point(ellipse.center), 2, cv::Scalar(0, 0, 255), -1);
      cv::imshow("diagonalImg", diagonalImg);
      cv::waitKey();
      cv::circle(diagonalImg, cv::Point(ellipseCenter), 2, cv::Scalar(0, 255, 0), -1);
      cv::imshow("diagonalImg", diagonalImg);
      cv::waitKey();
#endif
    } else {
      ellipseCenter = refineEllipseCenter_[curEllipseID];
      DEBUG_INFO("we don't need refine_center in this ellipse");
    }

    // step-2. group the input corners to facing group
    // each facing group has two corners and they are facing to each other
    std::vector<std::pair<int, int>> facingGroups;
    for (int i  = 0; i < corners.size(); ++i) {
      for (int j = i + 1; j < corners.size(); ++j) {
        bool isFacing = facing(corners[i], corners[j]);
        bool isInLine = in_line(ellipseCenter, corners[i].loc, corners[j].loc);
        DEBUG_INFO("the corners[" << i << "] and [" << j << "]: facing[" << isFacing << "], inLine[" << isInLine << "]");
        if (isFacing && isInLine) {
          facingGroups.push_back(std::make_pair(i, j));
        }
      }
    }

    // step-3. process each facing corner pair, check if they can form a quad
    quads.clear();
    for (int i = 0; i < facingGroups.size(); ++i) {
      DEBUG_INFO("current working facing pair is: [" << facingGroups[i].first << "]-[" <<facingGroups[i].second << "]");
      // step-3.1 calculate the two intersection point of these two corners
      cv::Point2d estC1Loc, estC3Loc;
      bool estC1LocRes = intersect(corners[facingGroups[i].first].l2, corners[facingGroups[i].second].l1, estC1Loc);
      if (!estC1LocRes) {
        DEBUG_WARNING("estimation c1 location failed!");
        continue;
      }
      bool estC3LocRes = intersect(corners[facingGroups[i].first].l1, corners[facingGroups[i].second].l2, estC3Loc);
      if (!estC3LocRes) {
        DEBUG_WARNING("estimation c3 location failed!");
        continue;
      }
      // step-3.2 check if the estimation c1 and c3 point are collinear with ellipse center
      bool c1c3InlineCenter = corner_center_inline(ellipse, {estC1Loc, estC3Loc});
      if (!c1c3InlineCenter) {
        DEBUG_WARNING("estimation c1 and c3 are not collinear with ellipse center");
        continue;
      }
      // step-3.3 check if the formed quad is close to the ellipse
      std::vector<cv::Point2d> quadCorners({corners[facingGroups[i].first].loc, estC1Loc,
                                            corners[facingGroups[i].second].loc, estC3Loc});
      bool isNearEllipse = near_ellipse(ellipse, quadCorners, false);
      if (!isNearEllipse) {
        DEBUG_WARNING("the formed quad is not close to current ellipse!");
        continue;
      }

      quads.push_back(CopperQuad(quadCorners, std::vector<bool>({1, 0, 1, 0}), curEllipseID));
    }
    
    if (quads.empty()) {
      return false;
    } else {
      DEBUG_INFO("form quad in diagonal situation succeed!");
      return true;
    }
  }

  bool Detector::process_2_sameside_corners(const cv::RotatedRect& ellipse,
                                            int curEllipseID,
                                            std::vector<CopperCorner> corners,
                                            CopperQuad &quad) {
    // step-0: the input corners should include at least two corners
    if (corners.size() < 2) {
      return false;
    }
    
    // step-1. check if the two corners are both facing to the ellipse center
    cv::Point2d ellipseCenter(ellipse.center.y, ellipse.center.x);
    bool c0Facing = face_point(ellipseCenter, corners[0]);
    if (!c0Facing) {
      DEBUG_WARNING("corner[" << corners[0].loc << "] is not facing to the ellipse center");
      return false;
    }
    bool c1Facing = face_point(ellipseCenter, corners[1]);
    if (!c1Facing) {
      DEBUG_WARNING("corner[" << corners[1].loc << "] is not facing to the ellipse center");
      return false;
    }

    // step-2. check if the corresponding line of two corners are almost parallel:
    // example: corner0_l2 // corner1_l1; corner0_l1 // corner1_l2
    double line0Angle = line_angle_diff(corners.at(0).l2.angle, corners.at(1).l1.angle);
    if (line0Angle > PARAMS->CORNER_LINE_ANGLE()) {
      DEBUG_WARNING("it seems the corner0_l2 is not parallel to the corner1_l1, the line0Angle["
                    << line0Angle << "], and the thresold is: " << PARAMS->CORNER_LINE_ANGLE());
      return false;
    }
    double line1Angle = line_angle_diff(corners.at(1).l2.angle, corners.at(0).l1.angle); 
    if (line1Angle > PARAMS->CORNER_LINE_ANGLE()) {
      DEBUG_WARNING("it seems the corner0_l1 is not parallel to the corner1_l2, the line1Angle["
                    << line1Angle << "], and the thresold is: " << PARAMS->CORNER_LINE_ANGLE());
      return false;
    }

    // re-order s.t. they are counter-clockwise;
    std::pair<CopperCorner, CopperCorner> cornerPair(corners.at(0), corners.at(1));
    if(cross(cv::Point2d(corners.at(0).loc.x - ellipse.center.y, 
                         corners.at(0).loc.y - ellipse.center.x), 
             cv::Point2d(corners.at(1).loc.x - ellipse.center.y, 
                         corners.at(1).loc.y - ellipse.center.x)) > 0)
    {
      cornerPair = std::make_pair(corners.at(1), corners.at(0));
    }

    // step-3. refine the ellipse center if it's never been processed
    if (!centerRefined_[curEllipseID]) {
      ellipseCenter = refine_center(ellipse, InnerCircleMode::incomplete);
      DEBUG_INFO("refine_center: before[" << cv::Point2f(ellipse.center.y, ellipse.center.x) << "]" << ", after[" << ellipseCenter << "]");
      centerRefined_[curEllipseID] = true;
      refineEllipseCenter_[curEllipseID] = ellipseCenter;
      /**************** display the original center and refinement center ***************/
#if DISDEBUG
      cv::Mat samesideRefineImg;
      cv::cvtColor(imgGray, samesideRefineImg, cv::COLOR_GRAY2BGR);
      cv::circle(samesideRefineImg, cv::Point(ellipse.center.y, ellipse.center.x), 2, cv::Scalar(0, 0, 255), -1);
      cv::imshow("samesideRefineImg", samesideRefineImg);
      cv::waitKey();
      cv::circle(samesideRefineImg, cv::Point(ellipseCenter), 2, cv::Scalar(0, 255, 0), -1);
      cv::imshow("samesideRefineImg", samesideRefineImg);
      cv::waitKey();
#endif
    } else {
      ellipseCenter = refineEllipseCenter_[curEllipseID];
      DEBUG_INFO("we don't need refine_center in this ellipse");
    }
    
    // step-4. now we can use the ellipse center and the two same_side corners to estimate the other two points
    cv::Point2d estC2Loc, estC3Loc;
    CopperLine c2AssistLine(cornerPair.first.loc, ellipseCenter);
    CopperLine c3AssistLine(cornerPair.second.loc, ellipseCenter);
    // step-4.1 calculate the intersection point
    if(intersect(c2AssistLine, cornerPair.second.l2, estC2Loc) && 
       intersect(c3AssistLine, cornerPair.first.l1, estC3Loc))
    {
      std::vector<cv::Point2d> quadCorners({cornerPair.first.loc, cornerPair.second.loc, estC2Loc, estC3Loc});
      /**************** display the quad formed by sameside corners ***************/
#if DISDEBUG
      cv::Mat samesideImg;
      cv::cvtColor(imgGray, samesideImg, cv::COLOR_GRAY2BGR);
      cv::circle(samesideImg, cv::Point(ellipseCenter), 2, cv::Scalar(0, 0, 255));
      draw_corner(samesideImg, quadCorners[0], true);
      draw_corner(samesideImg, quadCorners[1], true);
      draw_corner(samesideImg, quadCorners[2], true);
      draw_corner(samesideImg, quadCorners[3], true);
      cv::imshow("samesideImg", samesideImg);
      cv::waitKey();
#endif

      // step-4.2 check whether the quad formed by these corners is valid or not
      bool validQuad = valid_quad(quadCorners);
      if (!validQuad) {
        DEBUG_WARNING("the quad formed by these quads is invalid!");
        return false;
      }
      // step-4.3 check whether the quad formed by these corners is clost to outer ellipse
      bool isNearEllipse = near_ellipse(ellipse, quadCorners, false);
      if (!isNearEllipse) {
        DEBUG_WARNING("the quad formed by these quads is not close to the ellipse!");
        return false;
      }

      quad = CopperQuad(quadCorners, std::vector<bool>({1, 1, 0, 0}), curEllipseID);
      DEBUG_INFO("form quad in same_side situation succeed!");
      return true;      
    } else {
      DEBUG_WARNING("the input lines seems to be parallel because they find the intersection point failed");
      return false;
    }
  }

  bool Detector::process_2_sameside_corners_group(const cv::RotatedRect& ellipse,
                                                  int curEllipseID,
                                                  const std::vector<CopperCorner>& corners, 
                                                  std::vector<CopperQuad> &quads) {
    // step-0 the input corners should include at least two corners
    if (corners.size() < 2) {
      return false;
    }

    // step-1. refine the ellipse center if it's never been processed
    cv::Point2d ellipseCenter;
    if (!centerRefined_[curEllipseID]) {
      ellipseCenter = refine_center(ellipse, InnerCircleMode::incomplete);
      DEBUG_INFO("refine_center: before[" << cv::Point2f(ellipse.center.y, ellipse.center.x) << "]" << ", after[" << ellipseCenter << "]");
      centerRefined_[curEllipseID] = true;
      refineEllipseCenter_[curEllipseID] = ellipseCenter;
    } else {
      ellipseCenter = refineEllipseCenter_[curEllipseID];
      DEBUG_INFO("we don't need refine_center in this ellipse");
    }

    // step-2. group the input corners to sameside group
    // each sameside group has two corners and they are satifised:
    //   1. corner[i].l2 collinear with corner[j].l1 (or reverse)
    //   2. corner[i].l1 parallel with corner[j].l2 (or reverse)
    // TODO: maybe we can decrease the number of samesideGroups generation...
    std::vector<std::pair<int, int>> samesideGroups;
    std::vector<bool> usedCorner(corners.size(), false);
    for (int i  = 0; i < corners.size(); ++i) {
      // if (usedCorner[i]) {
      //   continue;
      // }
      int iPrev, iNext;
      if (i == 0) {
        iPrev = corners.size() - 1;
      } else {
        iPrev = i - 1;
      }
      iNext = (i + 1) % corners.size();
      bool isCollinear = false, isClose = false;
      // we check i and iPrev
      if (!usedCorner[iPrev] && !usedCorner[i]) {
        isCollinear = are_lines_collinear(corners[i].l1, corners[iPrev].l2);
        isClose = distance_of_2point(corners[i].l1.startPt, corners[iPrev].loc) < 3;
        DEBUG_INFO("the corners[" << i << "] and [" << iPrev << "]: collinear["
                  << isCollinear << "], isClose[" << isClose << "]");
        if (isCollinear && isClose) {
          samesideGroups.push_back(std::make_pair(iPrev, i));
          usedCorner[iPrev] = true;
          usedCorner[i] = true;
        }
      }
      // we check i and iNext
      isCollinear = are_lines_collinear(corners[i].l2, corners[iNext].l1);
      isClose = distance_of_2point(corners[i].l2.endPt, corners[iNext].loc) < 3;
      DEBUG_INFO("the corners[" << i << "] and [" << iNext << "]: collinear["
                << isCollinear << "], isClose[" << isClose << "]");
      if (isCollinear && isClose) {
        samesideGroups.push_back(std::make_pair(i, iNext));
        usedCorner[i] = true;
        usedCorner[iNext] = true;
      }

    }

    if (samesideGroups.size() == 0) {
      return true;
    }

    /**************** display the extracted sameside corners ***************/
#if DISDEBUG
    cv::Mat samesideGroupImg;
    cv::cvtColor(imgGray, samesideGroupImg, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < samesideGroups.size(); ++i) {
      draw_corner(samesideGroupImg, corners[samesideGroups[i].first], false);
      draw_corner(samesideGroupImg, corners[samesideGroups[i].second], true);
      cv::imshow("samesideGroupImg", samesideGroupImg);
      cv::waitKey();
    }
#endif

    bool formedQuad = false;
#if DISDEBUG
    cv::Mat samesideEstImg;
    cv::cvtColor(imgGray, samesideEstImg, cv::COLOR_GRAY2BGR);
#endif
    // we use each sameside corner group to form quad
    for (int i = 0; i < samesideGroups.size(); ++i) {
      CopperQuad quad;
      bool samesideRes = process_2_sameside_corners(ellipse, curEllipseID,
                                                    {corners[samesideGroups[i].first],
                                                     corners[samesideGroups[i].second]},
                                                    quad);
      if (samesideRes) {
        formedQuad = true;
        DEBUG_INFO("use samesideGroups[" << i << "] form quad succeed!");
#if DISDEBUG
        draw_quad(samesideEstImg, quad);
        cv::imshow("samesideEstImg", samesideEstImg);
        cv::waitKey();
#endif
        quads.push_back(quad);
      }
    }

#if DISDEBUG
    cv::Mat samesideMergeImg;
    cv::cvtColor(imgGray, samesideMergeImg, cv::COLOR_GRAY2BGR);
#endif

    // we use the facing sameside corner group to merge quad
    if (samesideGroups.size() >= 2) {
      for (int i = 0; i < samesideGroups.size(); ++i) {
        for (int j = i + 1; j < samesideGroups.size(); ++j) {
          CopperLine line1(corners[samesideGroups[i].second].loc,
                           corners[samesideGroups[i].first].loc);
          CopperLine line2(corners[samesideGroups[j].second].loc,
                           corners[samesideGroups[j].first].loc);
          double includedAngle = atan((line1.m - line2.m) / (1 + line1.m * line2.m)) * 180.0 / M_PI;
          if (std::fabs(includedAngle) > 10) {
            continue;
          } else {
            formedQuad = true;
            std::vector<cv::Point2d> cornersLoc({corners[samesideGroups[i].first].loc, 
                                                 corners[samesideGroups[i].second].loc,
                                                 corners[samesideGroups[j].first].loc, 
                                                 corners[samesideGroups[j].second].loc}); 
            std::vector<bool> validCorners({1, 1, 0, 0});
            CopperQuad quad(cornersLoc, validCorners);
            DEBUG_INFO("use samesideGroups[" << i << "] and [" << j << "] merge quad succeed!");
#if DISDEBUG
            draw_quad(samesideMergeImg, quad);
            cv::imshow("samesideMergeImg", samesideMergeImg);
            cv::waitKey();
#endif
            quads.push_back(quad);
          }
        }
      }
    }

    return formedQuad;
  }

  bool Detector::process_3n4_corners(std::vector<CopperCorner> cornerGroup, std::vector<int>idx, CopperQuad &quad, 
                                     bool needStrict, const cv::RotatedRect& ellipse, bool needCheckInner) {
    // step-0: the input corners should include at least three corners
    if (cornerGroup.size() < 3) {
      return false;
    }  
    
    // step-1. In get_candidate() function, we need more stricter standard, which means 
    //         those corners with a real short line, will not be treated as a priority valid one.
    //         But in fit_singleton_corners() function, these corners maybe are all we have,
    //         so we can use them again to try to from a quad.  
    int failedCornerCount = 0;
    for (auto corner : cornerGroup) {
      float minLine = std::min(corner.l1.len, corner.l2.len);
      if (minLine < 15) {
        ++failedCornerCount;
      }
    }
    if (needStrict && failedCornerCount >= 1) {
      DEBUG_WARNING("we found some maybe invalid corner here, not suggest to form the quad directly");
      return false;
    }
    
    // step-2. group the corners according to the input idx
    std::vector<CopperCorner> corners;
    std::vector<bool> validCorners; 
    if(idx.at(0) == idx.at(3)){
      corners = std::vector<CopperCorner>({cornerGroup.at(idx.at(0)), 
                                           cornerGroup.at(idx.at(1)),
                                           cornerGroup.at(idx.at(2)), 
                                           CopperCorner(cv::Point2d(INFINITY, INFINITY))});
      validCorners = std::vector<bool>({1, 1, 1, 0});
      // DEBUG_INFO("the number of valid input corner in is 3");
    }else{
      corners = std::vector<CopperCorner>({cornerGroup.at(idx.at(0)), 
                                           cornerGroup.at(idx.at(1)),
                                           cornerGroup.at(idx.at(2)), 
                                           cornerGroup.at(idx.at(3))}); 
      validCorners = std::vector<bool>({1, 1, 1, 1});
      // DEBUG_INFO("the number of valid input corner in is 4");
    }
  
    // step-3. check if the corners can form a quad
    std::vector<cv::Point2d> quadCorners;
    bool isFormQuad = check_if_form_quad(corners, quadCorners, validCorners);
    if (!isFormQuad) {
      DEBUG_WARNING("the input corners cannot form a quad");
      return false;
    }

    // step-4 check whether the quad formed by these corners is valid or not
    bool validQuad = valid_quad(quadCorners);
    if (!validQuad) {
      DEBUG_WARNING("the quad formed by these quads is invalid!");
      return false;
    }
    
    // step-5 check whether the quad formed by these corners is clost to outer ellipse
    //        this step is not the necessary step, we accept both single quad and quad within ellipse
    if (needCheckInner) {
      bool isNearEllipse = near_ellipse(ellipse, quadCorners, false);
      if (!isNearEllipse) {
        DEBUG_WARNING("the quad formed by these quads is not close to the ellipse!");
        return false;
      }
    }
    
    /*************************** display the corners and formed quad in 3n4 suitation ********************************/
    // TODO

    quad = CopperQuad(quadCorners, validCorners);
    DEBUG_INFO("form quad in 3n4 situation succeed!");
    return true; 
  }
  
  bool Detector::facing(CopperCorner c1, CopperCorner c2) {
      bool set1 = cross(c1.l2.endPt - c1.l2.startPt, c2.l1.endPt - c2.l1.startPt) < 0;
      bool set2 = cross(c2.l2.endPt - c2.l2.startPt, c1.l1.endPt - c1.l1.startPt) < 0;
      bool facing1 = cross(c1.l2.endPt - c1.l2.startPt, c2.loc - c1.loc) < 0;
      bool facing2 = cross(c2.l2.endPt - c2.l2.startPt, c1.loc - c2.loc) < 0;
      return set1 && set2 && facing1 && facing2; 
  }

  bool Detector::face_point(cv::Point2d pt, CopperCorner corner) {
    // the presmise of cross is the centerLine and corner.l1(l2) are not collinear
    // the collinera standard here will be stricter
    if (strict_in_line(pt, corner.l1.startPt, corner.l1.endPt) ||
        strict_in_line(pt, corner.l2.startPt, corner.l2.endPt)) {
      return false;
    }

    cv::Point2d centerLine = pt - corner.loc;
    return cross(centerLine, cv::Point2d(corner.l1.startPt - corner.l1.endPt)) < 0 && 
           cross(centerLine, cv::Point2d(corner.l2.endPt - corner.l2.startPt)) > 0;
  }
  
  cv::Point2d Detector::refine_center(cv::RotatedRect ellipse, int mode) {
    // extract a rectangle ROI area for inner circular detection
    int roiSize = ceil(std::max(ellipse.size.width, ellipse.size.height) / 7);
    cv::Point ROIcenterP(ellipse.center.y, ellipse.center.x);
    cv::Rect centerROI (ellipse.center.y - roiSize, ellipse.center.x - roiSize, roiSize*2, roiSize*2);
    cv::Mat centerImg = imgDownSample_(centerROI).clone();

    // find ellipse
    cv::Mat binaryImg;
    // all contour points of the ROI area
    std::vector<std::vector<cv::Point>> contours;
    // OTSU is good enough for the small ROI area
    cv::threshold(centerImg, binaryImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // find all the contours from the binary image
    cv::findContours(binaryImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // many ellipse shape may be found, but we only accept the best one
    cv::RotatedRect bestEllipse;
    // the worst similarity of the ellipse and its original contour
    // the smaller the better (0 is the best)
    double worstSimilarity = 0.8;
    double outerAspectRatio = ellipse.size.width / ellipse.size.height;
    // the minimum area ratio of current ellipse and outer ellipse
    double MIN_AREA_RATIO = 0.08;
    // current best ellipse score
    // TODO: for those score < 1.5, we are not suggest to use it
    double curBestScore = 1.5;

    /***************** display the center refinement process *****************/
#if DISDEBUG
    cv::Mat refineCenterImg; 
    cv::cvtColor(binaryImg, refineCenterImg, cv::COLOR_GRAY2BGR);
    cv::drawContours(refineCenterImg, contours, -1, cv::Scalar(255, 0, 0), 1);
    cv::imshow("refineCenter", refineCenterImg);
    cv::waitKey();
#endif

    for(auto contour : contours) {
      // we ignore too small contour
      if (contour.size() > 5) {
        // fit a ellipse according to its contour
        cv::RotatedRect curEllipse = cv::fitEllipse(contour);
        // the ideal points of the fitted ellipse
        std::vector<cv::Point> curEllipseContour;
        cv::ellipse2Poly(curEllipse.center, curEllipse.size, static_cast<int>(curEllipse.angle), 0, 360, 1, curEllipseContour);

        // we only compare those ellipses that are big enough
        if (double(curEllipse.size.area() / (roiSize * roiSize)) > MIN_AREA_RATIO) {
          // calculate current ellipse score
          // score = 0.7 * thisSimInv + 0.1 * aspectRatioInv + 0.2 * distInv;
          double thisSimInv, aspectRatioInv, distInv;

          // the similarity of current ellipse and its original contour
          double thisSim = cv::matchShapes(contour, curEllipseContour, cv::CONTOURS_MATCH_I1, 0);
          if (thisSim > worstSimilarity) {
            // continue;
          }
          if (thisSim > 0.00001) {
            thisSimInv = 1 / thisSim;
          } else {
            thisSimInv = 100010;
          }
          // the similarity of current ellipse and outer ellipse
          double innerAspectRatio = curEllipse.size.width / curEllipse.size.height;
          double aspectRatio = innerAspectRatio / outerAspectRatio;
          if (aspectRatio == 1) {
            aspectRatioInv = 10;
          } else {
            aspectRatioInv = 1 / (1 - (innerAspectRatio / outerAspectRatio));
          }
          // the distance between current ellipse center and ROI area center
          cv::Point curEllipseCenter(ellipse.center.y - roiSize + curEllipse.center.x, ellipse.center.x - roiSize + curEllipse.center.y);
          double distance = distance_of_2point(curEllipseCenter, ROIcenterP);
          
          if (distance == 0) {
            distInv = 5;
          } else {
            distInv = 1 / distance;
          }

          double score;
  
          // mode-1 is for most of the situation
          if (mode == InnerCircleMode::complete) {
            score = 0.7 * thisSimInv + 0.1 * aspectRatioInv + 0.2 * distInv;
          // mode-2 is for the diagonal situation, which the inner circle may also being destroyed
          // we will decrease the weight of the similarity of inner circle
          } else if (mode == InnerCircleMode::incomplete) {
            score = 0.3 * thisSimInv + 0.1 * aspectRatioInv + 0.6 * distInv;
          } else {
            score = 0;
          }

          // if the inner ellipse is too close to the ROIarea border, score = 0
          double disClose2Edge = std::sqrt(pow(curEllipse.size.width / 2, 2) +
                                           pow(curEllipse.size.height / 2, 2)) * 0.8;
          DEBUG_INFO("curEllipse.center.x is: " << curEllipse.center.x
                     << ", and curEllipse.center.y is: " << curEllipse.center.y
                     << ", and disClose2Edge is: " << disClose2Edge);
          if ((curEllipse.center.x < disClose2Edge) ||
              (curEllipse.center.y < disClose2Edge) ||
              (curEllipse.center.x + disClose2Edge > roiSize * 1.8) ||
              (curEllipse.center.y + disClose2Edge > roiSize * 1.8)) {
            score = 0;
          }

          // some region was too far away from the ROI center, score = 0
          if (distance > roiSize) {
            score = 0;
          }
          
          DEBUG_INFO("current refinement ellipse center is: [" << curEllipseCenter << "]"
                     << " and the original ellipse center is: [" << ROIcenterP << "], thisSim["
                     << thisSim << "], aspectRatio[" << aspectRatio << "], distance[" 
                     << distance << "], ROISize is[" << roiSize << "], and its score is: " << score);

#if DISDEBUG          
          cv::circle(refineCenterImg, cv::Point(curEllipse.center), 2, cv::Scalar(0, 0, 255), -1);  
          cv::imshow("refineCenter", refineCenterImg);
          cv::waitKey();
#endif

          if (score > curBestScore) {
            curBestScore = score;
            bestEllipse = curEllipse;
          }
        }
      }
    }

    if (curBestScore > 1.5) {
      return cv::Point2d(ellipse.center.y - roiSize + bestEllipse.center.x, 
                         ellipse.center.x - roiSize + bestEllipse.center.y);
    } else {
      return cv::Point2d(ellipse.center.y, ellipse.center.x); 
    }
  }
  
  bool Detector::check_if_form_quad(std::vector<CopperCorner> corners, 
                                    std::vector<cv::Point2d> &quadCorners, 
                                    std::vector<bool> &validCorners) {
    if (corners.size() != 4) {
      return false;
    }
                
    if(!facing(corners.at(0), corners.at(2))) {
      return false; 
    }

    quadCorners = std::vector<cv::Point2d>({corners.at(0).loc, 
                                            corners.at(1).loc, 
                                            corners.at(2).loc, 
                                            corners.at(3).loc});

    // the estimate c1 and c3 corner location                                        
    cv::Point2d estC1Loc; 
    cv::Point2d estC3Loc;
    intersect(corners.at(0).l2, corners.at(2).l1, estC1Loc);
    intersect(corners.at(0).l1, corners.at(2).l2, estC3Loc);

    if (estC1Loc.x > width_ || estC1Loc.x < 0 || estC1Loc.y > height_ || estC1Loc.y < 0) {
      return false;
    }
    if (estC3Loc.x > width_ || estC3Loc.x < 0 || estC3Loc.y > height_ || estC3Loc.y < 0) {
      return false;
    }

    double distC1estC1 = cv::norm(corners.at(1).loc - estC1Loc);
    double distC3estC3 = cv::norm(corners.at(3).loc - estC3Loc);
    double distC1estC3 = cv::norm(corners.at(1).loc - estC3Loc);
    double distC3estC1 = cv::norm(corners.at(3).loc - estC1Loc);
    
    double thresDist = PARAMS->EST_CORNER_THRESH();
    std::vector<double> dists({distC1estC1, distC3estC3, distC1estC3, distC3estC1});

    // calculate the number of valid corner (which is real exist)
    int validCornersCount = 0;
    for (auto validCorner : validCorners) {
      if (validCorner) {
        ++validCornersCount;
      }
    }

    // if the input four corner are all valid corner, we just need to check the following condition:
    // 1. the corner 1 and corner 3 are facing to each other
    // 2. the quad (0-1-2-3) are in anti-clockwise order
    if (validCornersCount == 4) {
      double dist0n1 = distance_of_2point(corners[0].loc, corners[1].loc);
      double dist0nEst1 = distance_of_2point(corners[0].loc, estC1Loc);
      double dist2n3 = distance_of_2point(corners[2].loc, corners[3].loc);
      double dis2nEst3 = distance_of_2point(corners[2].loc, estC3Loc);

      if (!facing(corners.at(1), corners.at(3))) {
        return false;
      } else {
        // the corner1 and corner3 are already in right position
        if ((distC1estC1 < distC1estC3) && (distC3estC3 < distC3estC1)) {
          if (distC1estC1 > (std::max(dist0n1, dist0nEst1) / 10) ||
              distC3estC3 > (std::max(dist2n3, dis2nEst3) / 10) ) {
            return false;
          }
          return true;
        // the corner1 and corner3 should exchange their position
        } else if ((distC1estC1 > distC1estC3) && (distC3estC3 > distC3estC1)) {
          if (distC1estC3 > (std::max(dist0n1, dist0nEst1) / 10) ||
              distC3estC1 > (std::max(dist2n3, dis2nEst3) / 10) ) {
            return false;
          }

          cv::Point2d tmp = quadCorners.at(1);
          quadCorners.at(1) = quadCorners.at(3);
          quadCorners.at(3) = tmp;
          return true;
        // unexpected situation, just return false
        } else {
          return false;
        }
      }
    } else {
      // find value of min dist; 
      auto minElement = std::min_element(dists.begin(), dists.end()); 
      // get index of the min dist(to know which corner has min dist)
      int minIndex = std::distance(dists.begin(), minElement); 

      if(*minElement > thresDist){
        // std::cout << " no good match" << std::endl;
        return false;
      }

      switch (minIndex)
      {
      // case 0: c1-estc1 is closest (means c1 is real), check the c3 position
      case 0:
        if(distC3estC3 > thresDist) {
          quadCorners.at(3) = estC3Loc; 
          validCorners.at(3) = false; 
        }
        break;
      // case 1: c3-estc3 is closest (means c3 is real), check the c1 position
      case 1:
        if(distC1estC1 > thresDist) {
          quadCorners.at(1) = estC1Loc; 
          validCorners.at(1) = false; 
        }
        break;
      // case 2: c1-estc3 is closest (means c1 is real, but it should exchange with c3)
      // 1. if c3-estc1 is far (means c3 is fake, then exchange: c0-c1-c2-c3 -> c0-estc1-c2-c1)
      // 2. if c3-estc1 is close (means c3 is real, then exchange: c0-c1-c2-c3 - > c0-c3-c2-c1)
      case 2:
        if(distC3estC1 > thresDist) {
          quadCorners.at(3) = quadCorners.at(1);
          quadCorners.at(1) = estC1Loc;
          validCorners.at(1) = false; 
        } else {
          cv::Point2d tmp = quadCorners.at(1);
          quadCorners.at(1) = quadCorners.at(3);
          quadCorners.at(3) = tmp;
        }
        break;
      // case 3: c3-estc1 is closest (means c3 is real, but it should exchange with c1)
      // 1. if c1-estc3 is far (means c1 is fake, then exchange: c0-c1-c2-c3 -> c0-c3-c2-estc3)
      // 2. if c1-etsc3 is close (means c1 is real, then exchange: c0-c1-c2-c3 -> c0-c3-c2-c1)
      case 3:
        if(distC1estC3 > thresDist){
          quadCorners.at(1) = quadCorners.at(3);
          quadCorners.at(3) = estC3Loc; 
          validCorners.at(3) = false; 
        }else{
          cv::Point2d tmp = quadCorners.at(1);
          quadCorners.at(1) = quadCorners.at(3);
          quadCorners.at(3) = tmp;
        }
        break;
      // unexpected situation, return false
      default:
        return false; 
      }

      return true;
    }
  }

  bool Detector::near_ellipse(const cv::RotatedRect &ellipse, 
                              std::vector<cv::Point2d> points, 
                              bool bothDir) {
    double maxDist = std::max(ellipse.size.width, ellipse.size.height) / PARAMS->MAX_DIST_K(); 

    for(int i = 0; i < points.size(); i++){
      double dist = point_to_ellipse_dist(ellipse, points[i]);
      DEBUG_INFO("the corner[" << points[i] << "] and ellipse distance is: " << dist);
      // if(dist < 0 || dist > maxDist){
      if(dist < (bothDir ? -maxDist : 0) || dist > maxDist){
        return false; 
      }
    }
    return true; 
  }

  bool Detector::corner_center_inline(const cv::RotatedRect &ellipse, 
                                      std::vector<cv::Point2d> cornerPts) {
    if(cornerPts.size() == 4){
      return in_line(cv::Point2d(ellipse.center.y, ellipse.center.x), cornerPts.at(0), cornerPts.at(2)) && 
             in_line(cv::Point2d(ellipse.center.y, ellipse.center.x), cornerPts.at(1), cornerPts.at(3));
    }else if (cornerPts.size() == 2){
      return in_line(cv::Point2d(ellipse.center.y, ellipse.center.x), cornerPts.at(0), cornerPts.at(1));
    }
    return false; // wrong vect size; 
  }

  bool Detector::in_line(cv::Point2d center, cv::Point2d end1, cv::Point2d end2) {
    CopperLine l1(end1, center);
    CopperLine l2(end2, center);
    // DEBUG_INFO("the line_angle_diff is: " << line_angle_diff(l1.angle, l2.angle));
    return (line_angle_diff(l1.angle, l2.angle) < PARAMS->CORNER_SWAY_THRESH());
  }

  double Detector::quad_corner_dist(CopperQuad q1, CopperQuad q2, int &offset) {
    offset = 0;
    for(int i = 1; i < 4; i++){
      if(cv::norm(q1.cornerLoc.at(0) - q2.cornerLoc.at(i)) < 
         cv::norm(q1.cornerLoc.at(0) - q2.cornerLoc.at(offset)))
      {
        offset = i; 
      }
    }
    double sumDiff = 0;  
    for(int i = 0; i < 4; i++){
      sumDiff += cv::norm(q1.cornerLoc.at(i) - q2.cornerLoc.at((i+offset)%4)); 
    }
    return sumDiff; 
  }
} // namespace CopperTag