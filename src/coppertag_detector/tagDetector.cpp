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

#include <chrono>

#include "coppertag/tagDetector.h"

namespace CopperTag {
  /************************************* Helper function *************************************/
  void draw_corner(cv::Mat img, CopperCorner corner, bool valid) {
    cv::Scalar color1, color2;
    if (valid) {
        color1 = cv::Scalar(255, 0, 0);
        color2 = cv::Scalar(0, 255, 0);
    } else {
        color1 = cv::Scalar(0, 0, 255);
        color2 = cv::Scalar(0, 0, 255);
    }
    cv::line(img, corner.l1.startPt, corner.l1.endPt, color1, 2);
    cv::line(img, corner.l2.startPt, corner.l2.endPt, color2, 2);
    cv::circle(img, corner.loc, 3, color1, -1); 
  }

  void draw_quad(cv::Mat img, CopperQuad quad) {
    if (quad.cornerLoc.size() != 4) {
      return;
    }
    for (int i = 0; i < 4; ++i) {
      cv::circle(img, cv::Point(quad.cornerLoc[i]), 2, cv::Scalar(0, 0, 255), -1);
      cv::line(img, cv::Point(quad.cornerLoc[i]), cv::Point(quad.cornerLoc[(i + 1) % 4]), cv::Scalar(0, 255, 0), 2);
    }
  }

  bool on_same_side(std::pair<cv::Point2d, cv::Point2d> line, 
                    cv::Point2d pt1, cv::Point2d pt2) {
    cv::Point2d lineVec = line.second - line.first;
    cv::Point2d vec1 = pt1 - line.first;
    cv::Point2d vec2 = pt2 - line.first;

    // float crossProduct1 = lineVec.x * vec1.y - lineVec.y * vec1.x;
    // float crossProduct2 = lineVec.x * vec2.y - lineVec.y * vec2.x;

    float crossProduct1 = cross(lineVec, vec1);
    float crossProduct2 = cross(lineVec, vec2);

    // If the cross products have the same sign, the points are on the same side
    return (crossProduct1 * crossProduct2 >= 0);
  }

  double find_dist(double semi_major, double semi_minor, cv::Point2d p) {
    double px = std::abs(p.x);
    double py = std::abs(p.y);

    double tx = 0.707;
    double ty = 0.707;

    double a = semi_major;
    double b = semi_minor;

    for (int x = 0; x < 5; x++) {
        double x_val = a * tx;
        double y = b * ty;

        double ex = (a * a - b * b) * std::pow(tx, 3) / a;
        double ey = (b * b - a * a) * std::pow(ty, 3) / b;

        double rx = x_val - ex;
        double ry = y - ey;

        double qx = px - ex;
        double qy = py - ey;

        double r = std::hypot(ry, rx);
        double q = std::hypot(qy, qx);

        tx = std::min(1.0, std::max(0.0, (qx * r / q + ex) / a));
        ty = std::min(1.0, std::max(0.0, (qy * r / q + ey) / b));
        double t = std::hypot(ty, tx);
        tx /= t;
        ty /= t;
    }

    return cv::norm(cv::Point2d(std::copysign(a * tx, p.x), std::copysign(b * ty, p.y)) - p);
  }

  cv::Point2d rotate_point(cv::Point2d pt, double deg) {
    double radians = deg * M_PI / 180.0;

    return cv::Point2d(pt.x*cos(radians) - pt.y*sin(radians), 
                        pt.x*sin(radians) + pt.y*cos(radians));
  }

  double point_to_ellipse_dist(const cv::RotatedRect& ellipse, cv::Point2d pt) {
    // move to center
    cv::Point2d reflect_point(pt.y, pt.x);
    cv::Point2d transedPt(reflect_point.x - ellipse.center.x, reflect_point.y - ellipse.center.y);
    cv::Point2d rotatePt = rotate_point(transedPt, -ellipse.angle);

    double p = (pow((rotatePt.x), 2) / pow(ellipse.size.width/2, 2))
             + (pow((rotatePt.y), 2) / pow(ellipse.size.height/2, 2));
            
    return std::copysign(find_dist(ellipse.size.width / 2, ellipse.size.height / 2, rotatePt), 1-p);
  }

  /********************************* End of Helper function *********************************/


  /*************************************** CopperLine ***************************************/
  CopperLine::CopperLine(cv::Point2d sp, cv::Point2d ep, int ID) {
    startPt = sp;
    endPt = ep;
    segmentID = ID;
    len = abs(cv::norm(endPt - startPt));
    angle = line_angle(startPt, endPt);
    std::pair<double, double> mb = find_line_mb(startPt, endPt);
    m = mb.first; 
    b = mb.second; 
  }

  CopperLine::CopperLine(cv::Point2d sp, cv::Point2d ep) {
    *this = CopperLine(sp, ep, -1);
  }
  /*********************************** End of CopperLine ************************************/


  /************************************** CopperCorner **************************************/
  CopperCorner::CopperCorner(cv::Point2d location) {
    loc = location;
  }

  CopperCorner::CopperCorner(cv::Point2d location, CopperLine line1, CopperLine line2, int id) {
    loc = location; 
    l1 = line1; l2 = line2; 
    segmentID = id; 
  }
  /********************************** End of CopperCorner ***********************************/


  /*************************************** CopperQuad ***************************************/
  CopperQuad::CopperQuad(std::vector<cv::Point2d> corners, std::vector<bool> inputFoundCorners, int id) {
    perimeter = 0;
    validCornerCount = 0;
    cornerLoc = corners;
    foundCorners = inputFoundCorners;
    for(bool corner : inputFoundCorners) {
      if(corner)
        validCornerCount++;
    }
    for (int i = 0; i < cornerLoc.size(); ++i) {
      perimeter += distance_of_2point(cornerLoc[i], cornerLoc[(i + 1) % cornerLoc.size()]);
    }
    averageSideLen = perimeter / 4;
    ellipseId = id; 
  }

  CopperQuad::CopperQuad(std::vector<cv::Point2d> corners, std::vector<bool> inputFoundCorners) {
    *this = CopperQuad(corners, inputFoundCorners, -1);
  }
  /*********************************** End of CopperQuad ************************************/

  
  /**************************************** Detector ****************************************/
  Detector::Detector(int inputH, int inputW) {
    width_ = inputW;  
    height_ = inputH;

    double lenFSA = std::max(2, height_/240);

    aamedObjPtr_ = std::make_shared<AAMED>(inputH, inputW);
    // slightly better? need to investigate these
	  aamedObjPtr_->set_parameters(CV_PI / 3.5, lenFSA, 0.7);

  }

  void Detector::detect(cv::Mat imgInput, std::vector<CopperQuad> &allQuads) {
    // **************** Pre-processing ****************
    imgOrigin_ = imgInput;
    cv::resize(imgInput, imgDownSample_, cv::Size(imgInput.cols / 2, imgInput.rows / 2));
    
#if DEBUG
    imgGray = imgDownSample_.clone();
#endif

    // **************** Extract edge segments ****************
    cv::Mat imgEdge = cv::Mat::zeros(imgDownSample_.rows, imgDownSample_.cols, CV_8UC1);;
    GetSegmentsRes edgeSegments_yx = aamedObjPtr_->get_edgeSegments(imgDownSample_, imgEdge, gradX_, gradY_);
    
    // **************** Find ellipse by AAMED ****************
    std::vector<cv::RotatedRect> detected_ellipse = aamedObjPtr_->detect_ellipse(imgEdge, edgeSegments_yx.points);

    // **************** Reverse edge segments ****************
    std::vector<std::vector<cv::Point>> edgeSegments_xy;
    reverse_xy(edgeSegments_yx.points, edgeSegments_xy);
    
    // **************** Find lines ****************
    std::vector<std::vector<CopperLine>> lines; 
    std::vector<std::vector<cv::Point>> convexCandidate;
    edgeSegments_yx.clusters.clear();
    extract_lines(edgeSegments_xy, edgeSegments_yx.clusters, lines, convexCandidate);

    // **************** Find corners ****************
    std::vector<std::vector<CopperCorner>> corners; 
    find_corners(lines, corners); 


    // **************** Search CopperTag candidates ****************
    std::vector<CopperQuad> coarseQuads;
    get_candidates(detected_ellipse, corners, coarseQuads);

    // **************** Filter CopperTag candidates ****************
    candidates_filter(coarseQuads, allQuads);
  }
}