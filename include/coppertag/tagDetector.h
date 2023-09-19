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

#ifndef _COPPER_TAG_DETECTOR_H_
#define _COPPER_TAG_DETECTOR_H_

#include <vector>
#include <memory>
#include <algorithm>

#include "opencv2/opencv.hpp"

#include "aamed/AAMED.h"
#include "common/common.h"
#include "common/colors.h" 
#include "tagDetectorParams.h"

namespace CopperTag {
  
  enum InnerCircleMode : int {
    complete = 1,
    incomplete = 2
  };

  struct CopperLine {
    // All pixels on the line
    std::vector<cv::Point2d> pixels;

    // The start point pixel position on the line
    cv::Point2d startPt;

    // The end point pixel position on the line
    cv::Point2d endPt;

    // The angle between start point and end point
    double angle;

    // The parameter m in the line equation y = mx + b
    double m;

    // The parameter b in the line equation y = mx + b
    double b;

    // The edge segment ID to which the line belongs
    int segmentID;

    // The line length
    double len; 
    
    /**
     * @brief default constructor
     */
    CopperLine() = default;

    /**
     * @brief constructor with start point, end point and segmentID
     * 
     * @param sp
     * @param ep
     * @param ID
     */
    CopperLine(cv::Point2d sp, cv::Point2d ep, int ID);

    /**
     * @brief constructor with start point and end point
     * 
     * @param sp
     * @param ep
     */
    CopperLine(cv::Point2d sp, cv::Point2d ep);

  }; // struct CopperLine

  struct CopperCorner {
    // The corner pixel position on the image
    cv::Point2d loc;

    // The line pointing from somewhere to the corner
    CopperLine l1;

    // The line pointing from the corner to somewhere
    CopperLine l2;

    // The edge segment ID to which the corner belongs
    int segmentID;

    /**
     * @brief constructor with corner location
     * 
     * @param location
     */
    CopperCorner(cv::Point2d location);

    /**
     * @brief constructor with location, lines and segmentID
     * 
     * @param location
     * @param line1
     * @param line2
     * @param id
     */
    CopperCorner(cv::Point2d location, CopperLine line1, CopperLine line2, int id);
      
  }; // struct CopperCorner

  struct CopperQuad{
    // All corner pixel position of one quad, the number should be four
    std::vector<cv::Point2d> cornerLoc;

    // The corner property: real existence[true] restore by rules[false]
    std::vector<bool> foundCorners;

    // The number of real existing corners
    int validCornerCount;

    // ID of the ellipse which surrounding the quad
    // -1 if no ellipse attach
    int ellipseId;

    // The perimeter of this quad
    double perimeter;

    // The average side length of this quad
    // averageSideLength = perimeter / 4
    double averageSideLen;

    /**
     * @brief default constructor
     */
    CopperQuad() = default; 

    /**
     * @brief constructor with corners, foundCorners and ellipseID
     * 
     * @param corners
     * @param inputFoundCorners
     * @param id
     */
    CopperQuad(std::vector<cv::Point2d> corners, std::vector<bool> inputFoundCorners, int id);

    /**
     * @brief constructor with corners, foundCorners
     * 
     * @param corners
     * @param inputFoundCorners
     */
    CopperQuad(std::vector<cv::Point2d> corners, std::vector<bool> inputFoundCorners);

  }; // struct CopperQuad

  class Detector {
    public:
      // The image for painting (Debug mode)
      cv::Mat imgGray;

      /**
       * @brief constructor with image width and image height
       * 
       * @param inputH
       * @param inputW
       */
      Detector(int inputH, int inputW);

      /**
       * @brief default destructor
       */
      ~Detector() = default;

      /**
       * @brief CopperTag detector
       * 
       * @param imgInput the input image
       * @param allQuads the output candidate quad group
       */
      void detect(cv::Mat imgInput, std::vector<CopperQuad> &allQuads);
    
      float totalTime;
    private:
      // The image width
      int width_;

      // The image height
      int height_;

      // The original size image
      cv::Mat imgOrigin_;

      // The x-direction gradient image of detection image
      cv::Mat gradX_;

      // The y-direction gradient image of detection image
      cv::Mat gradY_;

      // The pointer of AAMED class
      std::shared_ptr<AAMED> aamedObjPtr_;

      // indicate if current ellipse center had been refined
      std::vector<bool> centerRefined_;

      // save each refinement ellipse center in each detection process
      std::vector<cv::Point2d> refineEllipseCenter_;

      // the down-sampling image
      cv::Mat imgDownSample_;

      /**
       * @brief extract lines from input edge segments group
       * 
       * @param edgeSegments 
       * @param edgeClusters
       * @param lines
       * @param convexQuadContours 
       */
      void extract_lines(const std::vector<std::vector<cv::Point>> &edgeSegments,
                         const std::vector<std::vector<int>> &edgeClusters,
                         std::vector<std::vector<CopperLine>> &lines, 
                         std::vector<std::vector<cv::Point>> &convexQuadContours);

      /**
       * @brief determine whether the input line1 and line2  are collinear
       * 
       * @param line1
       * @param line2
       */
      bool are_lines_collinear(const CopperLine& line1, const CopperLine& line2);

      /**
       * @brief input a dpContour and output the legal lines in the contour
       * use correct direction to ensure the inserted lines are in 
       * correct direction(white left, black right)
       * 
       * @param dpContour 
       * @param lines 
       * @param segmentID 
       */
      void filter_lines(std::vector<cv::Point> edgeSegment, 
                        std::vector<cv::Point> dpContour, 
                        std::vector<CopperLine> &lines, 
                        int segmentID, 
                        bool closeContour);

      /**
       * @brief return original indexes of the dpContour; 
       * 
       * @param edgeSegment 
       * @param dpContour 
       * @return std::vector<int>& 
       */
      void dp_Index(std::vector<cv::Point> &edgeSegment, 
                    std::vector<cv::Point> &dpContour, 
                    std::vector<int> &rst);
   
      /**
       * @brief input edgesegments and the corresponding index to sample
       * output the sample result to be cross product; 
       * 
       * @param edgeSegment edgeSegment
       * @param dpIdx index to sample
       * @param sampled_points sampled result;  
       */  
      void sample_points(std::vector<cv::Point> &edgeSegment, 
                         std::vector<int> &sampleIdx, 
                         std::vector<cv::Point2d> &sampledPoints);

      /**
       * @brief check the direction of the line created by endPt and startPt
       * with the gradient of points at checkPts
       * 
       * if direction is reversed, startPt and endPt will be reversed in place
       * such that filter lines can directly use the startPt and endPt to 
       * construct lines; 
       * 
       * @param startPt line start points
       * @param endPt line end points
       * @param sampled_points sampled Points(comes from sample_points)
       * they should be the gradient values; 
       */
      void correct_direction(cv::Point2d &startPt, 
                             cv::Point2d &endPt, 
                             std::vector<cv::Point2d> &sampled_points);

      // ** Corners ** 
      /**
       * @brief input all the lines in the image, 
       * 1) find lines from same segment, using segmentID
       *      only check segment with more than 1 line;
       * 2) assume only consecutive lines form corners
       *    check min l1.end <-> l2.start and l2.end <-> l1.start dist 
       *    if dist == 0 => use intersection as cornerPt; 
       *    if dist < threshold => get intersection 
       *      check theta difference; 
       *      MAYBE?(check distance to original edgeSegments)
       *      MAYBE?(use closest edgeSegment as cornerPt?)
       * 3) Only keep corect heading (check corner orientation)
       * 4) return 2d Vector s.t.same group corner together; 
       * 
       * TODO: potentially use a variable for loop guard to prevent repetitive calculation; 
       * @param lines(input)
       * @param corners(output)
       */
      void find_corners(std::vector<std::vector<CopperLine>> lines, 
                        std::vector<std::vector<CopperCorner>> &corners); 

      /**
       * @brief use smaller DP contour to refine corners
       * 
       * @param l1 
       * @param l2 
       */
      void line_refinement(std::vector<cv::Point> &edgeSegments, 
                            int &idx, 
                            cv::Point2d &cornerPt, 
                            bool forward);
      
      /**
       * @brief find angle difference of 2 lines; 
       * 
       * @param l1Angle 
       * @param l2Angle 
       * @return double 
       */
      double line_angle_diff(double l1Angle, double l2Angle); 

      /**
       * @brief input 2 copperline if intersect, output to rst; 
       * 
       * @param line1 
       * @param line2 
       * @param rst 
       * @return true 
       * @return false 
       */
      bool intersect(CopperLine line1, CopperLine line2, cv::Point2d &rst);

      /**
       * @brief form candidate quads from ellipses and corners
       * 
       * @param quadCorners 
       */
      void get_candidates(std::vector<cv::RotatedRect> ellipses,
                          std::vector<std::vector<CopperCorner>> &corners, 
                          std::vector<CopperQuad> &allQuads);      

      /**
       * @brief remove invalid and duplicate quads
       * 
       * @param inputQuads
       * @param outputQuads
       */
      void candidates_filter(std::vector<CopperQuad> &inputQuads,
                             std::vector<CopperQuad> &outputQuads);

      /**
       * @brief check if 4 corners can form a quad
       * 
       * @param quadCorners 
       * @return true 
       * @return false 
       */
  
      bool check_if_form_quad(std::vector<CopperCorner> corners, 
                              std::vector<cv::Point2d> &quadCorners, 
                              std::vector<bool> &validCorners); 

      /**
       * @brief check if points near threshold
       * 
       * @param ellipse 
       * @param points
       * @param thresh 
       * @return true 
       * @return false 
       */
      bool near_ellipse(const cv::RotatedRect &ellipse, 
                        std::vector<cv::Point2d> points, 
                        bool bothDir = false);

      /**
       * @brief input 2 or 4 corners 
       * 4 corner: 
       *  check if 0, 2 oppose; 1, 3 oppose
       * 2 corner:
       *  check if 0, 1 oppose; 
       * @param ellipse 
       * @param corners 
       * @return true 
       * @return false 
       */
      bool corner_center_inline(const cv::RotatedRect &ellipse, 
                                std::vector<cv::Point2d> cornerPts);

      /**
       * @brief check if 2 corners are facing each other 
       * by checking their arms and 
       * arm relative location to centerline
       * 
       * @param c1 
       * @param c2 
       * @return true 
       * @return false 
       */
      bool facing(CopperCorner c1, CopperCorner c2);  

      /**
       * @brief find center featrue; 
       * 
       * @param ellipse 
       * @param center
       * @param mode 1-prefer similarity to ellipse 2-prefer the shortest distance
       */
      cv::Point2d refine_center(cv::RotatedRect ellipse, int mode = InnerCircleMode::complete); 

      /**
       * @brief check if point in corner Fan;
       * 
       * @param pt 
       * @param corner 
       * @return true 
       * @return false 
       */
      bool face_point(cv::Point2d pt, CopperCorner corner);

      /**
       * @brief check if this corner is good enough to save to the ellipseCorner
       * 
       * @param ellipse 
       * @param corner 
       * @param ellpiseCorner 
       */
      bool process_1_corners(const cv::RotatedRect& ellipse, 
                             CopperCorner corner, 
                             std::vector<CopperCorner> &ellipseCorner);

      /**
       * @brief process the 2 corners that are under same_side condition
       * 
       * @param ellipse
       * @param curEllipseID
       * @param corners 
       * @param ellipseCorner 
       * @param quad 
       * @return true 
       * @return false 
       */
      bool process_2_sameside_corners(const cv::RotatedRect& ellipse,
                                      int curEllipseID,
                                      std::vector<CopperCorner> corners, 
                                      CopperQuad &quad);

      /**
       * @brief process the 2 corners that are under same_side condition
       * @attention the difference between process_2_sameside_corners_group and process_2_sameside_corners is:
       *            1. this function will extract all sameside group first
       *            2. this function will estimate the quad from one sameside corner group and
       *               try to merge two sameside corner groups into one quad
       * 
       * @param ellipse
       * @param curEllipseID
       * @param corners
       * @param quads
       * @return true
       * @return false
       */
      bool process_2_sameside_corners_group(const cv::RotatedRect& ellipse,
                                            int curEllipseID,
                                            const std::vector<CopperCorner>& corners, 
                                            std::vector<CopperQuad> &quads);

      /**
       * @brief process the 2 corners that are under diagonal condition
       * 
       * @param ellipse
       * @param curEllipseID
       * @param corners
       * @param quad
       * @return true
       * @return false
       */
      bool process_2_diagonal_corners(const cv::RotatedRect& ellipse,
                                      int curEllipseID,
                                      const std::vector<CopperCorner>& corners, 
                                      std::vector<CopperQuad> &quads);

      /**
       * @brief process the 3 or 4 corners that are probably can form a quad directly
       * 
       * @param cornerGroup 
       * @param quad 
       * @return true 
       * @return false 
       */
      bool process_3n4_corners(std::vector<CopperCorner> cornerGroup, 
                               std::vector<int>idx, 
                               CopperQuad &quad,
                               bool needStrict,
                               const cv::RotatedRect& ellipse = cv::RotatedRect(),
                               bool needCheckInner = false);

      /**
       * @brief test fit all the singleton ellpise and extract all the ones that forms a quad; 
       * 
       * @param ellipse 
       * @param ellipseCorner 
       * @param quads 
       */
      bool fit_singleton_corners(const cv::RotatedRect& ellipse,
                                 int curEllipseID,
                                 std::vector<CopperCorner> ellipseCorner, 
                                 std::vector<CopperQuad> &allQuads); 

      /**
       * @brief check if these 3 points are in a line;
       * 
       * @param center 
       * @param end1 
       * @param end2 
       * @return true 
       * @return false 
       */
      bool in_line(cv::Point2d center, cv::Point2d end1, cv::Point2d end2);

      /**
       * @brief 
       * 1) compare q1.cornerLoc[1] closest to q2.corner[offset]
       * 2) return total corner loc Diff; 
       * 
       * @param q1 
       * @param q2 
       * @param offset 
       * @return double 
       */
      double quad_corner_dist(CopperQuad q1, CopperQuad q2, int &offset);

      /************************************ template function for quadrilateral validity checks ************************************/

      /**
       * @brief calculate the line at infinity position (reference STag)
       * 
       * @param corners
       * @return cv::Point3d
       */
      template <typename T>
      cv::Point3d calculate_line_at_infinity(std::vector<T> corners) {
        // intersection points at the vanishing line
        cv::Point2d inters1, inters2;
        cv::Point3d lineInf;

        if (corners.size() != 4) {
          return lineInf;
        }

        // cross products of corners (i.e. lines representing the edges)
        double cross14 = cross(corners[0], corners[3]);
        double cross23 = cross(corners[1], corners[2]);
        double cross12 = cross(corners[0], corners[1]);
        double cross34 = cross(corners[2], corners[3]);

        // vectors going from one corner to another
        cv::Point2d vec23(corners[1].x - corners[2].x, corners[1].y - corners[2].y);
        cv::Point2d vec14(corners[0].x - corners[3].x, corners[0].y - corners[3].y);
        cv::Point2d vec34(corners[2].x - corners[3].x, corners[2].y - corners[3].y);
        cv::Point2d vec12(corners[0].x - corners[1].x, corners[0].y - corners[1].y);

        // if both edge pairs are parallel
        if ((cross(vec14, vec23) == 0) && (cross(vec12, vec34) == 0)) // lines are parallel
        {
          lineInf = cv::Point3d(0, 0, 1);
          return lineInf;
        }
        // if one edge pair is parallel
        else if (cross(vec14, vec23) == 0)
        {
          inters2.x = (cross12 * vec34.x - vec12.x * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);
          inters2.y = (cross12 * vec34.y - vec12.y * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);

          //this intersection is not real. doing this to find the equation of the line with only one point.
          inters1.x = inters2.x + vec14.x;
          inters1.y = inters2.y + vec14.y;
        }
        // if the other edge pair is parallel
        else if (cross(vec12, vec34) == 0)
        {
          inters1.x = (cross14 * vec23.x - vec14.x * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);
          inters1.y = (cross14 * vec23.y - vec14.y * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);

          //this intersection is not real. doing this to find the equation of the line with only one point.
          inters2.x = inters1.x + vec12.x;
          inters2.y = inters1.y + vec12.y;
        }
        // if neither pairs are parallel
        else
        {
          inters1.x = (cross14 * vec23.x - vec14.x * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);
          inters1.y = (cross14 * vec23.y - vec14.y * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);

          inters2.x = (cross12 * vec34.x - vec12.x * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);
          inters2.y = (cross12 * vec34.y - vec12.y * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);
        }

        // find the vanishing line in homogeneous coordinates
        // l = P1 x P2
        double l1 = inters1.y - inters2.y;
        double l2 = inters2.x - inters1.x;
        double l3 = inters1.x * inters2.y - inters2.x * inters1.y;

        // normalize using http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html (13)
        double normalizer = sqrt(l1 * l1 + l2 * l2);
        l1 /= normalizer;
        l2 /= normalizer;
        l3 /= normalizer;

        lineInf = cv::Point3d(l1, l2, l3);
        return lineInf;
      }

      /**
       * @brief calculate the projective distortion of input quad (reference STag)
       * 
       * @param corners
       * @return double 
       */
      template <typename T>
      double calculate_projective_distortion(std::vector<T> &corners) {
        if (corners.size() != 4) {
          return 999;
        }

        cv::Point3d lineInf = calculate_line_at_infinity(corners);

        // find the minimum and maximum distance from corners to the vanishing line
        // projective distortion = maxDist / minDist
        double curDist = abs(lineInf.x * corners[0].x + lineInf.y * corners[0].y + lineInf.z);

        double minDist = curDist;
        double maxDist = curDist;

        for (int i = 1; i < 4; i++)
        {
          curDist = abs(lineInf.x * corners[i].x + lineInf.y * corners[i].y + lineInf.z);
          if (curDist < minDist)
            minDist = curDist;
          if (curDist > maxDist)
            maxDist = curDist;
        }
        return maxDist / minDist;
      }

      /**
       * @brief check if 4 corner locations are in clockwise order and valid; 
       * 
       * @param corners 
       * @return true 
       * @return false 
       */
      template <typename T>
      bool valid_quad(std::vector<T> cornerLoc) {
        // a valid quad should had four corners
        if (cornerLoc.size() != 4)
          return false;

        // its projective distortion should less than 1.6 (reference STag)
        if (calculate_projective_distortion(cornerLoc) > 1.6)
          return false;

        T l0 = cornerLoc.at(1) - cornerLoc.at(0);
        T l1 = cornerLoc.at(2) - cornerLoc.at(1); 
        T l2 = cornerLoc.at(3) - cornerLoc.at(2); 
        T l3 = cornerLoc.at(0) - cornerLoc.at(3);

        // each side length should longer than 15 pixel and the corners are in clockwise order
        return cv::norm(l0) > 15 && cv::norm(l1) > 15 && cv::norm(l2) > 15 && cv::norm(l3) > 15 && 
               cross(l0, l1) < 0 && cross(l1, l2) < 0 && cross(l2, l3) < 0 && cross(l3, l0) < 0; 
      }

  }; // class Detector

  // some little helper function
  /**
   * @brief draw the corner on the input image, valid(green), invalid(red)
   * 
   * @param img
   * @param corner
   * @param valid
   */
  void draw_corner(cv::Mat img, CopperCorner corner, bool valid);

  /**
   * @brief draw the quad on the input image
   * 
   * @param img
   * @param quad 
   */
  void draw_quad(cv::Mat img, CopperQuad quad);

  /**
   * @brief determine the input point pt1 and pt2 whether are on the same side of input line
   * 
   * @param line
   * @param pt1
   * @param pt2
   * 
   * @return true / false 
   */
  bool on_same_side(std::pair<cv::Point2d, cv::Point2d> line, 
                    cv::Point2d pt1, cv::Point2d pt2);

  /**
   * @brief calculate the distance between the input point and the input ellipse center
   * 
   * @param semi_major
   * @param semi_minor
   * @param p
   * 
   * @reference http://wet-robots.ghost.io/simple-method-for-distance-to-ellipse/
   */
  double find_dist(double semi_major, double semi_minor, cv::Point2d p);

  /**
   * @brief rotate the input point according the input degree
   * 
   * @param pt
   * @param deg
   */
  cv::Point2d rotate_point(cv::Point2d pt, double deg);

  /**
   * @brief calculate the distance between the input pt and input ellipse
   * 
   * @param ellipse
   * @param pt 
   */
  double point_to_ellipse_dist(const cv::RotatedRect& ellipse, cv::Point2d pt);
} // namespace CopperTag

#endif // _COPPER_TAG_DETECTOR_H_
