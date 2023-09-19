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

#ifndef _COPPER_TAG_RULE_H_
#define _COPPER_TAG_RULE_H_

#include <bitset>
#include <string>

#include "opencv2/opencv.hpp"

#include "tagRuleParams.h"
#include "common/common.h"

namespace CopperTag {

  struct SamplePosRes {
    bool valid;
    uchar sampleVal;
  }; // struct SamplePosRes

  class CopperTagRule {
    public:
      /**
       * @brief default construct function
       */
      CopperTagRule() = default;

      /**
       * @brief construct function with size
       */
      CopperTagRule(double size);

      /**
       * @brief generate marker from given words
       * return false if fail; 
       * @param words 
       * @param path 
       * @return true 
       * @return false 
       */
      bool generate(int words, std::string path);

      // private:

      // 
      cv::Point2d center_;

      // square size 
      double sSize_;

      // tangential circle radius
      double cRad_;

      // levels of data in 1 direction
      int levelCount_;

      // size of a bit unit in pixel 
      double fullStp_;

      // half fullStp
      double halfStp_;

      // width of the boarder 
      double boarderWidth_; 

      // the thing we draw on
      cv::Mat tag_;

      // center location of bits
      std::vector<cv::Point2d> xyList_;

      // reorder sequnce to sampling index
      std::vector<int> reMapIdx_;

      // reverse sampling index to ordering idx; 
      std::vector<int> reMapInverse_;
      
      // masking
      std::vector<bool> mask_; 

      /**
       * @brief draw marker background
       * 
       */
      void marker_back(); 

      /**
       * @brief draw a 2x2 checker board pattern in center with 
       * width being size for ellipse center refinement; 
       * @param size 
       */
      void draw_center_checker(double size);

      /**
       * @brief output mask 
       * 
       * @param mask 
       */
      void calc_mask(std::vector<bool> &mask);

      /**
       * @brief output locations of the bits; 
       * 
       */
      void find_xy_list(); 

      /**
       * @brief return bit order map; 
       * 
       */
      std::vector<int> re_map();

      /**
       * @brief remap_inverse for rotational get bit; 
       * 
       * @param reMap
       */
      std::vector<int> re_map_inverse(std::vector<int> reMap);

      /**
       * @brief draw 1 bit
       * 
       * @param center 
       * @param padding 
       * @param color 
       */
      void draw_bit(cv::Point2d center, double padding = 1, 
                    cv::Scalar color = BLACK, int width = -1);

      /**
       * @brief rotate idx 90 deg
       * 123      741
       * 456  ->  852
       * 789      963
       * 
       * provide both 0, 1, 2, 3
       * and deg 0, 90, 180, 270 idx remap; 
       * @param count 
       * @param degree 
       * @return cv::Point2d 
       */
      int rotate_idx(int index, int degree);
            
        /**
         * @brief check if this is a valid 
         * 
         * @param inStr 
         * @return true 
         * @return false 
         */
      bool is_legit(std::string inStr);

      /**
       * @brief sample bits from img and output 
       * output raw data at bit locations; 
       * 
       * @param img 
       * @param corners 
       * @param dataString 
       * @param fecString 
       */
      bool sample_bits(const cv::Mat imgGray, 
                      std::vector<cv::Point2d> corners, 
                      std::vector<bool> &allBits); 


      /**
       * @brief 
       * project point pt according to H
       * used in project sample points
       * @param H 
       * @param pt 
       * @return cv::Point2d 
       */
      cv::Point2d transform_point(cv::Mat H, cv::Point2d pt); 

      /**
       * @brief 
       * -----------
       * | 1  2  1 |
       * |         |
       * | 2  4  2 |
       * |         |
       * | 1  2  1 |
       * -----------
       * project center, and half distance bit to marker plane
       * sum up all the sampled value by their weight
       * output: sum(weight*sample) / sum(weight) => 0 ~ 255
       * @param img 
       * @param center 
       * @return int 
       */
      SamplePosRes sample_point(const cv::Mat img, cv::Mat H, cv::Point2d center); 

      /**
       * @brief convert msg to number;
       * 
       * @param msg 
       * @return int (output encoded number) 
       */
      int msg_to_ints(std::string msg);

#ifdef CopperTag14
      /**
       * @brief 
       * input raw binary representaiton of marker 
       * draw base black and white base on masking information 
       * @param biStr Ascii representation
       * @param path 
       */
      void draw_marker(std::vector<std::bitset<8>> &biStr); 

      /**
       * @brief input string, output biStr encoding
       * works with draw_marker()
       * 
       * @param message 
       * @return true 
       * @return false 
       */
      bool encode(int msg, std::vector<std::bitset<8>> &biStr);

      /**
       * @brief check if this biStr can only be decode in 1 direction; 
       * The safety check for encoding 
       * @param biStr 
       * @return true 
       * @return false 
       */
      bool rotation_safe(std::vector<std::bitset<8>> &biStr,const decoder_t &decoder);

      /**
       * @brief 
       * input all raw sampled bit, 
       * if decode, return true, and result in rst
       * unmasking here
       * 
       * @param dataString 
       * @param fecString 
       * @param decoded 
       * @return true 
       * @return false 
       */
      bool decode(std::vector<bool> &allBits, int &rst);
#else
      /**
       * @brief  input binary representation of the words and draw marker to path
       * 2 versions for big markers and small markers
       * 
       * @param biStr Ascii representation
       * @param path 
       */
      void draw_marker(std::vector<std::bitset<4>> &biStr); 

      /**
       * @brief input string, output 
       * 
       * @param message 
       * @return true 
       * @return false 
       */
      bool encode(int msg, std::vector<std::bitset<4>> &biStr);

      /**
       * @brief check if this biStr can only be decode in 1 direction; 
       * The safety check for encoding 
       * @param biStr 
       * @return true 
       * @return false 
       */
      bool rotation_safe(std::vector<std::bitset<4>> &biStr,const decoder_t &decoder);

      /**
       * @brief 
       * input all raw sampled bit, 
       * if decode, return true, and result in rst
       * unmasking here
       * @param dataString 
       * @param fecString 
       * @param decoded 
       * @return true 
       * @return false 
       */
      bool decode(std::vector<bool> &allBits, int &rst);
#endif // ifdef CopperTag14
  
  }; // class CopperTagRule

} // namespace CopperTag

#endif // _COPPER_TAG_RULE_H_