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

#include "coppertag/tagRule.h"

namespace CopperTag {

  CopperTagRule::CopperTagRule(double size) {
    sSize_ = size; 
    cRad_ = std::sqrt(2) * sSize_/2;
    double imageSize = size*2;
    
    double x = imageSize/2; 
    double y = imageSize/2; 

    center_ = cv::Point2d(x, y);

    // 8*8 = 64 > 60, 14*14 = 196 > 192; 

    // levelCount = (type == SMALL ? 8 : 14); 
    
    levelCount_ = LEVEL_COUNT; 

    fullStp_ = sSize_ / (levelCount_ + 2);  // 2 layer for white outer
    halfStp_ = fullStp_ / 2; 
    boarderWidth_ = sSize_ / 8; 

    tag_ = cv::Mat(imageSize, imageSize, CV_8UC3, WHITE);

    find_xy_list();
    reMapIdx_ = re_map();
    reMapInverse_ = re_map_inverse(reMapIdx_);
    calc_mask(mask_);
  }

  void CopperTagRule::marker_back() {
    // Outer Circles
    cv::circle(tag_, center_, cRad_+boarderWidth_, BLACK, -1);
    cv::circle(tag_, center_, cRad_, WHITE, -1);

    // Inner Rectangles
    double x = center_.x; 
    double y = center_.y;

    cv::Point2d tl;
    cv::Point2d br;

    // Top Bot Covers
    tl = cv::Point2d(x - (sSize_/2), y - cRad_);
    br = cv::Point2d(x + (sSize_/2), y + cRad_);  
    cv::rectangle(tag_, tl, br, BLACK, -1); 
    tl = cv::Point2d(x - (cRad_), y + (sSize_/13)); // for beauty; 
    br = cv::Point2d(x + (cRad_), y + sSize_/2);  
    cv::rectangle(tag_, tl, br, BLACK, -1); 

    // Bit Plane
    tl = cv::Point2d(x - sSize_/2 - boarderWidth_, y - sSize_/2 - boarderWidth_); 
    br = cv::Point2d(x + sSize_/2 + boarderWidth_, y + sSize_/2 + boarderWidth_);
    cv::rectangle(tag_, tl, br, BLACK, -1); 
    tl = cv::Point2d(x - sSize_/2, y - sSize_/2); 
    br = cv::Point2d(x + sSize_/2, y + sSize_/2);
    cv::rectangle(tag_, tl, br, WHITE, -1); 
    
    // // Ears
    double width = 1.5 * boarderWidth_; 
    double offset = sSize_/2; 
    tl = cv::Point2d(x - (cRad_), y - sSize_/2);
    br = cv::Point2d(x - (offset+width), y + sSize_/6);  
    cv::rectangle(tag_, tl, br, BLACK, -1); 
    tl = cv::Point2d(x + (cRad_), y - sSize_/2);
    br = cv::Point2d(x + (offset+width), y + sSize_/6);  
    cv::rectangle(tag_, tl, br, BLACK, -1); 
    // tl = cv::Point2d(x + (sSize_/2 + offset + width), y + sSize_/2);  
    // br = cv::Point2d(x + (sSize_/2 + offset), y + sSize_/12);
    // cv::rectangle(tag_, tl, br, BLACK, -1); 

    // Clean Extra exterior
    cv::circle(tag_, cv::Point(x, y), cRad_+boarderWidth_*std::sqrt(2), WHITE, boarderWidth_);
  }

  void CopperTagRule::calc_mask(std::vector<bool> &mask) {
    for(int i = 0; i < levelCount_ * levelCount_; i++){
      mask.push_back(((i%levelCount_)+(i/levelCount_))%2);
    }
  }

  void CopperTagRule::draw_bit(cv::Point2d center, double padding, 
                           cv::Scalar color, int width) {
    double halfSize = halfStp_*padding; 
    cv::Point2d tl(center.x - halfSize, center.y - halfSize); 
    cv::Point2d br(center.x + halfSize, center.y + halfSize); 

    // cv::rectangle(tag_, tl, br, color, 2);
    cv::rectangle(tag_, tl, br, color, width);
  }

  void CopperTagRule::find_xy_list() {
    double x = center_.x - sSize_/2 + fullStp_*1.5; 
    double y = center_.y - sSize_/2 + fullStp_*1.5; 

    for(int i = 0; i < levelCount_; i++)
    {
      for(int j = 0; j < levelCount_; j++)
      {
        xyList_.push_back(cv::Point2d(x, y));
        x += fullStp_; 
      }
      y += fullStp_; 
      x = center_.y - sSize_/2 + fullStp_*1.5;
    }
  }

  int CopperTagRule::rotate_idx(int index, int degree) {
    switch (int(degree)%360)
    {
    case 0:
      return (index); 
      break;
    case 1:
    case 90:
      return (index % levelCount_ * levelCount_ + 
            (levelCount_ - index / levelCount_) -1);
      break;
    case 2:
    case 180:
      return (levelCount_*levelCount_ - index - 1);
      break;
    case 3:
    case 270:
      return (levelCount_*(levelCount_ - 1 - index%levelCount_) + index / levelCount_);
      break;
    
    default:
      std::cout << "invalid degree number: " << degree << std::endl;  
      return -1;
    }
  }

  std::vector<int> CopperTagRule::re_map_inverse(std::vector<int> reMap) {
    std::vector<int> rst(reMap.size(), 0);
    for(int i = 0; i < reMap.size(); i++){
      rst.at(reMap.at(i)) = i;
    }
    return rst; 
  }

  void CopperTagRule::draw_center_checker(double size) {
    cv::Point2d tl(center_.x - size/2, center_.y - size/2);
    cv::Point2d br(center_.x + size/2, center_.y + size/2);
    cv::rectangle(tag_, tl, br, BLACK, -1);
    cv::circle(tag_, center_, size/3, WHITE, -1); 
  }

#ifdef CopperTag14
  bool CopperTagRule::generate(int words, std::string path) {
    marker_back();
    std::vector<std::bitset<8>> encodedWord;
    
    int fontFace = cv::FONT_HERSHEY_DUPLEX; // Font type 18 2.5
    // int fontFace = cv::FONT_HERSHEY_SIMPLEX; // Font type 19 2.5
    // int fontFace = cv::FONT_HERSHEY_PLAIN; // Font type 11 6
    double fontScale = 2 * double(TAG_SIZE)/512.0; // Font scale factor
    int lineThick = 3 * double(TAG_SIZE)/512.0; 
    std::string message = std::to_string(words);
    double offset = double(message.size())/2.0 * fontScale * 19.5; 
    cv::Point2d textLoc(center_.x - offset, center_.y+cRad_); 
    cv::putText(tag_, message, textLoc, fontFace, fontScale, WHITE, 3, false); 
    // cv::putText(tag_, "abc", textLoc, fontFace, fontScale, WHITE, 3, false); 

    if(encode(words, encodedWord)) {
      draw_marker(encodedWord);
      if (path.size() > 0) {
        cv::imwrite(path, tag_); 
      } else {
        cv::imshow("Tag_"+std::to_string(words), tag_); 
        cv::waitKey();
      }
      return true;
    }
    return false; 
  }

  void CopperTagRule::draw_marker(std::vector<std::bitset<8>> &biStr) {
    // marker_back();
    // std::cout << std::endl;

    for(int i = 0; i < levelCount_ * levelCount_; i++) { 
      // levelCount * levelCount, biStr.size()*field_des

      cv::Scalar bitColor; 
      // int biStrIdx = i;
      // int xyIdx = reMapIdx_.at(biStrIdx);

      int xyIdx = i; 
      int biStrIdx = reMapInverse_.at(xyIdx);

      bool bitVal = biStr.at(biStrIdx/8)[7-biStrIdx%8];

      // bitColor = bitVal ? GRAY : BLACK; // field_Des
      // draw_bit(xyList_.at(xyIdx), 1, bitColor);
      // cv::imshow("tag", tag_);
      // cv::waitKey();

      if(mask_.at(xyIdx))
        bitColor = bitVal ? WHITE : BLACK; // field_Des
      else  
        bitColor = bitVal ? BLACK : WHITE; // field_Des

      // bitColor = bitVal ? GRAY : BLACK; 
      // std::cout << "drawing Loc: " << xyList_.at(xyIdx) << std::endl;

      draw_bit(xyList_.at(xyIdx), 1, bitColor);
      // cv::imshow("tag", tag_);
      // cv::waitKey();
    }
    draw_center_checker(4*fullStp_); 
  }

  std::vector<int> CopperTagRule::re_map() { 
    /**
     * 0 0 0 1 1 1 1 
     * 0 0 0 1 1 1 1 
     * 0 0 2 2 2 3 3 
     * 4 4 2 2 2 3 3 
     * 4 4 2 2 - 5 5 
     * 4 4 3 3 5 5 5 
     * 4 4 3 3 5 5 5 
     */
    std::vector<int> pattern = {1, 2, 14, 15, 16, 28, 29, 30, 
                                3, 4, 5, 6, 17, 18, 19, 20, 
                                32, 33, 34, 46, 47, 48, 61, 62, 
                                42, 43, 56, 57, 70, 71, 84, 85, 
                                58, 59, 72, 73, 74, 86, 87, 88};
    std::vector<int> rst;
    for(int sectId = 0; sectId < 4; sectId++) {
      for(int i = 0; i < pattern.size(); i++) {
        rst.push_back(rotate_idx(pattern.at(i), sectId)); 
      }
    }
    
    std::vector<int> filling = {0, 31, 44, 45, 60, 75, 76, 89, 90};
    for(int dir = 0; dir < 4; dir ++) {
      for(int i = 0; i < filling.size(); i++) {
        rst.push_back(rotate_idx(filling.at(i), dir));
      }
    }
    return rst; 
  }

  bool CopperTagRule::rotation_safe(std::vector<std::bitset<8>> &biStr,const decoder_t &decoder) {
    // Need to change bit count, bit assembly
    // std::cout << "in rotation check" << std::endl;
    for(int dir = 1; dir < 4; dir++) {
      int sampleIdx = 0; 
      std::string dataString(DATA_LENGTH, 0);
      std::string fecString(FEC_LENGTH, 0);
      // Data Part
      for(int i = 0; i < DATA_LENGTH; i++) {
        for(int j = 0; j < FIELD_DESCRIPTOR; j++) {
          int bitJIdx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx], dir));
          bool bitJ = biStr.at(bitJIdx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bitJIdx%FIELD_DESCRIPTOR];
          // std::cout << bitJ << " ";
          dataString.at(i) = (dataString.at(i) << 1) | bitJ;
          sampleIdx++;
        }
        // std::cout << ": " << int(uchar(dataString.at(i))) <<  std::endl;
      }
      // std::cout << std::endl;
      // Fec Part
      for(int i = 0; i < FEC_LENGTH; i++) {
        for(int j = 0; j < FIELD_DESCRIPTOR; j++) {
          int bitJIdx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx], dir));
          bool bitJ = biStr.at(bitJIdx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bitJIdx%FIELD_DESCRIPTOR];
          // std::cout << bitJ << " ";
          fecString.at(i) = (fecString.at(i) << 1) | bitJ;
          sampleIdx++;
        }
        // std::cout << ": " << int(uchar(fecString.at(i))) <<  std::endl;
      }
      // std::cout << " ---------- " << std::endl;
      // std::cout << std::endl;

      schifra::reed_solomon::block<CODE_LENGTH,FEC_LENGTH> block(dataString, fecString);
      if(decoder.decode(block)) {
        // return false;
        // std::cout << "decoded at dir: " << dir << std::endl;
        std::string msg; 
        block.data_to_string(msg);
        if(is_legit(msg)) {
          // std::cout << "also legit" << std::endl;
          return false; 
        }
      }
      // Decode Check speed
      // std::cout << std::endl;
    }
    return true; 
  }

  // Check if legit alias
  bool CopperTagRule::is_legit(std::string inStr) {
    return std::find(validVerifyBit.begin(), validVerifyBit.end(), inStr[DATA_LENGTH-1]) != validVerifyBit.end();
  }
#else 
  bool CopperTagRule::generate(int words, std::string path) {
    marker_back();
    std::vector<std::bitset<4>> encodedWord;

    int fontFace = cv::FONT_HERSHEY_DUPLEX; // Font type 18 2.5
    // int fontFace = cv::FONT_HERSHEY_SIMPLEX; // Font type 19 2.5
    // int fontFace = cv::FONT_HERSHEY_PLAIN; // Font type 11 6
    double fontScale = 2 * double(TAG_SIZE)/512.0; // Font scale factor
    int lineThick = 3 * double(TAG_SIZE)/512.0; 
    std::string message = std::to_string(words);
    double offset = double(message.size())/2.0 * fontScale * 19.5; 
    cv::Point2d textLoc(center_.x - offset, center_.y+cRad_); 
    cv::putText(tag_, message, textLoc, fontFace, fontScale, WHITE, lineThick, false); 

    if(encode(words, encodedWord)) {
      draw_marker(encodedWord);
      if (path.size() > 0) {
        cv::imwrite(path, tag_); 
      } else {
        cv::imshow("Tag_"+std::to_string(words), tag_); 
        cv::waitKey();
      }
      return true;
    }
    return false; 
  }

  void CopperTagRule::draw_marker(std::vector<std::bitset<4>> &biStr) {
    // marker_back();
    // std::cout << std::endl;
    for(int i = 0; i < 64; i ++) { // levelCount * levelCount, biStr.size()*field_des
      cv::Scalar bitColor; 
      int biStrIdx = i;
      int xyIdx = reMapIdx_.at(biStrIdx);

      bool bitVal = biStr.at(biStrIdx/4)[3-biStrIdx%4];

      if(mask_.at(xyIdx))
        bitColor = bitVal ? WHITE : BLACK; // field_Des
      else  
        bitColor = bitVal ? BLACK : WHITE; // field_Des
      
      // bitColor = bitVal ? GRAY : BLACK; // field_Des

      draw_bit(xyList_.at(xyIdx), 1, bitColor);
    }
    
    draw_center_checker(2*fullStp_);
  }

  std::vector<int> CopperTagRule::re_map() {
    /*
    0 0 1 1 2 2 3 3 
    0 0 1 1 2 2 3 3 
    4 4 5 5 6 6 7 7 
    4 4 5 - - 6 7 7 
    8 8 5 - - 6 9 9 
    8 8 A A A A 9 9 
    B B C C D D E E
    B B C C D D E E
    */
    std::vector<int> rst({
       0,  1,  8,  9, 
       2,  3, 10, 11, 
       4,  5, 12, 13, 
       6,  7, 14, 15, 
      16, 17, 24, 25, 
      18, 19, 26, 34, 
      20, 21, 29, 37, 
      22, 23, 30, 31, 
      32, 33, 40, 41, 
      42, 43, 44, 45, 
      38, 39, 46, 47, 
      48, 49, 56, 57, 
      50, 51, 58, 59, 
      52, 53, 60, 61, 
      54, 55, 62, 63, 

      // the center unused bits; 
      28, 29, 35, 36
    });

    return rst; 
  }

  bool CopperTagRule::rotation_safe(std::vector<std::bitset<4>> &biStr,const decoder_t &decoder) {
    // Need to change bit count, bit assembly
    // std::cout << "in rotation check" << std::endl;
    for(int dir = 1; dir < 4; dir++) {
      int sampleIdx = 0; 
      std::string dataString(DATA_LENGTH, 0);
      std::string fecString(FEC_LENGTH, 0);
      // Data Part
      for(int i = 0; i < DATA_LENGTH; i++) {
        int bit0Idx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx], dir));
        int bit1Idx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx+1], dir));
        int bit2Idx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx+2], dir));
        int bit3Idx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx+3], dir));
        bool bit0 = biStr.at(bit0Idx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bit0Idx%FIELD_DESCRIPTOR];
        bool bit1 = biStr.at(bit1Idx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bit1Idx%FIELD_DESCRIPTOR];
        bool bit2 = biStr.at(bit2Idx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bit2Idx%FIELD_DESCRIPTOR];
        bool bit3 = biStr.at(bit3Idx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bit3Idx%FIELD_DESCRIPTOR];
        dataString.at(i) = char(bit0 << 3 | bit1 << 2 | bit2 << 1 | bit3);
        // std::cout << bit0 << bit1 << bit2 << bit3 << " ";
        sampleIdx+=4;
      }
      // std::cout << std::endl;
      for(int i = 0; i < FEC_LENGTH; i++) {
        int bit0Idx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx], dir));
        int bit1Idx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx+1], dir));
        int bit2Idx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx+2], dir));
        int bit3Idx = reMapInverse_.at(rotate_idx(reMapIdx_[sampleIdx+3], dir));
        bool bit0 = biStr.at(bit0Idx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bit0Idx%FIELD_DESCRIPTOR];
        bool bit1 = biStr.at(bit1Idx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bit1Idx%FIELD_DESCRIPTOR];
        bool bit2 = biStr.at(bit2Idx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bit2Idx%FIELD_DESCRIPTOR];
        bool bit3 = biStr.at(bit3Idx/FIELD_DESCRIPTOR)[(FIELD_DESCRIPTOR-1)-bit3Idx%FIELD_DESCRIPTOR];
        fecString.at(i) = char(bit0 << 3 | bit1 << 2 | bit2 << 1 | bit3);
        // std::cout << bit0 << bit1 << bit2 << bit3 << " ";
        sampleIdx+=4;
      }
      // std::cout << std::endl;
      schifra::reed_solomon::block<CODE_LENGTH,FEC_LENGTH> block(dataString, fecString);
      if(decoder.decode(block)) {
        return false;
        // std::cout << "decoded at dir: " << dir << std::endl;
        std::string msg; 
        block.data_to_string(msg);
        if(is_legit(msg)) {
          // std::cout << "also legit" << std::endl;
          return false; 
        }
      }
      // Decode Check speed
      // std::cout << std::endl;
    }
    return true; 
  }

  // Check if legit alias
  bool CopperTagRule::is_legit(std::string inStr) {
    return std::find(validVerifyBit.begin(), validVerifyBit.end(), inStr[DATA_LENGTH-1]) != validVerifyBit.end();
  }
#endif // ifdef CopperTag14
  
} // namespace CopperTag