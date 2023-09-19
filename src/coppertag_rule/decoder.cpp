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

#include "coppertag/tagRule.h"

namespace CopperTag {

  cv::Point2d CopperTagRule::transform_point(cv::Mat H, cv::Point2d pt) {
    float x = H.at<double>(0, 0) * (pt.x - sSize_/2) + H.at<double>(0, 1) * (pt.y - sSize_/2) + H.at<double>(0, 2) * 1; 
    float y = H.at<double>(1, 0) * (pt.x - sSize_/2) + H.at<double>(1, 1) * (pt.y - sSize_/2) + H.at<double>(1, 2) * 1; 
    float t = H.at<double>(2, 0) * (pt.x - sSize_/2) + H.at<double>(2, 1) * (pt.y - sSize_/2) + H.at<double>(2, 2) * 1; 
    return cv::Point2d(x/t, y/t); 
  }
  
  bool CopperTagRule::sample_bits(const cv::Mat imgGray, 
                                  std::vector<cv::Point2d> corners, 
                                  std::vector<bool> &allBits) {
    std::vector<cv::Point2f> inPoints({cv::Point2f(0, 0), 
                                       cv::Point2f(0, sSize_), 
                                       cv::Point2f(sSize_, sSize_), 
                                       cv::Point2f(sSize_, 0)});

    std::vector<cv::Point2f> outPoints({cv::Point2f(corners[0].x, corners[0].y), 
                                        cv::Point2f(corners[1].x, corners[1].y), 
                                        cv::Point2f(corners[2].x, corners[2].y), 
                                        cv::Point2f(corners[3].x, corners[3].y)});

    cv::Mat H = cv::getPerspectiveTransform(inPoints, outPoints);


    // Sample All bits
    std::vector<uchar> sampleValues; 
    for(int i = 0; i < levelCount_ * levelCount_; i++){
      SamplePosRes sampleRes;
      sampleRes = sample_point(imgGray, H, xyList_.at(i));
      
      if (!sampleRes.valid) {
        return false;
      }
      sampleValues.push_back(sampleRes.sampleVal); 
    }
    std::vector<uchar> threshOutput;
    double threshold = cv::threshold(sampleValues, threshOutput, 0, 1, cv::THRESH_BINARY + cv::THRESH_OTSU);
    allBits = std::vector<bool>(threshOutput.begin(), threshOutput.end());

    return true;
  }

  SamplePosRes CopperTagRule::sample_point(const cv::Mat imgGray, cv::Mat H, cv::Point2d center) {
    SamplePosRes curSampleRes;
    int max_x = imgGray.cols;
    int max_y = imgGray.rows;
    cv::Point2d samplePosition = transform_point(H, center);

    // we need to make sure the samplePosition is within the image
    if (samplePosition.x > max_x || samplePosition.y > max_y ||
        samplePosition.x < 0 || samplePosition.y < 0) {
      curSampleRes.valid = false;
      curSampleRes.sampleVal = 0;
    } else {
      curSampleRes.valid = true;
      curSampleRes.sampleVal = imgGray.at<uchar>(samplePosition);
    }

    return curSampleRes;
  }

  // only need to modify this for CopperTag14 and SMALL
  int CopperTagRule::msg_to_ints(std::string msg) {
    int rst = 0; 
    // std::cout << "DATA_LENGTH-2: " << DATA_LENGTH-2 << std::endl;
    for(int i = DATA_LENGTH-2; i >= 0; i --){
      rst = rst << FIELD_DESCRIPTOR | uchar(msg.at(i));
    }
    return rst; 
  }

#ifdef CopperTag14
  
  bool CopperTagRule::decode(std::vector<bool> &allBits, int &rst) {
    /* Instantiate Finite Field and Generator Polynomials */

    schifra::galois::field_polynomial generator_polynomial(field);

    if (!schifra::make_sequential_root_generator_polynomial(field,
                                                            GENERATOR_POLYNOMIAL_INDEX,
                                                            GENERATOR_POLYNOMIAL_ROOT_COUNT,
                                                            generator_polynomial)) {
      std::cout << "Error - Failed to create sequential root generator!" << std::endl;
      return false;
    }
    /* Instantiate Encoder and Decoder (Codec) */

    const encoder_t encoder(field,generator_polynomial);
    const decoder_t decoder(field,GENERATOR_POLYNOMIAL_INDEX);

    // set up data and fec
    for(int dir = 0; dir < 4; dir++) {

      std::string dataString, fecString;
      int sampleIdx = 0;

      // put in data
      for(int i = 0; i < DATA_LENGTH; i++) {
        char symbol = 0;
        for(int j = 0; j < FIELD_DESCRIPTOR; j++) {
          bool bit = allBits.at(rotate_idx(reMapIdx_.at((sampleIdx)), dir));
          if (!mask_.at(reMapIdx_.at((sampleIdx)))) bit = !bit;
          symbol = symbol << 1 |  bit; 
          sampleIdx++;
        }
        // std::cout << int(symbol) << "-";
        dataString.push_back(symbol);
      }

      // std::cout << std::endl;
      // put in fec
      for(int i = 0; i < FEC_LENGTH; i++) {
        char symbol = 0;
        for(int j = 0; j < FIELD_DESCRIPTOR; j++) {
          bool bit = allBits.at(rotate_idx(reMapIdx_.at((sampleIdx)), dir));
          if (!mask_.at(reMapIdx_.at((sampleIdx)))) bit = !bit;
          // std::cout << bit;
          symbol = symbol << 1 |  bit; 
          sampleIdx++;
        }
        // std::cout << int(symbol) << "-";
        fecString.push_back(symbol);
      }
      // std::cout << std::endl;

      /* Instantiate RS Block For Codec */
      schifra::reed_solomon::block<CODE_LENGTH,FEC_LENGTH> decode_block(dataString, fecString);
      
      if (!decoder.decode(decode_block)) {
        // std::cout << "Error - Critical decoding failure! "
        //           << "Msg: " << decode_block.error_as_string()  << std::endl;
        // return false;
        continue;
      }
      std::string decoded; 
      decode_block.data_to_string(decoded);
      if (!is_legit(decoded)) {
        // std::cout << "message not legit" << std::endl;
        // return false; 
        continue; 
      }

      // std::cout << "legit!" << std::endl;
      rst = msg_to_ints(decoded);
      return true; 
    }
    return false; 
  } 
#else
  bool CopperTagRule::decode(std::vector<bool> &allBits, int &rst) {
    /* Instantiate Finite Field and Generator Polynomials */

    schifra::galois::field_polynomial generator_polynomial(field);

    if (!schifra::make_sequential_root_generator_polynomial(field,
                                                            GENERATOR_POLYNOMIAL_INDEX,
                                                            GENERATOR_POLYNOMIAL_ROOT_COUNT,
                                                            generator_polynomial)) {
      std::cout << "Error - Failed to create sequential root generator!" << std::endl;
      return false;
    }
    /* Instantiate Encoder and Decoder (Codec) */

    const encoder_t encoder(field,generator_polynomial);
    const decoder_t decoder(field,GENERATOR_POLYNOMIAL_INDEX);

    // set up data and fec
    for(int dir = 0; dir < 4; dir++){

      std::string dataString, fecString;
      int sampleIdx = 0;

      // put in data
      for(int i = 0; i < DATA_LENGTH; i++) {
        char symbol = 0;
        for(int j = 0; j < FIELD_DESCRIPTOR; j++) {
          bool bit = allBits.at(rotate_idx(reMapIdx_.at((sampleIdx)), dir));
          if (!mask_.at(reMapIdx_.at((sampleIdx)))) bit = !bit;
          symbol = symbol << 1 |  bit; 
          sampleIdx++;
        }
        // std::cout << int(symbol) << "-";
        dataString.push_back(symbol);
      }

      // std::cout << std::endl;
      // put in fec
      for(int i = 0; i < FEC_LENGTH; i++) {
        char symbol = 0;
        for(int j = 0; j < FIELD_DESCRIPTOR; j++) {
          bool bit = allBits.at(rotate_idx(reMapIdx_.at((sampleIdx)), dir));
          // std::cout << bit;
          if (!mask_.at(reMapIdx_.at((sampleIdx)))) bit = !bit;
          // std::cout << bit;
          symbol = symbol << 1 |  bit; 
          sampleIdx++;
        }
        // std::cout << int(symbol) << "-";
        fecString.push_back(symbol);
      }
      // std::cout << std::endl;

      /* Instantiate RS Block For Codec */
      schifra::reed_solomon::block<CODE_LENGTH,FEC_LENGTH> decode_block(dataString, fecString);
      
      if (!decoder.decode(decode_block)) {
        // std::cout << "Error - Critical decoding failure! "
        //           << "Msg: " << decode_block.error_as_string()  << std::endl;
        // return false;
        continue;
      }
      std::string decoded; 
      decode_block.data_to_string(decoded);
      if (!is_legit(decoded)) {
        // std::cout << "message not legit" << std::endl;
        // return false; 
        continue; 
      }

      // std::cout << "legit!" << std::endl;
      rst = msg_to_ints(decoded);
      return true; 
    }
    return false; 
  } 
#endif // ifdef CopperTag14

} // namespace CopperTag