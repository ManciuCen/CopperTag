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
#include "coppertag/tagRule.h"

// usage: ./coppertag your_image.jpg

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "you need to specific the input image (name) / path" << std::endl;
    return -1;
  }

  /************ load the detector parameters ************/
  PARAMS->load_from_yaml("../params/detectorParams.yaml");

  /************ read the image from local path ************/
  cv::Mat imgInput = imread(argv[1], cv::IMREAD_GRAYSCALE);
  int rows = imgInput.rows / 2;
  int cols = imgInput.cols / 2;

  /************ initialize the detector / decoder ************/
  CopperTag::Detector detectorObj(rows, cols);
  CopperTag::CopperTagRule decoderObj(TAG_SIZE);

  /************ start detect ************/
  std::vector<CopperTag::CopperQuad> quads;
  detectorObj.detect(imgInput, quads);

  /************ start decode ************/
  for (const auto& quad : quads) {
    int result;
    std::vector<bool> allBits;
    if (decoderObj.sample_bits(imgInput, quad.cornerLoc, allBits)) {
      if (decoderObj.decode(allBits, result)) {
        std::cout << "The CopperTag detection result is: " << result << std::endl;
      }
    }
  }

  return 1;
}