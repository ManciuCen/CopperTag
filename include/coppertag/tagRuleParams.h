
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

#ifndef _COPPER_TAG_RULE_PARAMS_H_
#define _COPPER_TAG_RULE_PARAMS_H_

#include <cstddef>

#include "Schifra/schifra_galois_field.hpp"
#include "Schifra/schifra_galois_field_polynomial.hpp"
#include "Schifra/schifra_sequential_root_generator_polynomial_creator.hpp"
#include "Schifra/schifra_reed_solomon_encoder.hpp"
#include "Schifra/schifra_reed_solomon_decoder.hpp"
#include "Schifra/schifra_reed_solomon_block.hpp"
#include "Schifra/schifra_error_processes.hpp"
#include "Schifra/schifra_reed_solomon_bitio.hpp"

#define BLACK cv::Scalar(0, 0, 0)
#define WHITE cv::Scalar(255, 255, 255)
#define GRAY cv::Scalar(200, 200, 200)
#define RED cv::Scalar(0, 0, 255)
#define GREEN cv::Scalar(0, 255, 0)
#define BLUE cv::Scalar(255, 0, 0)

#define CopperTag14 
// #undef CopperTag8

#define TAG_SIZE 512

namespace CopperTag {

#ifdef CopperTag14
  const std::size_t LEVEL_COUNT = 14; 
  /* Finite Field Parameters */
  const std::size_t FIELD_DESCRIPTOR                = 8;
  const std::size_t GENERATOR_POLYNOMIAL_INDEX      = 120; 
  const std::size_t GENERATOR_POLYNOMIAL_ROOT_COUNT = 18;
  // watch out to not put too much data in as it will overflow int; 
  /* Reed Solomon Code Parameters */
  const std::size_t CODE_LENGTH = 20; 
  const std::size_t FEC_LENGTH  =  18; 
  const std::size_t DATA_LENGTH = CODE_LENGTH - FEC_LENGTH;
  const schifra::galois::field field(FIELD_DESCRIPTOR,
                                     schifra::galois::primitive_polynomial_size06,
                                     schifra::galois::primitive_polynomial06);

  const std::vector<char> validVerifyBit = {'\x00', '\x01', '\x02', '\x03'};

  typedef schifra::reed_solomon::shortened_encoder<CODE_LENGTH,FEC_LENGTH,DATA_LENGTH> encoder_t;
  typedef schifra::reed_solomon::shortened_decoder<CODE_LENGTH,FEC_LENGTH,DATA_LENGTH> decoder_t;
#else
  const std::size_t LEVEL_COUNT = 8; 
  /* Finite Field Parameters */
  const std::size_t FIELD_DESCRIPTOR                = 4;
  const std::size_t GENERATOR_POLYNOMIAL_INDEX      = 0;
  const std::size_t GENERATOR_POLYNOMIAL_ROOT_COUNT = 10;
  /* Reed Solomon Code Parameters */
  const std::size_t CODE_LENGTH = 15; //(2^4 - 1)
  const std::size_t FEC_LENGTH  =  10;
  const std::size_t DATA_LENGTH = CODE_LENGTH - FEC_LENGTH;

  const schifra::galois::field field(FIELD_DESCRIPTOR,
                                     schifra::galois::primitive_polynomial_size01,
                                     schifra::galois::primitive_polynomial01);
  const std::vector<char> validVerifyBit = {'\x00', '\x01', '\x02'};

  typedef schifra::reed_solomon::encoder<CODE_LENGTH,FEC_LENGTH> encoder_t;
  typedef schifra::reed_solomon::decoder<CODE_LENGTH,FEC_LENGTH> decoder_t;
#endif // ifdef CopperTag14

} // namespace CopperTag

#endif // _COPPER_TAG_RULE_PARAMS_H_