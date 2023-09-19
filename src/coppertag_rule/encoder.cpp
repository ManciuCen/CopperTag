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
#ifdef CopperTag14
  bool CopperTagRule::encode(int msg, std::vector<std::bitset<8>> &biStr) {
    schifra::galois::field_polynomial generator_polynomial(field);

    if (!schifra::make_sequential_root_generator_polynomial(field,
                                                            GENERATOR_POLYNOMIAL_INDEX,
                                                            GENERATOR_POLYNOMIAL_ROOT_COUNT,
                                                            generator_polynomial)) {
      std::cout << "Error - Failed to create sequential root generator!" << std::endl;
      return 1;
    }
    /* Instantiate Encoder and Decoder (Codec) */

    const encoder_t encoder(field,generator_polynomial);
    const decoder_t decoder(field,GENERATOR_POLYNOMIAL_INDEX);

    /* Instantiate RS Block For Codec */
    schifra::reed_solomon::block<CODE_LENGTH,FEC_LENGTH> block;
    

    if(msg >= 1 << ((DATA_LENGTH-1) * FIELD_DESCRIPTOR)) { // 1 << 24
      std::cout << "Asking: " << (1 << (DATA_LENGTH-1) * FIELD_DESCRIPTOR) << std::endl;
      std::cout << "msg size too big: " << msg << std::endl;
      return false; 
    }
    
    /*
    Data = sym0, sym1, sym2, sym3, sym4 ...
    msg[0] = sym1+sym0
    msg[1] = sym3+sym2
    msg[2] = (drop)+sym4
    ...
    */
    bool valid = false; 
    for(auto verifyBit : validVerifyBit) {
      // TODO:need to modify for differnet occlusion levels
      std::string message(DATA_LENGTH, 0);
      for(int i = 0; i < DATA_LENGTH-1; i++) {
        char byte = char((msg>>(8*i)) & 0xFF);
        message[i] = byte; 
      }

      // /*tmp abc*/
      // message[0] = 'a'; 
      // message[1] = 'b'; 
      // message[2] = 'c'; 

      message[DATA_LENGTH-1] = verifyBit; 


      if (!encoder.encode(message, block)) {
          std::cout << "Error - Critical encoding failure! "
                    << "Msg: " << block.error_as_string()  << std::endl;
          return 1;
      }

      // Printing Result
      std::string data_string; 
      std::string fec_string; 
      block.data_to_string(data_string);
      block.fec_to_string(fec_string);


      biStr.clear();
      for(size_t i = 0; i < data_string.size(); i++) {
        // std::cout << " " << int(uchar(data_string[i]));
        biStr.push_back(data_string[i]);
      }
      std::cout << std::endl;

      for(size_t i = 0; i < fec_string.size(); i++) {
        // std::cout << " " << int(uchar(fec_string[i]));
        biStr.push_back(fec_string[i]);
      }
      std::cout << std::endl;
      
      if(biStr.size() != CODE_LENGTH) {
        std::cout << "Error: Encoded biStr.size() != code_length" << std::endl;
        return false; 
      }

      for(int i = 0; i < 25 - CODE_LENGTH; i++) {
        biStr.push_back('\x00'); // push empty stuff to fill the not touched area; 
      }

      if(!rotation_safe(biStr, decoder)) {
        // std::cout << "aliased failure: " << msg << std::endl;
      } else {
        valid = true; 
        break;
      }
    }
    
    if(!valid){
      std::cout << "there are too much or too little verifying bit" << std::endl;
      return false; 
    }
    std::cout << "encode complete" << std::endl; 
    return true; 
  } 
#else
  // Encoder for small code
  bool CopperTagRule::encode(int msg, std::vector<std::bitset<4>> &biStr) {
    /*
      Note: One must make sure to be using primitive polynomials
            of correct degree for generating elements in the specified
            field ie: a primitive polynomial of degree 4 for GF(2^4).
    */

    /* Instantiate Finite Field and Generator Polynomials */

    schifra::galois::field_polynomial generator_polynomial(field);

    if (!schifra::make_sequential_root_generator_polynomial(field,
                                                            GENERATOR_POLYNOMIAL_INDEX,
                                                            GENERATOR_POLYNOMIAL_ROOT_COUNT,
                                                            generator_polynomial)) {
      std::cout << "Error - Failed to create sequential root generator!" << std::endl;
      return 1;
    }
    /* Instantiate Encoder and Decoder (Codec) */

    const encoder_t encoder(field,generator_polynomial);
    const decoder_t decoder(field,GENERATOR_POLYNOMIAL_INDEX);

    /* Instantiate RS Block For Codec */
    schifra::reed_solomon::block<CODE_LENGTH,FEC_LENGTH> block;
    /*
      Note: The data length represents the number of code symbols that will be used.
            The effective data length is then the number of code symbols multipled
            by the number of bits per symbol, in this case it is 4-bits per code
            symbol.
    */

    // need to modify for different occlusion level 
    // currently is 1 4-bit symbol verify, 4 4-bit symbol data, 10 symbol fec;  
    if(msg >= 1 << ((DATA_LENGTH-1) * FIELD_DESCRIPTOR)) { // 1 << 16
      std::cout << "Asking: " << (1 << (DATA_LENGTH-1) * FIELD_DESCRIPTOR) << std::endl;
      std::cout << "msg size too big: " << msg << std::endl;
      return false; 
    }

    /*
    Data = sym0, sym1, sym2, sym3, sym4 ...
    msg[0] = sym1+sym0
    msg[1] = sym3+sym2
    msg[2] = (drop)+sym4
    ...
    */
    bool valid = false; 
    for(auto verifyBit : validVerifyBit) {
      // TODO:need to modify for differnet occlusion levels
      std::string message(DATA_LENGTH*FIELD_DESCRIPTOR/8+1, 0);
      message[0] = uchar(msg);
      message[1] = uchar((uint32_t(msg) >> 8));
      message[2] = verifyBit; 
      /* Copy data from 1 Byte per element message block into 5-bit RS Block */
      schifra::reed_solomon::bitio::convert_data_to_symbol<FIELD_DESCRIPTOR>(message.c_str(), message.size(), block.data);

      /* Transform message into Reed-Solomon encoded codeword */
      if (!encoder.encode(block)) {
          std::cout << "Error - Critical encoding failure! "
                    << "Msg: " << block.error_as_string()  << std::endl;
          return false;
      }

      // Printing Result
      std::string data_string; 
      std::string fec_string; 
      block.data_to_string(data_string);
      block.fec_to_string(fec_string);

      biStr.clear();
      for(size_t i = 0; i < data_string.size(); i++) {
        // std::cout << "-" << int(data_string[i]);
        biStr.push_back(data_string[i]);
      }
      // std::cout << std::endl;

      for(size_t i = 0; i < fec_string.size(); i++) {
        // std::cout << "-" << int(fec_string[i]);
        biStr.push_back(fec_string[i]);
      }
      // std::cout << std::endl;
      
      if(biStr.size() != CODE_LENGTH) {
        std::cout << "Error: Encoded biStr.size() != code_length" << std::endl;
        return false; 
      }

      // Add 4 to make it square for rotational check
      biStr.push_back(std::bitset<4>('\x00'));

      if(!rotation_safe(biStr, decoder)) {
        // std::cout << "aliased failure: " << msg << std::endl;
      } else {
        valid = true; 
        break;
      }
    }
    
    if(!valid) {
      std::cout << "there are too much or too little verifying bit" << std::endl;
      return false;
    }
    // std::cout << "encode complete" << std::endl; 
    return true; 
  } 
#endif // ifdef CopperTag14

} // namespace CopperTag