#ifndef _AAMED_PARAM_H_
#define _AAMED_PARAM_H_

/**
 * width = cols, height = rows; 
 * 
 * top left (0, 0),         top right (width-1, 0)
 * bot left (0, height-1)   bot right (width-1, height-1)
*/

// Convert i, j index to 1d Array index
#define IDX(i,j) i*WIDTH+j

#define dIDX(i,j) i*dCOLS+j
#define iIDX(i,j) i*dCOLS+j

#define OutOfRange(x,y) (x < 0 || y < 0 || x >= iROWS || y >= iCOLS)

#define VALIDATION_NUMBER 360

// group arc
#define FLED_GROUPING_IBmA1_IAnB1      0x00
#define FLED_GROUPING_FBmA1_FAnB1      0x00
#define FLED_GROUPING_FBmA1_CAnB1      0x01
#define FLED_GROUPING_CBmA1_FAnB1      0x10
#define FLED_GROUPING_CBmA1_CAnB1      0x11
#define FLED_GROUPING_CAnB1            0x01
#define FLED_GROUPING_CBmA1            0x10

#endif // _AAMED_PARAM_H_