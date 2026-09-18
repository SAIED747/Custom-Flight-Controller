/*
 * axis3f.h
 *
 *  Created on: Jan 3, 2026
 *      Author: SAIED
 */

#ifndef INC_AXIS3F_H_
#define INC_AXIS3F_H_



//typedef union {
  typedef struct {
         float x;
         float y;
         float z;
   //};
   //float axis[3];
 } Axis3f;


 typedef struct {
	 float x;
	 float y;
	 float z;
 }Axis3f_Bias;

#endif /* INC_AXIS3F_H_ */
