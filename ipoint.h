/*********************************************************** 
*  --- OpenSURF ---                                        *
*  This library is distributed under the GNU GPL. Please   *
*  contact chris.evans@irisys.co.uk for more information.  *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#ifndef IPOINT_H
#define IPOINT_H

#include <vector>
#include <math.h>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "stdio.h"

//-------------------------------------------------------

class Ipoint; // Pre-declaration
typedef std::vector<Ipoint> IpVec;
typedef std::vector<std::pair<Ipoint, Ipoint> > IpPairVec;

//-------------------------------------------------------

//! Ipoint operations
void getMatches(IpVec &ipts1, IpVec &ipts2, double (*normalP1)[2], double (*normalP2)[2], IpPairVec &matches);
int qualifiedMatches(CvMat* FundMat, IpPairVec qMatches, double (*normalP1)[2], double (*normalP2)[2], int size);
/***************for find best threshold*****************/
void getBestMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &bestMatches, double pair1[12][2], double pair2[12][2]);
CvMat* computeFMat(CvMat* points1, CvMat* points2, double pair1[12][2], double pair2[12][2]);

void PrintMat(CvMat* A);
double* computeLine(CvMat* F, double* point, double line[3]);

void printArr3(int size, double (*Arr)[3]);
void standarding(int size, double (*Arr)[2], double (*SArr)[3]);

void printArr(double (*Arr)[2]);
int translateCorners(IpPairVec &matches, const CvPoint src_corners[4], CvPoint dst_corners[4]);
//-------------------------------------------------------

class Ipoint {

public:

  //! Destructor
  ~Ipoint() {};

  //! Constructor
  Ipoint() : orientation(0) {};

  //! Gets the distance in descriptor space between Ipoints
  float operator-(const Ipoint &rhs)
  {
    float sum=0.f;
    for(int i=0; i < 64; ++i)
      sum += (this->descriptor[i] - rhs.descriptor[i])*(this->descriptor[i] - rhs.descriptor[i]);
    return sqrt(sum);
  };

  //! Coordinates of the detected interest point
  float x, y;

  //! Detected scale
  float scale;

  //! Orientation measured anti-clockwise from +ve x-axis
  float orientation;

  //! Sign of laplacian for fast matching purposes
  int laplacian;

  //! Vector of descriptor components
  float descriptor[64];

  //! Placeholds for point motion (can be used for frame to frame motion analysis)
  float dx, dy;

  //! Used to store cluster index
  int clusterIndex;
};

//-------------------------------------------------------


#endif
