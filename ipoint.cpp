#include "cv.h"

#include <vector>

#include "ipoint.h"


CvMat* computeFMat(CvMat* points1, CvMat* points2, double pair1[12][2], double pair2[12][2])
{
    points1 = cvCreateMat(12,2,CV_64F);
    points2 = cvCreateMat(12,2,CV_64F);
	CvMat* FundMat = NULL;
    FundMat = cvCreateMat(3,3,CV_64F);

	//cvInitMatHeader(points1,12,2,CV_64F,pair1);
    //cvInitMatHeader(points2,12,2,CV_64F,pair2);
	cvSetData(points1,pair1,points1->step);
	cvSetData(points2,pair2,points2->step);
	//PrintMat(points1);
	//PrintMat(points2);

	//num指找到的基础矩阵的数量
	int num = cvFindFundamentalMat(points1,points2,FundMat,CV_FM_RANSAC,1.00,0.99);
	
	//int num = cvFindFundamentalMat(points1,points2,FundMat,CV_FM_8POINT,0,0,0);
	//int num = cvFindFundamentalMat(points1,points2,FundMat,8,3,0.99,NULL);
	if(num == 0)
	{   
		printf("Can't create F Mat!\n");
		return NULL;
	}
	printf("F  in computeFMat= \n");
	PrintMat(FundMat);

	return FundMat;
}

void PrintMat(CvMat* A)
{
    int i,j;
    for(i=0;i<A->rows;i++)
    {
        printf("\n");
        
        switch( CV_MAT_DEPTH(A->type) )
        {
        case CV_32F:
            for(j=0;j<A->cols;j++)
                printf("%9.8f ", (float) cvGetReal2D( A, i, j ));
            break;
        case CV_64F:
            for(j=0;j<A->cols;j++)
                printf("%9.8f ", (double) cvGetReal2D( A, i, j ));
            break;
        case CV_8U:
        case CV_16U:
            for(j=0;j<A->cols;j++)
                printf("%6d",(int)cvGetReal2D( A, i, j ));
            break;
        default:
            break;
        }
    }
    printf("\n");
}

//! Populate IpPairVec with matched ipts 
void getBestMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &bestMatches, double pair1[12][2], double pair2[12][2])
{
  double dist, d1, d2;
  Ipoint *match = NULL;
  int p = 0;

  bestMatches.clear();

  for(unsigned int i = 0; i < ipts1.size(); i++)
  {
    d1 = d2 = FLT_MAX;

    for(unsigned int j = 0; j < ipts2.size(); j++) 
    {
      dist = ipts1[i] - ipts2[j];  

      if(dist<d1) // if this feature matches better than current best
      {
        d2 = d1;
        d1 = dist;
        match = &ipts2[j];
      }
      else if(dist<d2) // this feature matches better than second best
      {
        d2 = dist;
      }
    }

    // If match has a d1:d2 ratio < 0.45 ipoints are a match
    if(d1/d2 < 0.45f)//find 10 matches
    { 
      ipts1[i].dx = match->x - ipts1[i].x; 
      ipts1[i].dy = match->y - ipts1[i].y;
      bestMatches.push_back(std::make_pair(ipts1[i], *match));

	  /****************for best threshold************************/
	  pair1[p][0] = ipts1[i].x; pair1[p][1] = ipts1[i].y;
	  pair2[p][0] = match->x;   pair2[p][1] = match->y;
	  /****************for best threshold************************/
	  if(p < 12)
		  p = p + 1;
    }
  }
}
//! Populate IpPairVec with matched ipts 
void getMatches(IpVec &ipts1, IpVec &ipts2, double (*normalP1)[2], double (*normalP2)[2], IpPairVec &matches)
{
  double dist, d1, d2;
  int p = 0;
  Ipoint *match =NULL;

  matches.clear();

  for(unsigned int i = 0; i < ipts1.size(); i++)
  {
    d1 = d2 = FLT_MAX;
	//#define FLT_MAX         3.402823466e+38F(double的上限)   后面加f表示是float类型的数据

    for(unsigned int j = 0; j < ipts2.size(); j++) 
    {
      dist = ipts1[i] - ipts2[j];  
	  /*
	  printf("%d loop: dist = %f | ", i, dist);
	  printf("d1 = %f | ", d1);
	  printf("d2 = %f \n", d2);
	  */

      if(dist<d1) // if this feature matches better than current best
      {
        d2 = d1;
        d1 = dist;
        match = &ipts2[j];
      }
      else if(dist<d2) // this feature matches better than second best
      {
        d2 = dist;
      }
    }

    // If match has a d1:d2 ratio < 0.65 ipoints are a match
	/*********************************************
	 ************在这里加阈值寻优函数*************
	 *********************************************/

    if(d1/d2 < 0.7) 
    { 
      // Store the change in position
      ipts1[i].dx = match->x - ipts1[i].x; 
      ipts1[i].dy = match->y - ipts1[i].y;
      matches.push_back(std::make_pair(ipts1[i], *match));
	  /****************for best threshold************************/
	  *(*(normalP1+p)+0) = ipts1[i].x; *(*(normalP1+p)+1)= ipts1[i].y;
	  *(*(normalP2+p)+0) = match->x; *(*(normalP2+p)+1)= match->y;
	  /****************for best threshold************************/
	  p = p + 1;
    }
  }
  /*
   printf("getMatches-normalPoint1 = \n");
   printArr(normalP1);
   printf("getMatches-normalPoint2 = \n");
   printArr(normalP2);
   */
}

double* computeLine(CvMat* F, double* point, double line[3])
{
	for(int j = 0; j < 3; j++)
	{
		for(int jj = 0; jj < 3; jj++)
		{
			line[j] += CV_MAT_ELEM(*F, double, j, jj) * point[jj];
			//printf("F[%d][%d] = %9.8f\n",j,jj,CV_MAT_ELEM(*F, double, j, jj));

		}
	}
	return NULL;
}
void printArr(double (*Arr)[2])
{
	for(int i = 0; i < 20; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			printf("%lf ", Arr[i][j]);
		}
		printf("\n");
	}

	printf("\n");
}

void printArr3(int size, double (*Arr)[3])
{
	for(int i = 0; i < size; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			printf("%lf ", Arr[i][j]);
		}
		printf("\n");
	}

	printf("\n");
}

void standarding(int size, double (*Arr)[2], double (*SArr)[3])
{
	for(int i = 0; i < size; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			SArr[i][j] = Arr[i][j];
		}
		SArr[i][2] = 1;
	}
}

int qualifiedMatches(CvMat* FundMat, IpPairVec qMatches, double (*normalP1)[2], double (*normalP2)[2], int size)
{
	int bPairNum = 0;
	double distance;
	double lines[3] = {0,0,0};
	double tmp;
	int i;
	double normalp1[size][3];
	double normalp2[size][3];
	standarding(size, normalP1, normalp1);
	standarding(size, normalP2, normalp2);
	/******************************************/
	/*
	printf("qualified-normalP1 = \n");
	printArr(normalP1);
	printf("qualified-normalP2 = \n");
	printArr(normalP2);
	size = 20;
	printf("normalp1 = \n");
	printArr3(size, normalp1);
	printf("normalp2 = \n");
	printArr3(size, normalp2);
	*/
	/******************************************/

	for(i = 0; i < size; i++)
	{
		//line=FundMat*normalp1;
		computeLine(FundMat,*(normalp2+i),lines);

		/*
			for(int j = 0; j < 3; j++)
				printf("%9.4f ",lines[j]);
		printf("\n");
		*/
 

		//TMP=normalp1*line
		tmp = (normalp1[i][0] * lines[0]) + (normalp1[i][1] * lines[1]) + lines[2];
		distance = fabs(tmp)/sqrt(lines[0]*lines[0]+lines[1]*lines[1]);
		//tmp = (CV_MAT_ELEM(*line, float, i, 0)*normalP2[i][0]) + (CV_MAT_ELEM(*line, float, i, 1)*normalP2[i][1]) + CV_MAT_ELEM(*line, float, i, 2);
		//distance = fabs(tmp)/sqrt(CV_MAT_ELEM(*line, float, i, 0)*CV_MAT_ELEM(*line, float, i, 0) + CV_MAT_ELEM(*line, float, i, 1) * CV_MAT_ELEM(*line, float, i, 1));
		//printf("distance[%d] = %f\n", i,distance);
		
		if(distance <= 1.0f)
		{
			printf("\n[%d]fbsdistance = %9.8f\n", bPairNum, distance);
			bPairNum++;
		}
		else
		{}
	}
	
	return bPairNum;
}

//
// This function uses homography with CV_RANSAC (OpenCV 1.1)
// Won't compile on most linux distributions
//

//-------------------------------------------------------

//! Find homography between matched points and translate src_corners to dst_corners
//for mainMatch() function
int translateCorners(IpPairVec &matches, const CvPoint src_corners[4], CvPoint dst_corners[4])
{
#ifndef LINUX
  double h[9];
  CvMat _h = cvMat(3, 3, CV_64F, h);
  std::vector<CvPoint2D32f> pt1, pt2;
  CvMat _pt1, _pt2;
  
  int n = (int)matches.size();
  if( n < 4 ) return 0;

  // Set vectors to correct size
  pt1.resize(n);
  pt2.resize(n);

  // Copy Ipoints from match vector into cvPoint vectors
  for(int i = 0; i < n; i++ )
  {
    pt1[i] = cvPoint2D32f(matches[i].second.x, matches[i].second.y);
    pt2[i] = cvPoint2D32f(matches[i].first.x, matches[i].first.y);
  }
  _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
  _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );

  // Find the homography (transformation) between the two sets of points
  //if(!cvFindHomography(&_pt1, &_pt2, &_h, CV_RANSAC, 5))  // this line requires opencv 1.1
  if(!cvFindHomography(&_pt1, &_pt2, &_h))
    return 0;

  // Translate src_corners to dst_corners using homography
  for(int i = 0; i < 4; i++ )
  {
    double x = src_corners[i].x, y = src_corners[i].y;
    double Z = 1./(h[6]*x + h[7]*y + h[8]);
    double X = (h[0]*x + h[1]*y + h[2])*Z;
    double Y = (h[3]*x + h[4]*y + h[5])*Z;
    dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
  }
#endif
  return 1;
}


