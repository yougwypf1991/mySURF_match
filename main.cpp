/*********************************************************** 
*  --- OpenSURF ---                                        *
*  This library is distributed under the GNU GPL. Please   *
*  contact chris.evans@irisys.co.uk for more information.  *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#include "surflib.h"
#include "kmeans.h"
#include <ctime>
#include <iostream>
#include <sys/time.h>
#include <stdio.h>

//-------------------------------------------------------
// In order to you use OpenSURF, the following illustrates
// some of the simple tasks you can do.  It takes only 1
// function call to extract described SURF features!
// Define PROCEDURE as:
//  - 1 and supply image path to run on static image
//  - 2 to capture from a webcam
//  - 3 to match find an object in an image (work in progress)
//  - 4 to display moving features (work in progress)
//  - 5 to show matches between static images
//#define PROCEDURE 1
#define PROCEDURE 5

//-------------------------------------------------------
/*
int mainImage(void);
int mainVideo(void);
int mainMatch(void);
int mainMotionPoints(void);
int mainKmeans(void);
*/

int mainStaticMatch(void);
//-------------------------------------------------------

int main(void) 
{
  if (PROCEDURE == 5) return mainStaticMatch();
	/*
  if (PROCEDURE == 1) return mainImage();
  if (PROCEDURE == 2) return mainVideo();
  if (PROCEDURE == 3) return mainMatch();
  if (PROCEDURE == 4) return mainMotionPoints();
  if (PROCEDURE == 6) return mainKmeans();
  */
}

//-------------------------------------------------------

int mainStaticMatch()
{
  IplImage *img1, *img2;
  IpVec ipts1, ipts2;
  IpPairVec matches;
/************for fin best threshold****************/
  IpPairVec qMatches;
  IpPairVec bestMatches;
  CvMat* points1 = NULL;//图像一中的点
  CvMat* points2 = NULL;//图像二中的点
  CvMat* FMat = NULL;//基础矩阵
  double pair1[12][2];
  double pair2[12][2];
  double normalPoint1[800][2];
  double normalPoint2[800][2];
  int size;
  int bestPairNum;
/************for fin best threshold****************/

  //img1 = cvLoadImage("Images/img1.jpg");
  //img2 = cvLoadImage("Images/img2.jpg");
  img1 = cvLoadImage("Images/img4.jpg");
  img2 = cvLoadImage("Images/img5.jpg");

  /*
   * for time count global writed by kanglu at 2015 11 17
   */
  struct timeval start_g, end_g;
  int interval_g;

  gettimeofday(&start_g, NULL);

  surfDetDes(img1,ipts1,false,4,4,2,0.0006f);
  surfDetDes(img2,ipts2,false,4,4,2,0.0006f);

  /************for fin best threshold****************/
  getBestMatches(ipts1, ipts2, bestMatches, pair1, pair2); //计算精确匹配点



  FMat = computeFMat(points1, points2, pair1, pair2);//通过精确匹配点计算基础矩阵
  /*
  printf("\nF Matrix after computeF=");
  PrintMat(FMat);
  */
 
  //计算正常阈值下的匹配点对normalPoint1[][] <-> normalPoint2[][]
  getMatches(ipts1,ipts2,normalPoint1,normalPoint2,matches);
  /*
  printf("\nF Matrix after getMatches=");
  PrintMat(FMat);
  */

  size = matches.size(); 
  
  //计算通过对极几何优化的匹配点对的数目
  bestPairNum = qualifiedMatches(FMat,qMatches,normalPoint1,normalPoint2,size);
  printf("\nbetsPairNum = %d \n", bestPairNum);

  gettimeofday(&end_g, NULL);
  interval_g = 1000000*(end_g.tv_sec - start_g.tv_sec) + (end_g.tv_usec - start_g.tv_usec);
  printf("\nGlobal count time: %f ms\n", interval_g/1000.0);

  printf("\nFind %d interest points in img1. \n", ipts1.size());
  printf("Find %d interest points in img2. \n", ipts2.size());
  printf("Find %d interest points pairs. \n", matches.size());

  for (unsigned int i = 0; i < matches.size(); ++i)
  {
    drawPoint(img1,matches[i].first);
    drawPoint(img2,matches[i].second);
  
    int w = img1->width;
    cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
    cvLine(img2,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
  }
  cvNamedWindow("1", CV_WINDOW_AUTOSIZE );
  cvNamedWindow("2", CV_WINDOW_AUTOSIZE );
  cvShowImage("1", img1);
  cvShowImage("2",img2);

  cvWaitKey(0);

  return 0;
}

/*

//-------------------------------------------------------


int mainImage(void)
{
  // Declare Ipoints and other stuff
  IpVec ipts;
  IplImage *img=cvLoadImage("Images/img1.jpg");

  // Detect and describe interest points in the image
  surfDetDes(img, ipts, true, 3, 4, 2, 0.0004f);

  // Draw the detected points
  drawIpoints(img, ipts);

  // Display the result
  showImage(img);

  return 0;
}


//-------------------------------------------------------


int mainVideo(void)
{
  // Initialise capture device
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // Create a window 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  // Declare Ipoints and other stuff
  IpVec ipts;
  IplImage *img=NULL;

  // Main capture loop
  while( 1 ) 
  {
    // Grab frame from the capture source
    img = cvQueryFrame(capture);

    // Extract surf points
    surfDetDes(img, ipts, true, 3, 4, 2, 0.0004f);    

    // Draw the detected points
    drawIpoints(img, ipts);

    // Draw the FPS figure
    drawFPS(img);

    // Display the result
    cvShowImage("OpenSURF", img);

    // If ESC key pressed exit loop
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}


//-------------------------------------------------------


int mainMatch(void)
{
  // Initialise capture device
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // Create a window 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  // Declare Ipoints and other stuff
  IpPairVec matches;
  IpVec ipts, ref_ipts;
  
  // This is the reference object we wish to find in video frame
  // Replace the line below with IplImage *img = cvLoadImage("Images/object.jpg"); 
  // where object.jpg is the planar object to be located in the video
  IplImage *img = NULL; 
  CvPoint src_corners[4] = {{0,0}, {img->width,0}, {img->width, img->height}, {0, img->height}};
  CvPoint dst_corners[4];

  // Extract reference object Ipoints
  surfDetDes(img, ref_ipts, false, 4, 4, 2, 0.0002f);
  drawIpoints(img, ref_ipts);
  showImage(img);

  // Main capture loop
  while( 1 ) 
  {
    // Grab frame from the capture source
    img = cvQueryFrame(capture);
     
    // Detect and describe interest points in the frame
    surfDetDes(img, ipts, false, 4, 4, 2, 0.0002f);

    // Fill match vector
    getMatches(ipts,ref_ipts,matches);
    
    // This call finds where the object corners should be in the frame
    if (translateCorners(matches, src_corners, dst_corners))
    {
      // Draw box around object
      for(int i = 0; i < 4; i++ )
      {
        CvPoint r1 = dst_corners[i%4];
        CvPoint r2 = dst_corners[(i+1)%4];
        cvLine( img, cvPoint(r1.x, r1.y),
          cvPoint(r2.x, r2.y), cvScalar(255,255,255), 3 );
      }

      for (unsigned int i = 0; i < matches.size(); ++i)
        drawIpoint(img, matches[i].first);
    }

    // Draw the FPS figure
    drawFPS(img);

    // Display the result
    cvShowImage("OpenSURF", img);

    // If ESC key pressed exit loop
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  // Release the capture device
  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}


//-------------------------------------------------------


int mainMotionPoints(void)
{
  // Initialise capture device
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // Create a window 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  // Declare Ipoints and other stuff
  IpVec ipts, old_ipts, motion;
  IpPairVec matches;
  IplImage *img;

  // Main capture loop
  while( 1 ) 
  {
    // Grab frame from the capture source
    img = cvQueryFrame(capture);

    // Detect and describe interest points in the image
    old_ipts = ipts;
    surfDetDes(img, ipts, true, 3, 4, 2, 0.0004f);

    // Fill match vector
    getMatches(ipts,old_ipts,matches);
    for (unsigned int i = 0; i < matches.size(); ++i) 
    {
      float dx = matches[i].first.dx;
      float dy = matches[i].first.dy;
      float speed = sqrt(dx*dx+dy*dy);
      if (speed > 5 && speed < 30) 
        drawIpoint(img, matches[i].first, 3);
    }
        
    // Display the result
    cvShowImage("OpenSURF", img);

    // If ESC key pressed exit loop
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  // Release the capture device
  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}


//-------------------------------------------------------

int mainKmeans(void)
{
  IplImage *img = cvLoadImage("Images/img1.jpg");
  IpVec ipts;
  Kmeans km;
  
  // Get Ipoints
  surfDetDes(img,ipts,true,3,4,2,0.0006f);

  for (int repeat = 0; repeat < 10; ++repeat)
  {

    IplImage *img = cvLoadImage("Images/img1.jpg");
    km.Run(&ipts, 5, true);
    drawPoints(img, km.clusters);

    for (unsigned int i = 0; i < ipts.size(); ++i)
    {
      cvLine(img, cvPoint(ipts[i].x,ipts[i].y), cvPoint(km.clusters[ipts[i].clusterIndex].x ,km.clusters[ipts[i].clusterIndex].y),cvScalar(255,255,255));
    }

    showImage(img);
  }

  return 0;
}
*/
