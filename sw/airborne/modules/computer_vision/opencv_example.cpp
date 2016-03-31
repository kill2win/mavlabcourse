#include <iostream>
#include <ctype.h>
#include "opencv_example.h"
//#include <types.hpp>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/opencv.hpp>
using namespace cv;


int thresh = 200;
int max_thresh = 255;
const int MAX_COUNT = 500;

Mat image,dst, dst_norm, dst_norm_scaled, prevImage, bgr, test;

int opencv_example(char* img, int width, int height)
{
	// Create a new image, using the original bebop image.
	Mat M(width,height, CV_8UC2, img);
	vector<Point2f> points[2];
	//Mat image,dst, dst_norm, dst_norm_scaled, prevImage;
	dst = Mat::zeros( M.size(), CV_32FC1 );
	TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

	Size winSize(31,31), subPixWinSize(10,10);
	// If you want a color image, uncomment this line
	// cvtColor(M, image, CV_YUV2RGB_Y422);
	// For a grayscale image, use this one
	//cvtColor(M, bgr, CV_YUV2RGB);
	//cvtColor(bgr, test , COLOR_BGR2GRAY);
	cvtColor(M, image, CV_YUV2GRAY_Y422);

	// Blur it, because we can
	//blur(image, image, Size(5,5));

	// Canny edges, only works with grayscale image
	//int edgeThresh=35;
	int blockSize = 2;
	int apertureSize = 3;
	double k = 0.04;
	//Canny(image, image, edgeThresh, edgeThresh*3);
	//cornerHarris(image, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
	/// Normalizing
	//normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
	//convertScaleAbs( dst_norm, dst_norm_scaled );
	/// Drawing a circle around corners
	/// Drawing a circle around corners
	/*
	for( int j = 0; j < dst_norm.rows ; j++ )
		{ for( int i = 0; i < dst_norm.cols; i++ )
			{
				if( (int) dst_norm.at<float>(j,i) > thresh )
				{
					circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
				}
			}
		}
	*/
	
	if(prevImage.empty()) 
				image.copyTo(prevImage);
	goodFeaturesToTrack(prevImage, points[0], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
	//cornerSubPix(prevImage, points[0], subPixWinSize, Size(-1,-1), termcrit);
	
	if( !points[0].empty() )
		{
			vector<uchar> status;
			vector<float> err;
			calcOpticalFlowPyrLK(prevImage, image, points[0], points[1], status, noArray(), winSize, 3, termcrit, 0, 0.001);
			size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                /*if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 5 )
                    {
                        addRemovePt = false;
                        continue;
                    }
                }*/

               // if( !status[i] )
                 //   continue;

                //points[0][k++] = points[1][i];
                //circle( image, points[0][i], 3, Scalar(0,255,0), -1, 8);
		//circle( image, points[0][i], 5,  Scalar(0), 2, 8, 0 );
            }
            
            //points[1].resize(k);
            /*
            if( points[1].size() < (size_t)MAX_COUNT )
        	{
            	vector<Point2f> tmp;
            	//tmp.push_back(point);
            	//cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
            	points[1].push_back(tmp[0]);
            	//addRemovePt = false;
        	}*/

		}
  
  image.copyTo(prevImage);
	
		
		
	//image = dst_norm_scaled;
	// Convert back to YUV422, and put it in place of the original image
	for (int row=0; row <height; row++){
		for (int col=0; col <width; col++){
			img[(row*width+col)*2+1] = image.at<uint8_t>(row,col);
			img[(row*width+col)*2] = 127;
		}
	}
	return 0;
}
