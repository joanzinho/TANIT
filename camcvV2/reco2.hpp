/*
 * reco2.hpp
 *
 *  Created on: 25 mars 2014
 *      Author: jean
 */

#ifndef RECO2_HPP_
#define RECO2_HPP_


    #include <opencv/cv.h>
	#include <opencv/highgui.h>
	#include <stdio.h>
	#include <iostream>

	#include <vector>
	#include "opencv2/imgproc/imgproc.hpp"
    void detectFaces(IplImage *img);
    int reco2(IplImage *img);
    /*------------------------------------------------*/

    using namespace std;
    using namespace cv;


#endif /* RECO2_HPP_ */
