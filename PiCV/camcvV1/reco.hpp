/*
 * reco.hpp
 *
 *  Created on: 25 mars 2014
 *      Author: jean
 */

#ifndef RECO_HPP_
#define RECO_HPP_

 #include "opencv2/objdetect/objdetect.hpp"
 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"

 #include <iostream>
 #include <stdio.h>

 using namespace std;
 using namespace cv;

 /** Function Headers */
 void detectAndDisplay( Mat frame );
 int reco(Mat frame);


#endif /* RECO_HPP_ */
