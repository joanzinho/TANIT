/*
 * reco3.hpp
 *
 *  Created on: 1 avr. 2014
 *      Author: jean
 */




#ifndef RECO3_HPP_
#define RECO3_HPP_

 #include "opencv2/objdetect/objdetect.hpp"
 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"

 #include <iostream>
 #include <stdio.h>

 using namespace std;
 using namespace cv;

 /** Function Headers */
 void detectAndDisplay( Mat frame );
 int reco( Mat frame );

#endif /* RECO_HPP3_ */
