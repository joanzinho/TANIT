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
 //#include <vector>

 using namespace std;
 using namespace cv;

 /** Function Headers */
 int detectAndDisplay( Mat frame );
 int reco( Mat frame );
// int detectionMouvement(vector<Mat>& imgs, double& seuil);
// void initSeuil(vector<Mat>& imgs, double& seuil);
#endif /* RECO_HPP3_ */
