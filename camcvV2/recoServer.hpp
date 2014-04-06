#ifndef RECO3_SERVER_
#define RECO3_SERVER_

 #include "opencv2/objdetect/objdetect.hpp"
 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"

 #include <iostream>
 #include <stdio.h>

#include "client.h"

 using namespace std;
 using namespace cv;

 /** Function Headers */
 void detectAndDisplay( Mat frame );
 int reco( Mat frame );

#endif /* RECO_SERVER_ */



