#include "reco3.hpp"

 /** Global variables */
 String cascade_name = "LogoEnstaV1.xml";
 CascadeClassifier cascade;
 string window_name = "Capture - Face detection";
 RNG rng(12345);

 /** @function main */
 int reco( Mat frame ){
   //-- 1. Load the cascades
   if( !cascade.load( cascade_name ) ){ printf("Error loading\n"); return -1; };

   //-- 2. Apply the classifier to the frame
   if( !frame.empty() )
   { detectAndDisplay( frame ); }
   else{
	   printf(" --(!) No captured frame -- Break!");
   }
   return 0;
 }

/** @function detectAndDisplay */
void detectAndDisplay( Mat frame )
{
  std::vector<Rect> faces;
  Mat frame_gray=frame;
  // ATTENTION : Image sur un seul canal en noir et blanc!!
  //cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  for( size_t i = 0; i < faces.size(); i++ )
  {
    Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
    ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

    Mat faceROI = frame_gray( faces[i] );
    std::vector<Rect> eyes;


    for( size_t j = 0; j < eyes.size(); j++ )
     {
       Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
       int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
       circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
     }
  }
  //-- Show what you got
  imshow( window_name, frame );
}
