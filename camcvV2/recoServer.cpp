#include "reco3.hpp"

 /** Global variables */
 String cascade_name = "cascade.xml";
 CascadeClassifier cascade;
 string window_name = "Capture - Face detection";
 RNG rng(12345);

 /** @function main */
 int reco( Mat frame ){
	 cout << "3" << endl;
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
int detectAndDisplay( Mat frame ){
	cout << "4" << endl;
  std::vector<Rect> faces;
  Mat frame_gray;
  cout<<frame.channels()<<endl;
  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  if(faces.size()>0){
    cout << "Logo détecté" << endl;
    return 1;
  }
  else{return 0;}
}
