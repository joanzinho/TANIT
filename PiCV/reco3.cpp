#include "reco3.hpp"




 /** Global variables */
 String cascade_name = "LogoEnstaV1.xml";
 CascadeClassifier cascade;
 string window_name = "Capture - Face detection";
 RNG rng(12345);



 /*
  * Algorithme détectant la présence ou non de logo.
  * S'il détecte un logo, renvoi 1, sinon, renvoi 0.
  *
  */
 int reco( Mat frame ){
   //-- 1. Load the cascades
   if( !cascade.load( cascade_name ) ){ printf("Error loading\n"); return -1; };

   //-- 2. Apply the classifier to the frame
   if( !frame.empty() )
   { return(detectAndDisplay( frame )); }
   else{
	   printf(" --(!) No captured frame -- Break!");
	   return(0);
   }
 }

/** @function detectAndDisplay */
int detectAndDisplay( Mat frame )
{
  std::vector<Rect> faces;
  Mat frame_gray=frame;
  // ATTENTION : Image sur un seul canal en noir et blanc!!
  //cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  if(faces.size()>0){return 1;}
  else{return 0;}
}

/*
 *	Fonction détectant un mouvement sur la moyenne des 30 dernières frames
 *	1 => mouvement détecté.
 *	0 => pas de mouvment.
 */
/*int detectionMouvement(vector<Mat>& imgs, double& seuil){
	int i=0;
	Mat diff;
	double sommeDetect=0;
	for(i=0; i<imgs.size()-1; i++){
		cv::absdiff(imgs[i], imgs[i+1], diff);
		sommeDetect=sommeDetect+(cv::sum(diff)[0]+cv::sum(diff)[1]+cv::sum(diff)[2]+cv::sum(diff)[3]);
		cout << sommeDetect << endl;
	}
	if(sommeDetect<=(imgs.size()-1)*seuil){
		return 0;
	}
	return 1;
}*/

/*void initSeuil(vector<Mat>& imgs, double& seuil){
	int i=0;
	Mat diff;
	double sommeDetect=0;
	for(i=0; i<imgs.size()-1; i++){
		cv::absdiff(imgs[i], imgs[i+1], diff);
		sommeDetect=sommeDetect+(cv::sum(diff)[0]+cv::sum(diff)[1]+cv::sum(diff)[2]+cv::sum(diff)[3]);
	}
	seuil=(sommeDetect*1.1)/(imgs.size()-1);
	cout << sommeDetect << endl;
	cout << seuil << endl;
}*/


