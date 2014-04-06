/*
 * main.cpp
 *
 *  Created on: 14 mars 2014
 *      Author: jean
 */

    /*------------------------------------------------*/
    #include <opencv/cv.h>
	#include <opencv/highgui.h>
	#include <stdio.h>
	#include <iostream>

	#include <vector>
	#include "opencv2/imgproc/imgproc.hpp"

    /*------ Declaration des variables globales ------*/
    char key2;
    CvHaarClassifierCascade *cascade;
    CvMemStorage *storage;
    /*---------- Declaration des fonctions -----------*/
    void detectFaces(IplImage *img);
    /*------------------------------------------------*/

    using namespace std;
    using namespace cv;

    int reco2(IplImage *img)
    {

        Mat testImg(Size(512, 512), CV_8UC1 ); // Create an image
        testImg.setTo(Scalar(255)); // Fill the image with white
        Mat erodedImg;
        erode(testImg, erodedImg, Mat(Size(30, 30), CV_8UC1)); // Erode with a 30 x 30 kernel

        const char *filename = "haarcascade_frontalface_default.xml";
        //const char *filename = "/home/jean/workspaceCPP/VetJWebcam/cascade.xml";
        /* Chargement du classifieur */
        cascade = (CvHaarClassifierCascade*)cvLoadHaarClassifierCascade( filename, cvSize(24, 24) );

        // Ouverture d'un fichier video
        //capture = cvCreateFileCapture("/home/lobodol/Telechargements/cam.avi");

        /* Initialisation de l'espace memoire */
        storage = cvCreateMemStorage(0);

        /* Creation d'une fenetre */
        cvNamedWindow("Window-FT", 1);
        detectFaces(img);

        /* Liberation de l'espace memoire*/
        cvDestroyWindow("Window-FT");
        cvReleaseHaarClassifierCascade(&cascade);
        cvReleaseMemStorage(&storage);

        return 0;
    }
    /*------------------------------------------------*/
    void detectFaces(IplImage *img)
    {
        /* Declaration des variables */
        int i;
        CvSeq *faces = cvHaarDetectObjects(img, cascade, storage, 1.1, 3, 0, cvSize(40,40));

        for(i=0; i<(faces?faces->total:0); i++)
        {
            CvRect *r = (CvRect*)cvGetSeqElem(faces, i);
            cvRectangle(img, cvPoint(r->x, r->y), cvPoint(r->x + r->width, r->y + r->height), CV_RGB(255, 0, 0), 1, 8, 0);
            cout << "Visage " << i+1 <<" détecté" << endl;
        }

        cvShowImage("Window-FT", img);
    }


