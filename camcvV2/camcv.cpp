/////////////////////////////////////////////////////////////
// Many source code lines are copied from RaspiVid.c
// Copyright (c) 2012, Broadcom Europe Ltd
// 
// Lines have been added by Pierre Raufast - June 2013
// pierre.raufast@gmail.com
// to work with OpenCV 2.3
// visit thinkrpi.wordpress.com
//
// Modified by Cyril Rouvière - August 2013
// cyril.rouviere@ensta-bretagne.fr
// to make an optical odometry
/////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <cstring>
#include <ctime>
#include <cstdlib>
#include <list> 
#include <memory.h>
#include <math.h>
#include "time.h"

#include <iostream>
#include <fstream>
#include <sstream>

    #include <opencv/cv.h>
	#include <opencv/highgui.h>
	#include <stdio.h>

#include <cv.h>
#include <highgui.h>

extern "C" {
	#include "bcm_host.h"
	#include "interface/vcos/vcos.h"
	
	#include "interface/mmal/mmal.h"
	#include "interface/mmal/mmal_logging.h"
	#include "interface/mmal/mmal_buffer.h"
	#include "interface/mmal/util/mmal_util.h"
	#include "interface/mmal/util/mmal_util_params.h"
	#include "interface/mmal/util/mmal_default_components.h"
	#include "interface/mmal/util/mmal_connection.h"
	
	#include "RaspiCamControl.h"
	#include "RaspiPreview.h"
	#include "RaspiCLI.h"

}

#include <semaphore.h>

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>

using namespace cv;
using namespace std;


#define sign(n) ( (n) < 0 ? -1 : ( (n) > 1 ? 1 : 0 ) )

// Paramètre du TCP
#define PORT		55042
#define BUFFER_LEN	8
#define INVALID_SOCKET	-1
#define SOCKET_ERROR	-1
#define closesocket(s)	close(s)

// Paramètres de tracking
#define TAILLE_POD			200
#define ECART_MAX			400
#define COEFF_MULT_MOY			1000
#define NB_MAX_AMERS			10
#define NB_COLS_GRID			8
#define NB_ROWS_GRID			5

// Paramètres standards des ports caméra
#define MMAL_CAMERA_VIDEO_PORT		1
#define MMAL_CAMERA_CAPTURE_PORT	2

// Informations du format video
#define LARGEUR				160
#define HAUTEUR				120
#define VIDEO_FRAME_RATE_NUM		30
#define VIDEO_FRAME_RATE_DEN		1
#define MAX_BITRATE			30000000 // 30Mbits/s

// Le rendu video nécessite au moins deux buffers
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Matrices images
cv::Mat img_cam, img_prev, img_next, img_next_brute, img_show, img_pod, rot_mat;

// Pour convertir de I420 à IplImage
IplImage *py;

// OpenCV impose l'utilisation de "vector" et non de "list" ...
vector<cv::Point2f>	amers;	// points à repérer N-1
vector<cv::Point2f>	nv;	// points à repérer N
vector<uchar>		status;	// si les points ont été repérés
vector<float>		err;	// erreurs (inutilisé)
vector<cv::Point2f>	grid;	// grille manuelle de points à repérer

cv::Scalar rouge(0, 0, 255);
cv::Size dim_1(1, 1);
cv::Size flou_kern(3, 3);

cv::TermCriteria critere(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 50, 0.03);
cv::Point pod_centre(TAILLE_POD / 2, TAILLE_POD / 2);
cv::Point2f img_centre(LARGEUR / 2, HAUTEUR / 2);

char key;
int nCount = 0;			// nombre de frames capturées
int somme_x, somme_y, nb_vect;	// somme des vecteurs du champ dense
int moy_vect_x, moy_vect_y;	// déplacement moyen selon x et y
double angle_prev = 0.;		// angle boussole de l'image N-1
double angle_next = 0.;		// angle boussole de l'image N

int mmal_status_to_int(MMAL_STATUS_T status);

/*------ Declaration des variables globales ------*/
char key2;
CvHaarClassifierCascade *cascade;
CvMemStorage *storage;
/*---------- Declaration des fonctions -----------*/
void detectFaces(IplImage *img);
/*------------------------------------------------*/


// Pour le socket TCP
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;
struct MSG{
	char header;
	int data;
};
SOCKET sock;
char buffer[BUFFER_LEN];
MSG transf;


// Informations d'état sur l'exécution en cours
typedef struct{

	int timeout;		// Durée d'exécution en millisecondes
	int width;		// Largeur d'image demandée
	int height;		// Hauteur d'image demandée
	int bitrate;		// Bitrate demandé
	int framerate;		// FPS demandé

	RASPIPREVIEW_PARAMETERS preview_parameters;	// Paramètres d'aperçu
	RASPICAM_CAMERA_PARAMETERS camera_parameters;	// Paramètres de caméra

	MMAL_COMPONENT_T *camera_component;	// Pointeur vers composant camera
	MMAL_COMPONENT_T *encoder_component;	// Pointeur vers composant encodeur
	MMAL_CONNECTION_T *preview_connection;	// Pointeur vers connexion caméra->aperçu
	MMAL_CONNECTION_T *encoder_connection;	// Pointeur vers connexion caméra->encodeur
	MMAL_POOL_T *video_pool;		// Pointeur vers le pool des buffers
   
} RASPIVID_STATE;


// Pour passer les information du port encodeur vers le callback
typedef struct{
	FILE *file_handle;			// Fichier d'écriture du buffer (inutilisé)
	VCOS_SEMAPHORE_T complete_semaphore;	// Semaphore de fin de capture d'une frame
	RASPIVID_STATE *pstate;			// Pointeur vers l'état du programme
} PORT_USERDATA;

//////// MORCEAU RECONNAISSANCE FACIALE ///////

/*int reco(void)
    {
         Declaration des variables
        CvCapture *capture;
        IplImage *img;
        const char *filename = "/opt/vc/haarcascade_frontalface_default.xml";
        //const char *filename = "/opt/vc/cascade.xml";


         Chargement du classifieur
        cascade = (CvHaarClassifierCascade*)cvLoadHaarClassifierCascade( filename, cvSize(24, 24) );

         Ouverture du flux video de la camera
        capture = cvCreateCameraCapture(0);
        // Ouverture d'un fichier video
        //capture = cvCreateFileCapture("/home/lobodol/Telechargements/cam.avi");

         Initialisation de l'espace memoire
        storage = cvCreateMemStorage(0);

         Creation d'une fenetre
        cvNamedWindow("Window-FT", 1);

         Boucle de traitement
        while(key2 != 'q')
        {
            img = cvQueryFrame(capture);
            detectFaces(img);
            key2 = cvWaitKey(10);
        }

         Liberation de l'espace memoire
        cvReleaseCapture(&capture);
        cvDestroyWindow("Window-FT");
        cvReleaseHaarClassifierCascade(&cascade);
        cvReleaseMemStorage(&storage);

        return 0;
    }*/
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
        }

        cvShowImage("Window-FT", img);
    }

////// FIN RECO FACIALE    //////




// Récupérer une frame (*port : d'où vient le callback, *buffer : en-tête du buffer mmal)
static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer){

	MMAL_BUFFER_HEADER_T *new_buffer;
	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

	if(pData){
		if(buffer->length){

			mmal_buffer_header_mem_lock(buffer);

			int w = pData->pstate->width;
			int h = pData->pstate->height;

			// Lire Y de YUV
			memcpy(py->imageData,buffer->data,w*h);
			img_cam = cvarrToMat(py);

			cout << "Je suis là!!!" << endl ;

			/* Declaration des variables */
			const char *filename = "/opt/vc/haarcascade_frontalface_default.xml";
			//const char *filename = "/opt/vc/cascade.xml";


			/* Chargement du classifieur */
			cascade = (CvHaarClassifierCascade*)cvLoadHaarClassifierCascade( filename, cvSize(24, 24) );

			/* Initialisation de l'espace memoire */
			storage = cvCreateMemStorage(0);

			/* Creation d'une fenetre */
			cvNamedWindow("Window-FT", 1);

			/* Boucle de traitement */
			while(key2 != 'q')
			{
				detectFaces((IplImage*)img_cam);
				key2 = cvWaitKey(10);
			}

			/* Liberation de l'espace memoire*/
			cvReleaseCapture(&capture);
			cvDestroyWindow("Window-FT");
			cvReleaseHaarClassifierCascade(&cascade);
			cvReleaseMemStorage(&storage);

			key = (char) waitKey(1);
			nCount++;
		
			mmal_buffer_header_mem_unlock(buffer);

		}
		else{vcos_log_error("buffer null");}
	}
	else{vcos_log_error("Received a encoder buffer callback with no state");}

	// Remettre le buffer dans le pool
	mmal_buffer_header_release(buffer);

	// Et en renvoyer un dans le port
	if(port->is_enabled){
		MMAL_STATUS_T status;
		new_buffer = mmal_queue_get(pData->pstate->video_pool->queue);
		if(new_buffer){status = mmal_port_send_buffer(port, new_buffer);}
		if(!new_buffer || status != MMAL_SUCCESS){vcos_log_error("Unable to return a buffer to the encoder port");}
	}

}


// Créer le composant caméra (*state : état du programme)
// Renvoie : 0 si échec ; pointeur vers le composant si réussite
static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state){

	MMAL_COMPONENT_T *camera = 0;
	MMAL_ES_FORMAT_T *format;
	MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
	MMAL_STATUS_T status;

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

	if(status != MMAL_SUCCESS){
		vcos_log_error("Failed to create camera component");
		goto error;
	}
	
	if(!camera->output_num){
		vcos_log_error("Camera doesn't have output ports");
		goto error;
	}

	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

	// Configuration de la caméra
	{
		MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
		{
			{ MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
			cam_config.max_stills_w =				state->width,
			cam_config.max_stills_h =				state->height,
			cam_config.stills_yuv422 =				0,
			cam_config.one_shot_stills =				0,
			cam_config.max_preview_video_w =			state->width,
			cam_config.max_preview_video_h =			state->height,
			cam_config.num_preview_video_frames =			3,
			cam_config.stills_capture_circular_buffer_height =	0,
			cam_config.fast_preview_resume =			0,
			cam_config.use_stc_timestamp =				MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
		};
		mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}

	// Configurer le format d'encodage sur le port video
	format = video_port->format;
	format->encoding_variant =		MMAL_ENCODING_I420;
	format->encoding =			MMAL_ENCODING_I420;
	format->es->video.width =		state->width;
	format->es->video.height =		state->height;
	format->es->video.crop.x =		0;
	format->es->video.crop.y =		0;
	format->es->video.crop.width =		state->width;
	format->es->video.crop.height =		state->height;
	format->es->video.frame_rate.num =	state->framerate;
	format->es->video.frame_rate.den =	VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(video_port);
	if(status){
		vcos_log_error("camera video format couldn't be set");
		goto error;
	}
	
	status = mmal_port_enable(video_port, video_buffer_callback);
	if(status){
		vcos_log_error("camera video callback2 error");
		goto error;
	}

	// Vérifie qu'il y a assez de buffer pour éviter de sauter des frames
	if(video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM){
		video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
	}

	// Configure le format d'encodage sur le port still
	format = still_port->format;
	format->encoding =			MMAL_ENCODING_OPAQUE;
	format->encoding_variant =		MMAL_ENCODING_I420;
	format->es->video.width =		state->width;
	format->es->video.height =		state->height;
	format->es->video.crop.x =		0;
	format->es->video.crop.y =		0;
	format->es->video.crop.width =		state->width;
	format->es->video.crop.height =		state->height;
	format->es->video.frame_rate.num =	1;
	format->es->video.frame_rate.den =	1;

	status = mmal_port_format_commit(still_port);
	if(status){
		vcos_log_error("camera still format couldn't be set");
		goto error;
	}

	// Créer le pool de messages sur le port video
	MMAL_POOL_T *pool;
	video_port->buffer_size = video_port->buffer_size_recommended;
	video_port->buffer_num = video_port->buffer_num_recommended;
	pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
	if(!pool){vcos_log_error("Failed to create buffer header pool for video output port");}
	state->video_pool = pool;

	// Vérifie qu'il y a assez de buffer pour éviter de sauter des frames
	if(still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM){
		still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
	}

	// Activer le composant
	status = mmal_component_enable(camera);

	if(status){
		vcos_log_error("camera component couldn't be enabled");
		goto error;
	}
	
	raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);
	state->camera_component = camera;

	return camera;

error:

	if(camera){mmal_component_destroy(camera);}
	return 0;

}


// Désactiver la caméra (*state : état du programme)
static void destroy_camera_component(RASPIVID_STATE *state){
	if(state->camera_component){
		mmal_component_destroy(state->camera_component);
		state->camera_component = NULL;
	}
}


// Désactiver l'encodeur (*state : état du programme)
static void destroy_encoder_component(RASPIVID_STATE *state){
	if(state->video_pool){
		mmal_port_pool_destroy(state->encoder_component->output[0], state->video_pool);
	}
	if(state->encoder_component){
		mmal_component_destroy(state->encoder_component);
		state->encoder_component = NULL;
	}
}

// Connecter *output_port et *input_port (**connection : connexion mmal de sortie)
// Retourne le statut de connexion
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection){
	MMAL_STATUS_T status;
	status = mmal_connection_create(
		connection,
		output_port,
		input_port,
		MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
	if (status == MMAL_SUCCESS){
		status =  mmal_connection_enable(*connection);
		if (status != MMAL_SUCCESS){
			mmal_connection_destroy(*connection);
		}
	}
	return status;
}


// Désactive un port
static void check_disable_port(MMAL_PORT_T *port){
	if(port && port->is_enabled){mmal_port_disable(port);}
}


// Signal signint
static void signal_handler(int signal_number){
	vcos_log_error("Aborting program\n");
	exit(255);
}


// Main
int main(int argc, const char **argv){
	cout << "1" << endl;
	RASPIVID_STATE state;
	
	MMAL_STATUS_T status;
	MMAL_PORT_T *camera_video_port =	NULL;
	MMAL_PORT_T *camera_still_port =	NULL;
	MMAL_PORT_T *preview_input_port =	NULL;
	MMAL_PORT_T *encoder_input_port =	NULL;
	MMAL_PORT_T *encoder_output_port =	NULL;

	time_t timer_begin, timer_end;
	double secondsElapsed;
	signal(SIGINT, signal_handler);

	// Initialiser l'IplImage et la grille manuelle de tracking
	int w = state.width;
	int h = state.height;
	py = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
	cout << "2" << endl;
	int dx = w / (NB_COLS_GRID + 1);
	int dy = h / (NB_ROWS_GRID + 1);
	for(int i = 1; i < NB_COLS_GRID + 1; i++){
		for(int j = 1; j < NB_ROWS_GRID + 1; j++){
			grid.push_back(cv::Point2f(i * dx, j * dy));
		}
	}

	cout << "3" << endl;

	// Activer la caméra
	if(!create_camera_component(&state)){
		vcos_log_error("%s: Failed to create camera component", __func__);
	}
	else if((status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS){
		vcos_log_error("%s: Failed to create preview component", __func__);
		destroy_camera_component(&state);
	}
	else{

		PORT_USERDATA callback_data;

		camera_video_port = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
		camera_still_port = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];

		VCOS_STATUS_T vcos_status;

		callback_data.pstate = &state;

		vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
		vcos_assert(vcos_status == VCOS_SUCCESS);

		camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;

  		time(&timer_begin); 

		// Démarrer la capture
		if(mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS){
			return 0;
		}

		// Envoyer les buffers vers le port video
		int num = mmal_queue_length(state.video_pool->queue);
		int q;
		for(q=0;q<num;q++){
			MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.video_pool->queue);
			if(!buffer){vcos_log_error("Unable to get a required buffer %d from pool queue", q);}
			if(mmal_port_send_buffer(camera_video_port, buffer)!= MMAL_SUCCESS){
				vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
			}
		}

		// Now wait until we need to stop
		vcos_sleep(state.timeout);
  
		//mmal_status_to_int(status);
		// Disable all our ports that are not handled by connections
		check_disable_port(camera_still_port);
		
		if (state.camera_component)
		   mmal_component_disable(state.camera_component);
		
		//destroy_encoder_component(&state);
		raspipreview_destroy(&state.preview_parameters);
		destroy_camera_component(&state);
		
		}
	if (status != 0)
	raspicamcontrol_check_configuration(128);
	
	time(&timer_end);  // get current time; same as: timer = time(NULL) 
	cvReleaseImage(&py);
	
	secondsElapsed = difftime(timer_end,timer_begin);
	
	printf ("%.f seconds for %d frames : FPS = %f\n", secondsElapsed,nCount,(float)((float)(nCount)/secondsElapsed));
		
   return 0;
}

