Programme de reconnaissance du logo ENSTA Bretagne à l'aide de la caméra du Raspberry Pi (V1)

Cette version du programme contient : 

-Un classifieur reconnaissant le logo ENSTA Bretagne (LogoEnstaV1.xml)
-Les source de l'algo OpenCV implémentant Viola Et Jones (reco.cpp, reco.hpp)
-L'algo OpenCV implémentant Viola Et Jones modifié pour les besoin de RaspiCam (reco3.cpp, reco3.hpp)
-Les fichiers source permettant d'utiliser les ressources de la RaspCam
-Le fichier source permettant d'avoir accès à l'image noire et blanc de la caméra. Il faut insérer les fonction de traitement dans ce fichier (camcvvoid.cpp)
-Le fichier précédent, modifié pour permettre d'appliquer V&J à la vidéo capturée par la caméra. (camcvtest.cpp)
-Le fichier CMakeList permet de générer le Makefile du programme camcv.

Le programme camcv ainsi généré montre un fenêtre affichant la vidéo. Quand le logo est détecté, il est cerclé de gris.

