#include "client.h"



/*
 * Function to manage network aspect, connect with the server, and disconnect.
 */
int init_client(socket *sock, char data){
    #if defined (WIN32)
        WSADATA WSAData;
        int erreur = WSAStartup(MAKEWORD(2,2), &WSAData);
    #else
        int erreur = 0;
    #endif

    SOCKADDR_IN sin;

    char buffer[LENGTH_BUFFER];
    buffer=sprintf("%c",data);

    /* Si les sockets Windows fonctionnent */
    if(!erreur)
    {
        /* Création de la socket */
        *sock = socket(AF_INET, SOCK_STREAM, 0);

        /* Configuration de la connexion */
        sin.sin_addr.s_addr = inet_addr("192.168.1.1");
        sin.sin_family = AF_INET;
        sin.sin_port = htons(PORT);

        /* Si l'on a réussi à se connecter */
        if(connect(&sock, (SOCKADDR*)&sin, sizeof(sin)) != SOCKET_ERROR)
        {
            printf("Connexion réussie.\n");
        	send(sock, buffer, LENGTH_BUFFER, 0);
        	// recv(sock, buffer, LENGTH_BUFFER, 0);

        }
        /* sinon, on affiche "Impossible de se connecter" */
        else
        {
            printf("Impossible de se connecter\n");
            return -1;		// TODO Tenir compte de cette réponse
        }
        closesocket(sock);

        #if defined (WIN32)
            WSACleanup();
        #endif
    }
    return EXIT_SUCCESS;
}




















