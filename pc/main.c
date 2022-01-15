/*
 * Aplicacion de interfaz de usuario con el NUCLEO board para la asignatura
 * de Tiempo Real en Sistemas Empotrados
 *
 * Sonia Bartolome, Jose Mari Irizar y Gorka Irureta
 *
 */

#include "main.h"
#include <time.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/stat.h>


static volatile int keepRunning = 1;

void intHandler(int dummy) { //Control + C
    keepRunning = 0;
}

//Funcion para concatenar dos strings.
char* concat(const char *s1, const char *s2)
{
    const size_t len1 = strlen(s1);
    const size_t len2 = strlen(s2);
    char *result = malloc(len1 + len2 + 1);
    memcpy(result, s1, len1);
    memcpy(result + len1, s2, len2 + 1);
    return result;
}

int main (int argc, char **argv){

	signal(SIGINT, intHandler);

	char *portname = "/dev/ttyACM0";
	char  payload[256],temp_unit,tilt_unit[4];
	int size,retSize;
	char size_c[4],rec;
	FILE *f;
	time_t now;
	time(&now);
	//Apertura del fichero de Log.
	char *str=concat(ctime(&now),".log");
	f=fopen(str,"w+");
	fprintf(f,"Archivo de log \n");
	fprintf(f,"Temperature [ºC]	Light [lux]		Roll [deg]		Pitch [deg]		Yaw [deg]\n");
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
  		fprintf(stderr,"error opening /dev/ttyACM0\n");
	}

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

	usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
                                     // receive 25:  approx 100 uS per char transmit
	fprintf(stdout,"'--------------------------------------------\nUSB - UART Bridge Serial  Port </dev/ttyACM0>\n'--------------------------------------------\n");


	printf("\nConfig data format");
	fflush(stdout);
		 	       
	for (int i = 0 ; i < 3 ; i++){
   
      	        usleep(500000);
                printf(".");
                fflush(stdout);
        }
	printf("\n\n");

	usleep(500000);
	temp_unit = temp_unit_select();

	usleep(500000);
 	tilt_unit_select(tilt_unit);

	usleep(500000);
	printf("\nLight is in lux by default\n");

	size = sprintf(payload,"TEMP->%c",temp_unit);

	retSize = 0;
	while (retSize != size){retSize = write(fd,payload,size);}//Send temp config

	usleep(100);
	payload[0] = '\0';
	
	size = sprintf(payload,"TILT->%s",tilt_unit);

	retSize = 0;
        while (retSize != size){retSize = write(fd,payload,size);}//Send tilt config
        usleep(100);
	payload[0] = '\0';

	printf("\n'--------------------------------------------------------------------------------------------------------\n");
	size = sprintf(payload,"Temperature [º%c]\tLight [lux]\t\tRoll [%s]\t\tPitch [%s]\t\tYaw [%s]\r\n\n",temp_unit,tilt_unit,tilt_unit,tilt_unit);

	printf("%s",payload);

	while(keepRunning){ //Loop el cual sale cuando se hace ctrl+c-

		size = 0;
		while(size != 1){
	
			size = read(fd,&rec,1);
		}
		printf("%c",rec);
		fprintf(f,"%c",rec); //grabamos al fichero de log.
		fflush(f); //Hacemos flush del buffer.	
	}

	printf("Terminando el programa...\n");
	fclose(f); //Cerramos fichero de log.
        write(fd,"C",1); //Mandamos un caracter para reset.
	close(fd);
	chmod(str, 0777); //Cambiar permisos de fichero abierto

	exit(0);
}
