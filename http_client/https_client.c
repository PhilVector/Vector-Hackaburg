// Client side C/C++ program to demonstrate Socket programming
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <errno.h>
#include "./json-c/json.h"
//#define PORT 8502
#define PORT 1338

#define json_object_to_json_string(obj) json_object_to_json_string_ext(obj,sflags)

#define BUF_MAX 1024
#define MAX_CPU 128

int read_fields (FILE *fp, unsigned long long int *fields)
{
	int retval;
	char buffer[BUF_MAX];


	if (!fgets (buffer, BUF_MAX, fp)) {
		perror ("Error");
	}
	/* line starts with c and a string. This is to handle cpu, cpu[0-9]+ */
	retval = sscanf (buffer, "c%*s %Lu %Lu %Lu %Lu %Lu %Lu %Lu %Lu %Lu %Lu",
                            &fields[0],
                            &fields[1],
                            &fields[2],
                            &fields[3],
                            &fields[4],
                            &fields[5],
                            &fields[6],
                            &fields[7],
                            &fields[8],
                            &fields[9]);
	if (retval == 0) {
		return -1;
	}
	if (retval < 4) /* Atleast 4 fields is to be read */ {
		fprintf (stderr, "Error reading /proc/stat cpu field\n");
		return 0;
	}
	return 1;
}

int handle_utilisation(char **json) {
	FILE *fp;
	unsigned long long int fields[10], total_tick[MAX_CPU], total_tick_old[MAX_CPU], idle[MAX_CPU], idle_old[MAX_CPU], del_total_tick[MAX_CPU], del_idle[MAX_CPU];
	int i, cpus = 0, count;
	double percent_usage;

	fp = fopen ("/proc/stat", "r");
	if (fp == NULL) {
		perror ("Error");
	}

	/*Creating a json object*/
	json_object * jobj = json_object_new_object();
	json_object *jdtype = json_object_new_string("util");
	json_object_object_add(jobj,"rover_dtype", jdtype);
	json_object * jCoreData = json_object_new_object();

	while (read_fields (fp, fields) != -1) {
		for (i=0, total_tick[cpus] = 0; i<10; i++) {
			total_tick[cpus] += fields[i];
	    }
	    idle[cpus] = fields[3]; /* idle ticks index */
	    cpus++;
	}

	for(int i = 0; i < 1; i++) {
		sleep (1);
	    fseek (fp, 0, SEEK_SET);
	    fflush (fp);

	    for (count = 0; count < cpus; count++) {
	    	total_tick_old[count] = total_tick[count];
	    	idle_old[count] = idle[count];

	    	if (!read_fields (fp, fields)) {
	    		return 0;
	    	}

	    	for (i=0, total_tick[count] = 0; i<10; i++) {
	    		total_tick[count] += fields[i];
	    	}
	    	idle[count] = fields[3];

	    	del_total_tick[count] = total_tick[count] - total_tick_old[count];
	    	del_idle[count] = idle[count] - idle_old[count];

	    	percent_usage = ((del_total_tick[count] - del_idle[count]) / (double) del_total_tick[count]) * 100;
	    	if (count == 0) {
	    		printf ("Total CPU Usage: %3.2lf%%\n", percent_usage);
	    	} else {
	    		char* name = "CPU";
	    		char id[10];
	    		sprintf(id, "%d", count - 1);
	    		char *result = malloc(strlen(name) + strlen(id) + 1);//+1 for the null-terminator
				//in real code you would check for errors in malloc here
				strcpy(result, name);
				strcat(result, id);
	    		json_object *jdouble = json_object_new_double(percent_usage);
				json_object_object_add(jCoreData, result, jdouble);
//	    		printf ("\tCPU%d Usage: %3.2lf%%\n", count - 1, percent_usage);
				printf ("\t%s Usage: %3.2lf%%\n", result, percent_usage);
	    	}
	    }
	    printf ("\n");
	}

	json_object_object_add(jobj,"data", jCoreData);

	*json = json_object_to_json_string_ext(jobj, JSON_C_TO_STRING_SPACED | JSON_C_TO_STRING_PRETTY);

	fclose (fp);

	return 0;
}

int main(int argc, char const *argv[])
{
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char *utilisation;
    char buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, "172.31.31.2", &serv_addr.sin_addr)<=0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        printf("%i", errno);
        return -1;
    }

    while(1) {
    	handle_utilisation(&utilisation);
		printf("%s", utilisation);
		printf("\n");
		send(sock , utilisation , strlen(utilisation) , 0 );
		printf("Hello message sent\n");
    }
    valread = read( sock , buffer, 1024);
    printf("%s\n",buffer );
    return 0;
}
