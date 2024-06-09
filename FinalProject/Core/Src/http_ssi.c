/*
 *  CGI Form on HTTP Website
 *  Author: GateKeeper
 *  Copyright (C) 2024
*/


#include "http_ssi.h"
#include "string.h"
#include "stdio.h"
#include "IPs.h"

#include "lwip/tcp.h"
#include "lwip/apps/httpd.h"

#include "stm32f7xx_hal.h"

int indx = 0;

/************************ CGI HANDLER ***************************/
const char *CGIForm_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
//const char *CGILED_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

const tCGI FORM_CGI = {"/form.cgi", CGIForm_Handler};
//const tCGI LED_CGI = {"/led.cgi", CGILED_Handler};

int room;
int changed_ip_status = 0;

//tCGI CGI_TAB[1];

// This Handler function will get info from the HTML form Website
const char *CGIForm_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	if (iIndex == 0)
	{
		for (int i=0; i<iNumParams; i++)
		{
			if (strcmp(pcParam[i], "ipv4_address0") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				ipv4_address[0] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}

			else if (strcmp(pcParam[i], "ipv4_address1") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				ipv4_address[1] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}

			else if (strcmp(pcParam[i], "ipv4_address2") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				ipv4_address[2] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}

			else if (strcmp(pcParam[i], "ipv4_address3") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				ipv4_address[3] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}
			else if (strcmp(pcParam[i], "mask_address0") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				mask_address[0] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}

			else if (strcmp(pcParam[i], "mask_address1") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				mask_address[1] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}

			else if (strcmp(pcParam[i], "mask_address2") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				mask_address[2] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}
			else if (strcmp(pcParam[i], "mask_address3") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				mask_address[3] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}

			else if (strcmp(pcParam[i], "gateway_address0") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				gateway_address[0] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}

			else if (strcmp(pcParam[i], "gateway_address1") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				gateway_address[1] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}
			else if (strcmp(pcParam[i], "gateway_address2") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				gateway_address[2] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}

			else if (strcmp(pcParam[i], "gateway_address3") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				gateway_address[3] = atoi(pcValue[i]);
				changed_ip_status = 1;
			}

			else if (strcmp(pcParam[i], "room") == 0 && pcValue[i] != NULL && *pcValue[i] != '\0')
			{
				room = atoi(pcValue[i]);
				changed_ip_status = 1;
			}
		}
	}

	return "/index.html";
}

/*const char *CGILED_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	if (iIndex == 1)
	{
		for (int i=0; i<iNumParams; i++)
		{
			if (strcmp(pcParam[i], "fname") == 0)  // if the fname string is found
			{
				memset(name, '\0', 30);
				strcpy(name, pcValue[i]);
			}
			else if (strcmp(pcParam[i], "lname") == 0)  // if the fname string is found
			{
				strcat(name, " ");
				strcat(name, pcValue[i]);
			}
		}
	}
	return "/cgiled.html";
}*/

// This function will start the HTTP website
void http_server_init (void)
{
	httpd_init();

	//http_set_ssi_handler(ssi_handler, (char const**) TAGS, 3);

	//CGI_TAB[0] = FORM_CGI;
	//CGI_TAB[1] = LED_CGI;

	http_set_cgi_handlers (&FORM_CGI, 1);
	//http_set_cgi_handlers (CGI_TAB, 1);
}
