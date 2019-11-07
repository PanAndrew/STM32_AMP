/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Client/Src/tcp_client.c
  * @author  MCD Application Team
  * @brief   tcp tcp_client_struct application using LwIP RAW API
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/memp.h"
#include <stdio.h>
#include <string.h>

#if LWIP_TCP
/* Private typedef -----------------------------------------------------------*/

#define DEST_IP_ADDR0   (uint8_t) 192
#define DEST_IP_ADDR1   (uint8_t) 168
#define DEST_IP_ADDR2   (uint8_t) 1
#define DEST_IP_ADDR3   (uint8_t) 120

#define DEST_PORT       (uint16_t) 1112

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

struct tcp_pcb *tcp_client_pcb = NULL;

/* Protocol states */
enum tcp_client_states
{
  ES_NOT_CONNECTED = 0,
  ES_CONNECTED,
  ES_RECEIVED,
  ES_CLOSING,
};

/* structure to be passed as argument to the tcp callbacks */
struct tcp_client_struct
{
  enum tcp_client_states state; /* connection status */
  struct tcp_pcb *pcb;          /* pointer on the current tcp_pcb */
  struct pbuf *p_tx;            /* pointer on pbuf to be transmitted */
};

/* Private function prototypes -----------------------------------------------*/
static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_client_connection_close(struct tcp_pcb *tpcb, struct tcp_client_struct * es);
static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_struct * es);
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Connects to the TCP server
  * @param  None
  * @retval None
  */
void tcp_client_connect(void)
{
  ip_addr_t DestIPaddr;

  /* create new tcp pcb */
  tcp_client_pcb = tcp_new();
  
  if (tcp_client_pcb != NULL)
  {
    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
    
    /* connect to destination address/port */
    tcp_connect(tcp_client_pcb,&DestIPaddr,DEST_PORT,tcp_client_connected);
  }
}

/**
  * @brief Function called when TCP connection established
  * @param tpcb: pointer on the connection contol block
  * @param err: when connection correctly established err should be ERR_OK 
  * @retval err_t: returned error 
  */
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  struct tcp_client_struct *es = NULL;
  
  if (err == ERR_OK)   
  {
    /* allocate structure es to maintain tcp connection informations */
    es = (struct tcp_client_struct *)mem_malloc(sizeof(struct tcp_client_struct));
  
    if (es != NULL)
    {
      es->state = ES_CONNECTED;
      es->pcb = tpcb;
      
      /* allocate pbuf */
      es->p_tx = pbuf_alloc(PBUF_TRANSPORT, 1, PBUF_POOL);
         
      if (es->p_tx)
      {
        
        /* pass newly allocated es structure as argument to tpcb */
        tcp_arg(tpcb, es);
  
        /* initialize LwIP tcp_recv callback function */ 
        tcp_recv(tpcb, tcp_client_recv);
  
        /* initialize LwIP tcp_sent callback function */
        tcp_sent(tpcb, tcp_client_sent);
  
        /* initialize LwIP tcp_poll callback function */
        tcp_poll(tpcb, tcp_client_poll, 1);
        
        return ERR_OK;
      }
    }
    else
    {
      /* close connection */
      tcp_client_connection_close(tpcb, es);
      
      /* return memory allocation error */
      return ERR_MEM;  
    }
  }
  else
  {
    /* close connection */
    tcp_client_connection_close(tpcb, es);
  }
  return err;
}

void sendDataWhileConnected(uint8_t * dataToSend, uint16_t dataLength)
{
	if(tcp_client_pcb != NULL)
	{
		struct tcp_client_struct *es;
		es = (struct tcp_client_struct *)tcp_client_pcb->callback_arg;

		if(tcp_client_pcb->state != ESTABLISHED)
		{
			if(tcp_client_pcb->state == CLOSED)
			{
				tcp_client_connect();
			}

			return;
		}

		es->p_tx = pbuf_alloc(PBUF_TRANSPORT, dataLength, PBUF_POOL);

		if(es->p_tx)
		{
			pbuf_take(es->p_tx, dataToSend, dataLength);
			tcp_client_send(tcp_client_pcb,es);
		}
		else
		{
			pbuf_free(es->p_tx);
		}
	}
	else
	{
		tcp_client_connect();
	}
}
    
/**
  * @brief tcp_receiv callback
  * @param arg: argument to be passed to receive callback 
  * @param tpcb: tcp connection control block 
  * @param err: receive error code 
  * @retval err_t: retuned error  
  */
static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{ 
  struct tcp_client_struct *es;
  err_t ret_err; 

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct tcp_client_struct *)arg;
  
  /* if we receive an empty tcp frame from server => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    if(es->p_tx == NULL)
    {
       /* we're done sending, close connection */
       tcp_client_connection_close(tpcb, es);
    }
    else
    {    
      /* send remaining data*/
      tcp_client_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  /* else : a non empty frame was received from server but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_CONNECTED)
  {
    /* increment message count */
    //message_count += 10;
         
    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);  
    
    pbuf_free(p);
    //tcp_client_connection_close(tpcb, es);
    ret_err = ERR_OK;
  }

  /* data received when connection already closed */
  else
  {
    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);
    
    /* free pbuf and do nothing */
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief function used to send data
  * @param  tpcb: tcp control block
  * @param  es: pointer on structure of type tcp_client_struct containing info on data
  *             to be sent
  * @retval None 
  */
static void tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_struct * es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
 
  while ((wr_err == ERR_OK) &&
         (es->p_tx != NULL) && 
         (es->p_tx->len <= tcp_sndbuf(tpcb)))
  {
    
    /* get pointer on pbuf from es structure */
    ptr = es->p_tx;

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
    
    if (wr_err == ERR_OK)
    { 
      /* continue with next pbuf in chain (if any) */
      es->p_tx = ptr->next;
      
      if(es->p_tx != NULL)
      {
        /* increment reference count for es->p */
        pbuf_ref(es->p_tx);
      }
      
      /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
      pbuf_free(ptr);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later, defer to poll */
     es->p_tx = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }

  if(wr_err == ERR_OK)
  {
	  tcp_output(tpcb);
  }
}

/**
  * @brief  This function implements the tcp_poll callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: tcp connection control block
  * @retval err_t: error code
  */
static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct tcp_client_struct *es;

  es = (struct tcp_client_struct*)arg;
  if (es != NULL)
  {
    if (es->p_tx != NULL)
    {
      /* there is a remaining pbuf (chain) , try to send data */
      tcp_client_send(tpcb, es);
    }
    else
    {
      /* no remaining pbuf (chain)  */
      if(tpcb->state != ESTABLISHED)
      {
        /* close tcp connection */
        tcp_client_connection_close(tpcb, es);
      }
    }
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
    tcp_abort(tpcb);
    ret_err = ERR_ABRT;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data) 
  * @param  arg: pointer on argument passed to callback
  * @param  tcp_pcb: tcp connection control block
  * @param  len: length of data sent 
  * @retval err_t: returned error code
  */
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct tcp_client_struct *es;

  LWIP_UNUSED_ARG(len);

  es = (struct tcp_client_struct *)arg;
  
  if(es->p_tx != NULL)
  {
    /* still got pbufs to send */
    tcp_client_send(tpcb, es);
  }

  return ERR_OK;
}

/**
  * @brief This function is used to close the tcp connection with server
  * @param tpcb: tcp connection control block
  * @param es: pointer on tcp_client_struct structure
  * @retval None
  */
static void tcp_client_connection_close(struct tcp_pcb *tpcb, struct tcp_client_struct * es )
{
  /* remove callbacks */
  tcp_recv(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_poll(tpcb, NULL,0);

  if (es != NULL)
  {
    mem_free(es);
  }

  /* close tcp connection */
  tcp_close(tpcb);  
}

#endif /* LWIP_TCP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/