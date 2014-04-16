/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "app_uart.h"
#include "app_fifo.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util.h"
#include "app_gpiote.h"
#include "hrm_rx.h"
#define FIFO_LENGTH(F)             (F.write_pos - F.read_pos)               /**< Macro to calculate length of a FIFO. */
#define UART_INSTANCE_GPIOTE_BASE  0x00FF                                   /**< Define the base for UART instance ID when flow control is used. The userid from GPIOTE will be used with padded 0xFF at LSB for easy converting the instance id to GPIOTE id. */
#define UART_INSTANCE_ID_INVALID   0x0000                                   /**< Value 0x0000 is used to indicate an invalid instance id. When 0 is provided as instance id upon initialization, the module will provide a valid id to the caller. */


extern uint8_t  ANT_ResetSystem(void);
extern uint8_t ANT_Cmd55(uint8_t chan);
extern uint8_t ANT_OpenRxScanMode(uint8_t chan);
extern uint8_t ANT_Init(uint8_t devno, uint32_t baud);
extern uint8_t ANT_RequestMessage(uint8_t chan, uint8_t mesg);
extern uint8_t ANT_SetNetworkKeya(uint8_t net, uint8_t*key);
extern uint8_t ANT_AssignChannel(uint8_t chan, uint8_t chtype, uint8_t net);
extern uint8_t ANT_UnAssignChannel(uint8_t chan);
extern uint8_t ANT_SetChannelId(uint8_t chan, uint32_t dev, uint8_t devtype, uint8_t manid);
extern uint8_t ANT_SetChannelRFFreq(uint8_t chan, uint8_t freq);
extern uint8_t ANT_SetChannelPeriod(uint8_t chan, uint32_t period);
extern uint8_t ANT_SetChannelSearchTimeout(uint8_t chan,uint8_t timeout);
extern uint8_t ANT_SetSearchWaveform(uint8_t chan, uint32_t waveform);
extern uint8_t ANT_SendAcknowledgedDataA(uint8_t chan, uint8_t *data);
extern uint8_t ANT_SendAcknowledgedData(uint8_t chan, uint8_t *data);
extern uint32_t ANT_SendBurstTransferA(uint8_t chan, uint8_t *data, uint32_t numpkts);
extern uint32_t ANT_SendBurstTransfer(uint8_t chan, uint8_t *data, uint32_t numpkts);
extern uint8_t ANT_OpenChannel(uint8_t chan);
extern uint8_t ANT_CloseChannel(uint8_t chan); 

/** @brief States for the app_uart state machine. */
typedef enum
{
    UART_OFF,                                                               /**< app_uart state OFF, indicating CTS is low. */
    UART_READY,                                                             /**< app_uart state ON, indicating CTS is high. */
    UART_ON,                                                                /**< app_uart state TX, indicating UART is ongoing transmitting data. */
    UART_WAIT_CLOSE,                                                        /**< app_uart state WAIT CLOSE, indicating that CTS is low, but a byte is currently being transmitted on the line. */
} app_uart_states_t;

/** @brief State transition events for the app_uart state machine. */
typedef enum
{
    ON_CTS_HIGH,                                                            /**< Event: CTS gone high. */
    ON_CTS_LOW,                                                             /**< Event: CTS gone low. */
    ON_UART_PUT,                                                            /**< Event: Application wants to transmit data. */
    ON_TX_READY,                                                            /**< Event: Data has been transmitted on the uart and line is available. */
    ON_UART_CLOSE,                                                          /**< Event: The UART module are being stopped. */
} app_uart_state_events_t;

static app_fifo_t                  m_rx_fifo;                               /**< RX FIFO buffer for storing data received on the UART until the application fetches them using app_uart_get(). */
static app_fifo_t                  m_tx_fifo;                               /**< TX FIFO buffer for storing data to be transmitted on the UART when TXD is ready. Data is put to the buffer on using app_uart_put(). */

static uint8_t                     m_instance_counter = 1;                  /**< Instance counter for each caller using the UART module. The GPIOTE user id is mapped directly for callers using HW Flow Control. */
static app_gpiote_user_id_t        m_gpiote_uid;                            /**< GPIOTE id for currently active caller to the UART module. */
static uint32_t                    m_pin_cts_mask;                          /**< CTS pin mask for UART module. */
static app_uart_event_handler_t    m_event_handler;                         /**< Event handler function. */
static volatile app_uart_states_t  m_current_state = UART_OFF;              /**< State of the state machine. */

/**@brief Function for disabling the UART when entering the UART_OFF state.
 */
static void action_uart_deactivate(void)
{
    m_current_state         = UART_OFF;
    NRF_UART0->TASKS_STOPTX = 1;
    NRF_UART0->TASKS_STOPRX = 1;
    NRF_UART0->ENABLE       = (UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos);
}


void action_tx_stop()
{
    app_uart_evt_t app_uart_event;
    // No more bytes in FIFO, terminate transmission.
    NRF_UART0->TASKS_STOPTX = 1;
    m_current_state         = UART_READY;
    // Last byte from FIFO transmitted, notify the application.
    // Notify that new data is available if this was first byte put in the buffer.
    app_uart_event.evt_type = APP_UART_TX_EMPTY;
    m_event_handler(&app_uart_event);
}


/**@brief Function for sending the next byte in the TX buffer. Called when (re-)entering the UART_ON state.
 *       If no more data is available in the TX buffer, the state machine will enter UART_READY state.
 */
static void action_tx_send()
{
    uint8_t tx_byte;

    if (m_current_state != UART_ON)
    {
        // Start the UART.
        NRF_UART0->TASKS_STARTTX = 1;
    }

    if (app_fifo_get(&m_tx_fifo, &tx_byte) != NRF_SUCCESS)
    {
        action_tx_stop();
        return;
    }

    NRF_UART0->INTENCLR = (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos);
    NRF_UART0->TXD      = tx_byte;
    m_current_state     = UART_ON;
    NRF_UART0->INTENSET = (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos);
}


static void action_tx_ready()
{
    // Get next byte from FIFO.
    if (FIFO_LENGTH(m_tx_fifo) != 0)
    {
        action_tx_send();
    }
    else
    {
        action_tx_stop();
    }
}


/**@brief Function for the handling of the ON_CTS_HIGH event.
 */
static void on_cts_high(void)
{
    switch (m_current_state)
    {
        case UART_READY:
            action_uart_deactivate();
            break;

        case UART_ON:
            m_current_state = UART_WAIT_CLOSE;
            break;

        default:
            // Nothing to do.
            break;
    }
}


/**@brief Function for the handling of the ON_CTS_LOW event.
 */
static void on_cts_low(void)
{
    switch (m_current_state)
    {
        case UART_OFF:
            NRF_UART0->ENABLE        = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
            NRF_UART0->TASKS_STARTRX = 1;

            if (FIFO_LENGTH(m_tx_fifo) != 0)
            {
                action_tx_send();
            }
            else
            {
                m_current_state = UART_READY;
            }
            break;

        case UART_WAIT_CLOSE:
            m_current_state = UART_ON;
            break;

        default:
            // Nothing to do.
            break;
    }
}


/**@brief Function for the handling of the ON_TX_READY event.
 */
static void on_tx_ready(void)
{
    switch (m_current_state)
    {
        case UART_WAIT_CLOSE:
            action_uart_deactivate();
            break;

        case UART_ON:
        case UART_READY:
            action_tx_ready();
            break;

        default:
            // Nothing to do.
            break;
    }
}


/**@brief Function for the handling of the ON_UART_PUT event when application has put data in the TX buffer.
 */
static void on_uart_put(void)
{
    if (m_current_state == UART_READY)
    {
        action_tx_send();
    }
}


/**@brief Function for the handling of the ON_UART_CLOSE event when application is closing the UART module.
 */
static void on_uart_close(void)
{
    action_uart_deactivate();
}


/**@brief Function for the state machine main event handler.
 *
 * @param[in]  event    Event that has occurred.
 */
static void on_uart_event(app_uart_state_events_t event)
{
    switch (event)
    {
        case ON_CTS_HIGH:
            on_cts_high();
            break;

        case ON_CTS_LOW:
            on_cts_low();
            break;

        case ON_TX_READY:
            on_tx_ready();
            break;

        case ON_UART_PUT:
            on_uart_put();
            break;

        case ON_UART_CLOSE:
            on_uart_close();
            break;

        default:
            // All valid events are handled above.
            break;
    }
}


/**@brief Function for the GPIOTE event handler.
 *
 * @param[in] event_pins_low_to_high   Mask telling which pin(s) generated an event from low->high.
 * @param[in] event_pins_high_to_low   Mask telling which pin(s) generated an event from high->low.
 */
static void gpiote_uart_event_handler(uint32_t event_pins_low_to_high,
                                      uint32_t event_pins_high_to_low)
{
    if ((event_pins_high_to_low & event_pins_low_to_high & m_pin_cts_mask) != 0)
    {
        // We have an indication from GPIOTE that the CTS pin has toggled high->low and low->high.
        // If this occurs, we must read the active pins in the GPIOTE module ourself.
        uint32_t active_pins;
        uint32_t err_code;

        err_code = app_gpiote_pins_state_get(m_gpiote_uid, &active_pins);
        if (err_code != NRF_SUCCESS)
        {
            // Pin reading was not possible, even though an event from GPIOTE was received that the
            // CTS pin toggled. If pin double toggled but status cannot be fetched we silently
            // return and keep the current UART status as-is.
            return;
        }
        event_pins_low_to_high &= active_pins;
        event_pins_high_to_low &= ~active_pins;
    }

    if ((event_pins_high_to_low & m_pin_cts_mask) != 0)
    {
        on_uart_event(ON_CTS_LOW);
    }
    else if ((event_pins_low_to_high & m_pin_cts_mask) != 0)
    {
        on_uart_event(ON_CTS_HIGH);
    }
    else
    {
        // Do nothing, as the CTS pin didn't toggle.
    }
}

//==== robin
void firmware_line(uint8_t  * p_event_message_buffer,uint8_t  mesg,  uint8_t len ){
	
		//const uint32_t message_id = p_event_message_buffer[BUFFER_INDEX_MESG_ID]; 
	  uint8_t k =0;
	  uint8_t addedlen = 4+len;
   
		uint8_t  buf[48];
		//uint32_t nw;
		int i;
		uint8_t chk = 0xa4;
	   

		buf[0] = 0xa4;
		buf[1] = addedlen; chk ^= addedlen;
		buf[2] = mesg; chk ^= mesg;
		for (i = 0; i < len; i++) {
			buf[3+i] = p_event_message_buffer[i];
			chk ^= p_event_message_buffer[i];
		}
		buf[3+i] = chk;
		
		
		int j=0;
		
		for(j=0;j<4+len;j++){
			  
			 //printf("\n %02x \n\r",p_event_message_buffer[j]);
			 app_uart_put(buf[j]);	
			  for(k=0;k<100;k++);	
			  
		}

}



void new_uartreader_send_response(uint8_t rsp ){
	  uint8_t k =0;
	  uint16_t deviceNumber =0 ; uint8_t deviceType = 0;uint8_t transType =rsp;
	  uint8_t addedlen = 4+4;
		uint8_t  buf[30];
		uint8_t chk = 0xA4;
	   

		buf[0] = 0xA4;
		buf[1] = addedlen; chk ^= addedlen;
		buf[2] = 0x50; chk ^= 0x50;
		
		buf[3+0] = deviceNumber;
		chk ^= deviceNumber;
	
	  buf[3+1] = (deviceNumber>>8);
		chk ^= (deviceNumber>>8);
	  
	  buf[3+2] = deviceType;
		chk ^= deviceType;
	  
	  buf[3+3] = transType;
		chk ^= transType;

		buf[3+4] = chk;
		int j=0;
		for(j=0;j<8;j++){ 
			 
			 app_uart_put(buf[j]);	
			  //nrf_delay_us(1000);
         for(k=0;k<100;k++);			
		}
}

void uartreader_send_response(uartreader_response_t rsp)
{
		NRF_UART0->TXD = (uint8_t) rsp;

		NRF_UART0->TASKS_STARTTX = 1;
}

extern uartreader_response_t write_line(uartreader_evt_t * evt);
extern void erase_app(void);

void uart_handler(uartreader_evt_t * evt)
{
    //uartreader_response_t result;
	   
	    
    switch (evt->cmd)
    {
        case REST_SYSTEM:
					   ANT_ResetSystem();
             
            uartreader_send_response((uartreader_response_t)0x4a);
				    //SD_NVIC_SYSTEMRESET 
            break;

        case SET_NETWORK_KEY:
					  //ret = ANT_SetNetworkKeya(uint8_t net, uint8_t*key);
					  //printf("\n SET_NETWORK_KEY ");
            //result = 0;//write_line(evt);
            
				    uartreader_send_response((uartreader_response_t)0x46);
            break;

        case ASSIGN_CHANNEL:
					
						ANT_AssignChannel(evt->data[3], evt->data[4], evt->data[5]);
					  //printf("\n ASSIGN_CHANNEL ");
            uartreader_send_response((uartreader_response_t)0x42);           
            break;

        case MESG_CHANNEL_ID_ID_ENUM:
						//buf[4] = dev%256;
						//buf[5] = dev/256;
						//input=low | (high<<8).
						 ANT_SetChannelId(evt->data[3], (evt->data[4]|(evt->data[5]<<8)), evt->data[6],evt->data[7]);
					  //printf("\n SET_CHANNEL_ID ");
				    
            uartreader_send_response((uartreader_response_t)0x51);
            break;
				case SET_CHANNEL_PERIOD:					
					    ANT_SetChannelPeriod(evt->data[3], (evt->data[4]|(evt->data[5]<<8)));
					  //printf("\n SET_MESSAGE_PERIOD ");
            uartreader_send_response((uartreader_response_t)0x43);            
            break;
				
				case SET_RF_FREQ:					
					  ANT_SetChannelRFFreq(evt->data[3],evt->data[4]);
					  //printf("\n SET_RF_FREQ ");
            uartreader_send_response((uartreader_response_t)0x45);
            
            break;
				case OPEN_CHANNEL:					
						  ANT_OpenChannel(evt->data[3]);
					  //printf("\n OPEN_CHANNEL ");
            uartreader_send_response((uartreader_response_t)0x4b);

            
            break;
			 case NOP:
            uartreader_send_response((uartreader_response_t)0xEE);
            
            break;
			 case MESG_SENSOR_TYPE:
				      ANT_SensorType(evt->data[3]);
			      uartreader_send_response((uartreader_response_t)0x82);
						break;
			 case MESG_CHANNEL_CLOSE_ID:
			      //ret =ANT_RequestMessage(evt->data[3], MESG_CHANNEL_CLOSE_ID);
			        ANT_CloseChannel(evt->data[3]);
				    uartreader_send_response((uartreader_response_t)0x4c);
			       
			      
				    break;
			 case MESG_REQUEST_ID_ENUM:
			      ANT_RequestMessage(evt->data[3], evt->data[4]); 
				    //uartreader_send_response((uartreader_response_t)0x4d);
				    break;
			 case MESG_FIRMWARE_HEX_LINE:	
             //write_line(evt);				 
			       //firmware_line(&evt->data[3],0x91,evt->data[1]);			      
			      break;			 
			 case MESG_ERASE_APP:
            //erase_app();
            //firmware_line(&evt->data[3],0x91,evt->data[1]);
            break;
        case MESG_RESET_AND_RUN:
            //firmware_line(&evt->data[3],0x91,evt->data[1]);
            //NVIC_SystemReset();
            break;
			 
			 default :
				    //printf("\nJMD ");
				    uartreader_send_response((uartreader_response_t)0xFF);
			     break;
    }
}

static uint8_t m_buffer_index = 0;
static uartreader_handler_t m_evt_handler =uart_handler;
static uartreader_evt_t m_evt;
static uint32_t this_msg_len =0;


void command_handler_from_stm32xx(uint8_t ch){
		
			if(m_buffer_index==0){ //start of the message 
					if(ch==0xa4){ //first char  should be a4.
						 
							 m_evt.data[m_buffer_index] = ch;
               m_buffer_index++;
							 ch=0;							 
						}
			}else{
              m_evt.data[m_buffer_index] = ch;
							if(m_buffer_index == 1 ){  //this is length byte
								 this_msg_len = m_evt.data[m_buffer_index];
							}
							if(m_buffer_index == 2){ //this is command byte
								 m_evt.cmd = (uartreader_cmd_t)m_evt.data[m_buffer_index];
							}	
							if(m_evt.data[m_buffer_index] == '\n'  // this is need to check , may not required 
								          || m_buffer_index >= UARTREADER_MAX_LEN-1  //this is for a max value case 
							            || m_buffer_index >= this_msg_len+3 ){   //this is what is working now.
														
														 
                  m_evt.len = m_buffer_index-1;
                  m_evt_handler(&m_evt);  //jump to main command handler 
                  m_buffer_index = 0;  //clear index
								  this_msg_len =0;    //clear length
							}else{
                m_buffer_index++;
							}
			}
      //jmd --<<
}
//==== robin end 
static uint16_t                    m_rx_byte = 0xFFFF;
/**@brief Function for the UART Interrupt handler.
 *
 * @details UART interrupt handler to process TX Ready when TXD is available, RX Ready when a byte
 *          is received, or in case of error when receiving a byte.
 */
void UART0_IRQHandler(void)
{
    // Handle reception
    if (NRF_UART0->EVENTS_RXDRDY != 0)
    {
        uint32_t err_code;
        
        // Clear UART RX event flag
        NRF_UART0->EVENTS_RXDRDY = 0;
			
			  m_rx_byte  = (uint8_t)NRF_UART0->RXD;
		    command_handler_from_stm32xx(m_rx_byte);
        
        // Write received byte to FIFO
        //err_code = app_fifo_put(&m_rx_fifo, (uint8_t)NRF_UART0->RXD);
			  err_code = app_fifo_put(&m_rx_fifo, m_rx_byte);
        if (err_code != NRF_SUCCESS)
        {
            app_uart_evt_t app_uart_event;
            app_uart_event.evt_type          = APP_UART_FIFO_ERROR;
            app_uart_event.data.error_code   = err_code;
            m_event_handler(&app_uart_event);
        }
        // Notify that new data is available if this was first byte put in the buffer.
        else if (FIFO_LENGTH(m_rx_fifo) == 1)
        {
            app_uart_evt_t app_uart_event;
            app_uart_event.evt_type = APP_UART_DATA_READY;
            m_event_handler(&app_uart_event);
        }
        else
        {
            // Do nothing, only send event if first byte was added or overflow in FIFO occurred.
        }
    }
    
    // Handle transmission.
    if (NRF_UART0->EVENTS_TXDRDY != 0)
    {
        // Clear UART TX event flag.
        NRF_UART0->EVENTS_TXDRDY = 0;
        on_uart_event(ON_TX_READY);
    }
    
    // Handle errors.
    if (NRF_UART0->EVENTS_ERROR != 0)
    {
        uint32_t       error_source;
        app_uart_evt_t app_uart_event;

        // Clear UART ERROR event flag.
        NRF_UART0->EVENTS_ERROR = 0;
        
        // Clear error source.
        error_source        = NRF_UART0->ERRORSRC;
        NRF_UART0->ERRORSRC = error_source;

        app_uart_event.evt_type                 = APP_UART_COMMUNICATION_ERROR;
        app_uart_event.data.error_communication = error_source;

        m_event_handler(&app_uart_event);
    }
}


/**@brief Function for initialization of UART when flow control is disabled.
 */
static void uart_no_flow_control_init(void)
{
    NRF_UART0->ENABLE        = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;

    NRF_UART0->CONFIG       &= ~(UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);

    NRF_UART0->PSELRTS       = UART_PIN_DISCONNECTED;
    NRF_UART0->PSELCTS       = UART_PIN_DISCONNECTED;

    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;
}


/**@brief Function for initialization of UART when standard flow control is enabled.
 */
static void uart_standard_flow_control_init(const app_uart_comm_params_t * p_comm_params)
{
    NRF_UART0->ENABLE        = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;

    NRF_UART0->CONFIG       |= (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);

    NRF_UART0->PSELCTS       = p_comm_params->cts_pin_no;
    NRF_UART0->PSELRTS       = p_comm_params->rts_pin_no;

    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;
}


uint32_t app_uart_init(const app_uart_comm_params_t * p_comm_params,
                             app_uart_buffers_t *     p_buffers,
                             app_uart_event_handler_t event_handler,
                             app_irq_priority_t       irq_priority,
                             uint16_t *               p_app_uart_uid)
{
    uint32_t err_code;
    uint32_t gpiote_high_pins;
    uint32_t gpiote_pin_low_high_mask = 0;
    uint32_t gpiote_pin_high_low_mask = 0;

    m_current_state = UART_OFF;
    m_event_handler = event_handler;

    if (p_buffers == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // Configure buffer RX buffer.
    err_code = app_fifo_init(&m_rx_fifo, p_buffers->rx_buf, p_buffers->rx_buf_size);
    if (err_code != NRF_SUCCESS)
    {
        // Propagate error code.
        return err_code;
    }

    // Configure buffer TX buffer.
    err_code = app_fifo_init(&m_tx_fifo, p_buffers->tx_buf, p_buffers->tx_buf_size);
    if (err_code != NRF_SUCCESS)
    {
        // Propagate error code.
        return err_code;
    }

    // Configure RX and TX pins.
    nrf_gpio_cfg_output(p_comm_params->tx_pin_no);
    nrf_gpio_cfg_input(p_comm_params->rx_pin_no, NRF_GPIO_PIN_NOPULL);

    NRF_UART0->PSELTXD = p_comm_params->tx_pin_no;
    NRF_UART0->PSELRXD = p_comm_params->rx_pin_no;

    // Configure baud rate and parity.
    NRF_UART0->BAUDRATE = (p_comm_params->baud_rate << UART_BAUDRATE_BAUDRATE_Pos);
    if (p_comm_params->use_parity)
    {
        NRF_UART0->CONFIG = (UART_CONFIG_PARITY_Included << UART_CONFIG_PARITY_Pos);
    }
    else
    {
        NRF_UART0->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos);
    }

    if (p_comm_params->flow_control == APP_UART_FLOW_CONTROL_LOW_POWER)
    {
        // Configure hardware flow control.
        nrf_gpio_cfg_output(p_comm_params->rts_pin_no);
        NRF_GPIO->OUT = 1 << p_comm_params->rts_pin_no;

        NRF_UART0->PSELCTS  = UART_PIN_DISCONNECTED;
        NRF_UART0->PSELRTS  = p_comm_params->rts_pin_no;
        NRF_UART0->CONFIG  |= (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);

        // Setup the gpiote to handle pin events on cts-pin.
        // For the UART we want to detect both low->high and high->low transitions in order to
        // know when to activate/de-activate the TX/RX in the UART.
        // Configure pin.
        m_pin_cts_mask = (1 << p_comm_params->cts_pin_no);
        nrf_gpio_cfg_sense_input(p_comm_params->cts_pin_no, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);

        gpiote_pin_low_high_mask = (1 << p_comm_params->cts_pin_no);
        gpiote_pin_high_low_mask = (1 << p_comm_params->cts_pin_no);

        if (*p_app_uart_uid == UART_INSTANCE_ID_INVALID)
        {
            err_code   = app_gpiote_user_register(&m_gpiote_uid,
                                                  gpiote_pin_low_high_mask,
                                                  gpiote_pin_high_low_mask,
                                                  gpiote_uart_event_handler);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
            *p_app_uart_uid = (m_gpiote_uid << 8) | UART_INSTANCE_GPIOTE_BASE;
        }
        else if (*p_app_uart_uid < UART_INSTANCE_GPIOTE_BASE)
        {
            return NRF_ERROR_INVALID_PARAM;
        }
        else
        {
            m_gpiote_uid = ((*p_app_uart_uid) >> 8) & UART_INSTANCE_GPIOTE_BASE;
        }

        err_code = app_gpiote_pins_state_get(m_gpiote_uid, &gpiote_high_pins);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        err_code = app_gpiote_user_enable(m_gpiote_uid);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // UART CTS pin is active when low.
        if ((gpiote_high_pins & (1 << p_comm_params->cts_pin_no)) == 0)
        {
            on_uart_event(ON_CTS_LOW);
        }
        else
        {
            on_uart_event(ON_CTS_HIGH);
        }
    }
    else if (p_comm_params->flow_control == APP_UART_FLOW_CONTROL_ENABLED)
    {
        if (*p_app_uart_uid == UART_INSTANCE_ID_INVALID)
        {
            *p_app_uart_uid = m_instance_counter++;
        }

        uart_standard_flow_control_init(p_comm_params);
        m_current_state = UART_READY;
    }
    else
    {
        if (*p_app_uart_uid == UART_INSTANCE_ID_INVALID)
        {
            *p_app_uart_uid = m_instance_counter++;
        }

        uart_no_flow_control_init();
        m_current_state = UART_READY;
    }

    // Enable UART interrupt
    NRF_UART0->INTENCLR = 0xffffffffUL;
    NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
                          (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
                          (UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos);

    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_SetPriority(UART0_IRQn, irq_priority);
    NVIC_EnableIRQ(UART0_IRQn);

    return NRF_SUCCESS;
}


uint32_t app_uart_get(uint8_t * p_byte)
{
    return app_fifo_get(&m_rx_fifo, p_byte);
}


uint32_t app_uart_put(uint8_t byte)
{
    uint32_t err_code;

    err_code = app_fifo_put(&m_tx_fifo, byte);

    on_uart_event(ON_UART_PUT);

    return err_code;
}


uint32_t app_uart_flush(void)
{
    uint32_t err_code;

    err_code = app_fifo_flush(&m_rx_fifo);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = app_fifo_flush(&m_tx_fifo);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t app_uart_get_connection_state(app_uart_connection_state_t * p_conn_state)
{
    *p_conn_state = ((m_current_state == UART_OFF) ? APP_UART_DISCONNECTED : APP_UART_CONNECTED);

    return NRF_SUCCESS;
}


uint32_t app_uart_close(uint16_t app_uart_uid)
{
    uint16_t gpiote_uid;

    if (app_uart_uid < UART_INSTANCE_GPIOTE_BASE)
    {
        on_uart_event(ON_UART_CLOSE);
        return NRF_SUCCESS;
    }

    gpiote_uid = (app_uart_uid >> 8) & UART_INSTANCE_GPIOTE_BASE;

    if (gpiote_uid != m_gpiote_uid)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    on_uart_event(ON_UART_CLOSE);

    return app_gpiote_user_disable(m_gpiote_uid);
}

