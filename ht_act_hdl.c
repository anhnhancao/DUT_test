/*******************************************************************************
* Title                 :   Hardware Test
* Filename              :   ht_act_hdl.c
* Author                :   Trung Do
* Origin Date           :   Jul 10, 2020
* Notes                 :   None
*******************************************************************************/

/*************** MODULE REVISION LOG ******************************************
*  mm/dd/yyyy - AUTHOR  : Description.
*  07/12/2020 - Trung Do: Initial version.
*
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "hardware_test.h"
#include "ht_comm.h"
#include "ht_act_hdl.h"
#include "ads1294.h"
#include "bmi270.h"
#include "max30102.h"
#include "sdram.h"
#include "gd5f1gq4rf9igr.h"
#include "speaker.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define HT_ACT_HDL_TX_MSG_LEN        (50) /* Bytes */
#define HT_ACT_HDL_RX_MSG_LEN        (50) /* Bytes */
#define HT_ACT_HDL_TEST_RESULT_LEN   (40) /* Bytes */


/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/


/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
static uint8_t ht_act_hdl_tx_msg           [HT_ACT_HDL_TX_MSG_LEN];
static uint8_t ht_act_hdl_test_result      [HT_ACT_HDL_TEST_RESULT_LEN];


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static uint16_t ht_act_hdl_create_test_status_msg   (uint8_t component_id, uint8_t command_id, uint16_t len);
static void     ht_act_hdl_send_test_status         (uint8_t component_id, uint8_t command_id);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
//-----------------------------------------------------------------------------
// Analog Front-End
//-----------------------------------------------------------------------------
void ht_act_hdl_afe_1 (uint8_t* p_data, uint16_t data_length)
{
  afe_init();  
}

void ht_act_hdl_afe_2 (uint8_t* p_data, uint16_t data_length)
{
  afe_deinit();
} 

void ht_act_hdl_afe_3 (uint8_t* p_data, uint16_t data_length)
{
  uint8_t component_id = p_data[0];
  uint8_t command_id   = p_data[1];

  ht_act_hdl_test_result[0] = afe_get_chip_id();
  ht_act_hdl_send_test_status(component_id, command_id);    
}

void ht_act_hdl_afe_4 (uint8_t* p_data, uint16_t data_length)
{
  uint8_t component_id = p_data[0];
  uint8_t command_id   = p_data[1];
  uint8_t param        = p_data[2];
  uint8_t temp_buf[10];

  afe_activate_channel(param,true);
  afe_start();
  afe_read_data((afe_sample_set_t *)ht_act_hdl_test_result);
  
  switch(param)
  {
    case 3:
    for(uint8_t count=9; count<12; ++count)
    temp_buf[count-9]  = ht_act_hdl_test_result[count];
    break;

    case 4:
    for(uint8_t count=13; count<16; ++count)
    temp_buf[count-13] = ht_act_hdl_test_result[count];
    break;
  }
  memset(ht_act_hdl_test_result, 0, sizeof(ht_act_hdl_test_result));
  memcpy(ht_act_hdl_test_result,temp_buf,3);
  ht_act_hdl_send_test_status(component_id, command_id);
}



//-----------------------------------------------------------------------------
// IMU (gyro + accel)
//-----------------------------------------------------------------------------
void ht_act_hdl_imu_1 (uint8_t* p_data, uint16_t data_length)
{
  imu_init();
}

void ht_act_hdl_imu_2 (uint8_t* p_data, uint16_t data_length)
{
  imu_deinit();
}

void ht_act_hdl_imu_3 (uint8_t* p_data, uint16_t data_length)
{
  uint8_t component_id = p_data[0];
  uint8_t command_id   = p_data[1];

  ht_act_hdl_test_result[0] = imu_get_chip_id();
  ht_act_hdl_send_test_status(component_id, command_id);
}

void ht_act_hdl_imu_4 (uint8_t* p_data, uint16_t data_length)
{
  uint8_t component_id = p_data[0];
  uint8_t command_id   = p_data[1];

  /*
  imu_enable_accel(ACCEL_RANGE_2G,true);
  imu_set_odr(IMU_ODR_25);
  */
  imu_read_accel((accel_data_t *) ht_act_hdl_test_result);
  ht_act_hdl_send_test_status(component_id, command_id);
}

void ht_act_hdl_imu_5 (uint8_t* p_data, uint16_t data_length)
{
  uint8_t component_id = p_data[0];
  uint8_t command_id   = p_data[1];

  /*
  imu_enable_gyro(GYRO_RANGE_250,true);
  imu_set_odr(IMU_ODR_25);
  */
  imu_read_gyro((gyro_data_t *) ht_act_hdl_test_result);
  ht_act_hdl_send_test_status(component_id, command_id);
}


//-----------------------------------------------------------------------------
// MAX30102
//-----------------------------------------------------------------------------
void ht_act_hdl_hrm_1 (uint8_t* p_data, uint16_t data_length)
{
  max30102_init();
}

void ht_act_hdl_hrm_2 (uint8_t* p_data, uint16_t data_length)
{
  
}

void ht_act_hdl_hrm_3 (uint8_t* p_data, uint16_t data_length)
{
  uint8_t component_id = p_data[0];
  uint8_t command_id   = p_data[1];

  ht_act_hdl_test_result[0]=max30102_read_part_id();
  ht_act_hdl_send_test_status(component_id, command_id); 
}



//-----------------------------------------------------------------------------
// Button
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// LED
//-----------------------------------------------------------------------------
void ht_act_hdl_led_1 (uint8_t* p_data, uint16_t data_length)
{
  
}

void ht_act_hdl_led_2 (uint8_t* p_data, uint16_t data_length)
{
  
}

void ht_act_hdl_led_3 (uint8_t* p_data, uint16_t data_length)
{
  
}


//-----------------------------------------------------------------------------
// Microphone
//-----------------------------------------------------------------------------
void ht_act_hdl_mic_1 (uint8_t* p_data, uint16_t data_length)
{
  
}

void ht_act_hdl_mic_2 (uint8_t* p_data, uint16_t data_length)
{
  
}

void ht_act_hdl_mic_3 (uint8_t* p_data, uint16_t data_length)
{
  
}

void ht_act_hdl_mic_4 (uint8_t* p_data, uint16_t data_length)
{
  
}



//-----------------------------------------------------------------------------
// Speaker
//-----------------------------------------------------------------------------
void ht_act_hdl_spk_1 (uint8_t* p_data, uint16_t data_length)
{
  speaker_init();
}

void ht_act_hdl_spk_2 (uint8_t* p_data, uint16_t data_length)
{
  speaker_deinit();
}

void ht_act_hdl_spk_3 (uint8_t* p_data, uint16_t data_length)
{
  uint32_t duration_ms;
  
  duration_ms = (p_data[3]<<24) | (p_data[4]<<16) | (p_data[5]<<8) | p_data[6];
  speaker_play_audio(p_data[2],duration_ms);
}

void ht_act_hdl_spk_4 (uint8_t* p_data, uint16_t data_length)
{
  speaker_stop_audio();
}



//-----------------------------------------------------------------------------
// external RAM
//-----------------------------------------------------------------------------
void ht_act_hdl_exram_1 (uint8_t* p_data, uint16_t data_length)
{
	sdram_init();
}

void ht_act_hdl_exram_2 (uint8_t* p_data, uint16_t data_length)
{
	sdram_deinit();
}

void ht_act_hdl_exram_3 (uint8_t* p_data, uint16_t data_length)
{
	uint8_t* p_addr;
  uint32_t addr;

  addr    = (p_data[2]<<24) | (p_data[3]<<16) | (p_data[4]<<8) | p_data[5];
  p_addr  = (uint8_t *)addr;
  for(uint16_t count=6; count<6+data_length-8; ++count)
  {
    *p_addr = p_data[count];
    ++p_addr;
  }
}

void ht_act_hdl_exram_4 (uint8_t* p_data, uint16_t data_length)
{
	uint8_t component_id     = p_data[0];
  uint8_t command_id       = p_data[1];
  uint8_t* p_addr;          
  uint32_t addr;
	uint32_t byte_total;

  addr       = (p_data[2]<<24) | (p_data[3]<<16) | (p_data[4]<<8) | p_data[5];	
  byte_total = (p_data[6]<<24) | (p_data[7]<<16) | (p_data[8]<<8) | p_data[9];
	p_addr = (uint8_t *)addr;
  for(uint32_t count=0; count<byte_total; ++count)
  {
    ht_act_hdl_test_result[count] = *p_addr;
    ++p_addr; 
  }
  ht_act_hdl_send_test_status(component_id, command_id);
}


//-----------------------------------------------------------------------------
// external Flash
//-----------------------------------------------------------------------------
void ht_act_hdl_exflash_1 (uint8_t* p_data, uint16_t data_length)
{
	exflash_init();
}

void ht_act_hdl_exflash_2 (uint8_t* p_data, uint16_t data_length)
{
	exflash_deinit();
}

void ht_act_hdl_exflash_3 (uint8_t* p_data, uint16_t data_length)
{
	uint8_t component_id     = p_data[0];
  uint8_t command_id       = p_data[1];
	uint32_t exflash_chip_id = exflash_get_chip_id();

  for(uint8_t count=0; count<4; ++count)
	{
	  ht_act_hdl_test_result[count] = (uint8_t) (exflash_chip_id>>(8*(3-count)))&0x000000FF;
	}
  ht_act_hdl_send_test_status(component_id, command_id);
}

void ht_act_hdl_exflash_4 (uint8_t* p_data, uint16_t data_length)
{
  uint32_t addr;

  addr = (p_data[2]<<24) | (p_data[3]<<16) | (p_data[4]<<8) | p_data[5];
  exflash_block_erase(addr);
}

void ht_act_hdl_exflash_5 (uint8_t* p_data, uint16_t data_length)
{
	uint32_t addr;

  addr = (p_data[2]<<24) | (p_data[3]<<16) | (p_data[4]<<8) | p_data[5];
  exflash_page_write(addr,&p_data[6],data_length-8);
}

void ht_act_hdl_exflash_6 (uint8_t* p_data, uint16_t data_length)
{
  uint8_t component_id     = p_data[0];
  uint8_t command_id       = p_data[1];
  uint32_t byte_total;
	uint32_t addr;

  addr       = (p_data[2]<<24) | (p_data[3]<<16) | (p_data[4]<<8) | p_data[5];
  byte_total = (p_data[6]<<24) | (p_data[7]<<16) | (p_data[8]<<8) | p_data[9];
  exflash_page_read(addr,ht_act_hdl_test_result,byte_total);
  ht_act_hdl_send_test_status(component_id, command_id);
}


//-----------------------------------------------------------------------------
// GPIO
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// MAX30102
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// PMIC
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// BLE
//-----------------------------------------------------------------------------

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
static uint16_t ht_act_hdl_create_test_status_msg (uint8_t component_id, uint8_t command_id, uint16_t len)
{
  uint16_t size = 0;
  uint8_t  crc  = 0;
  uint16_t msg_size = 0;

  ht_act_hdl_tx_msg[0] = HT_COMM_MSG_ID_REQ_STATUS;
  ht_act_hdl_tx_msg[3] = component_id;
  ht_act_hdl_tx_msg[4] = command_id;

  /* Append test results data bytes to the status message */
  uint16_t j = 0;

  for (uint16_t i=5; i < len+5; ++i)
  {
    ht_act_hdl_tx_msg[i] = ht_act_hdl_test_result[j++];
  }

  // Payload length added to the message - includes the componentId, commandId,
  // message data, sequence num and checksum bytes in the length
  len += 4;
  ht_act_hdl_tx_msg[1] = len & 0xFF;
  ht_act_hdl_tx_msg[2] = (len >> 8) & 0xFF;

  // Add the messageId, two len bytes and the test results data bytes
  size = len + 3;

  ht_act_hdl_tx_msg[size-2] = ht_comm_increase_seq_num();

  // Placeholder for crc
  ht_act_hdl_tx_msg[size-1] = 0xFF;

  // Final total message size
  msg_size = size + 2;
  crc = ht_comm_cal_crc8(ht_act_hdl_tx_msg, msg_size);
  ht_act_hdl_tx_msg[size-1] = crc;

  ht_act_hdl_tx_msg[size]   = HT_COMM_SYMBOL_SYNC1;
  ht_act_hdl_tx_msg[size+1] = HT_COMM_SYMBOL_SYNC2;

  return msg_size;
}

static void ht_act_hdl_send_test_status (uint8_t component_id, uint8_t command_id)
{
  uint16_t len = strlen((const char *) ht_act_hdl_test_result);
  uint16_t msgSize  = 0;

  msgSize = ht_act_hdl_create_test_status_msg(component_id, command_id, len);

  ht_send_string((uint8_t *) ht_act_hdl_tx_msg, msgSize);

  ht_comm_wait_for_ate_response(ht_act_hdl_tx_msg, msgSize);

  memset(ht_act_hdl_test_result, 0, sizeof(ht_act_hdl_test_result));
}

/*************** END OF FILES *************************************************/
