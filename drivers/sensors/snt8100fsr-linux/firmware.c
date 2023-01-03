/*****************************************************************************
* File: firmware.c
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*
*
*****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include "config.h"
#include "irq.h"
#include "serial_bus.h"
#include "workqueue.h"
#include "event.h"
#include "file.h"
#include "memory.h"
#include "device.h"
#include "firmware.h"
#include "utils.h"
#include "debug.h"
#include "file_control.h"
#include "locking.h"
#include <linux/firmware.h>
/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/
#define MAX_PAYLOAD_NUM     1000 // 1 = Hdr only
#define MAX_PAYLOAD_BYTES   (65 * 1024)
/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
struct upload_work {
	struct delayed_work my_work;
	struct snt8100fsr *snt8100fsr;

	char *filename;
	struct file *f;
	int payload_num;
	int file_offset;

	int num_write;
	uint32_t size;
	uint32_t pay_write;

	bool final_irq;
	bool waiting_for_irq;

	uint32_t firmware_upload_time;
	uint32_t payload_upload_time;
	uint32_t total_bytes_written;

	uint8_t *data_in;
	uint8_t *data_out;
};
/* ASUS BSP Clay: load different fw version according to HWID +++ */
//extern enum DEVICE_HWID g_ASUS_hwID;
/* ASUS BSP Clay: load different fw version according to HWID --- */

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
void upload_wq_func(struct work_struct *work);
static void upload_firmware_internal(void);
static irqreturn_t irq_handler_top(int irq, void *dev);

static void download_wq_func(struct work_struct *work);
static void download_firmware_internal(void);

//static int open_firmware_file(struct upload_work *w);
//static error_t firmware_waiting_irq(struct upload_work *w);
//static error_t firmware_transfer_payloads(struct upload_work *w);
extern void grip_input_event_report(int g_id, int len, int trk_id, int bar_id, int force, int fr_nr, int center);
/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static uint32_t bcr_log_level = 0;
static uint32_t bcr_addr = 0;
static uint32_t bcr_irpt = 0;

/*==========================================================================*/
/* BOOT FIRMWARE CONFIG                                                     */
/*==========================================================================*/
void set_bcr_addr(uint32_t addr) {
	bcr_addr = (addr & BCR_I2CADDR_MASK) << BCR_I2CADDR_POS;
}

void set_bcr_log_lvl(uint32_t l) {
	bcr_log_level = (l & BCR_LOGLVL_MASK) << BCR_LOGLVL_POS;
}

void set_bcr_irpt_lvl(uint32_t l) {
	bcr_irpt |= (l & BCR_IRPTLVL_MASK) << BCR_IRPTLVL_POS;
}

void set_bcr_irpt_pol(uint32_t p) {
	bcr_irpt |= (p & BCR_IRPTPOL_MASK) << BCR_IRPTPOL_POS;
}

void set_bcr_irpt_dur(uint32_t d) {
	bcr_irpt |= (d & BCR_IRPTDUR_MASK) << BCR_IRPTDUR_POS;
}

/*
 * set_bcr_word()
 *
 * [31:24] - I2C address. valid 0,0x2c,0x2d,0x5c,0x5d (0 means "all 4")
 * [23:11] - Reserved.
 * [10:8] - Logging Level
 * [7:2] - Edge Duration in 78MHz tics. "0" means 10 tics (default).
 * [1] - Interrupt Level. 0 - Level, 1 - Edge
 * [0] - Interrupt Polarity. 0 - Active High, 1 - Active Low
 */
uint32_t set_bcr_word(void) {
	uint32_t bcr = bcr_addr | bcr_log_level | bcr_irpt;
	PRINT_DEBUG("Boot Cfg Record = 0x%08x", bcr);
	return bcr;
}

struct delayed_work own_work;
struct delayed_work rst_dl_fw_work;
void Grip_check_lock_status(void){
	if(mutex_is_locked(&snt8100fsr_g->ap_lock))
		PRINT_INFO("AP_Lock is locked");
	if(mutex_is_locked(&snt8100fsr_g->sb_lock))
		PRINT_INFO("SB_Lock is locked");
	if(mutex_is_locked(&snt8100fsr_g->tap_lock))
		PRINT_INFO("TAP_Lock is locked");
}

/*

static int g_retry_times = 25;
int Grip_Wait_for_DL_FW(void){
	int count = 0;
	grip_input_event_report(29273, 0, 0, 0, 0, 0, 0);
	snt8100fsr_g->chip_reset_flag = GRIP_RST_FW_DL;
	while(snt8100fsr_g->chip_reset_flag != GRIP_FW_DL_END && count <= g_retry_times){
		count++;
		msleep(1000);
		PRINT_INFO("wait fw loading...,%d", count);
	}
	PRINT_INFO("wait fw loading done");
	if(count >= g_retry_times || snt8100fsr_g->service_load_fw_status == false){
		return E_FAILURE;
	}
	return E_FINISHED;
}
*/
/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int upload_firmware_fwd(struct snt8100fsr *snt8100fsr, char *filename) {
	PRINT_FUNC();
	INIT_DELAYED_WORK(&rst_dl_fw_work, download_wq_func);
	snt8100fsr_g->chip_reset_flag = GRIP_RST_FW_DL;

	/* [Todo] use vendor/asusfw path by default */
	/*
	if(fw_version == 0){
		rst_work->filename = FW_PATH_ASUSFW;
	}else{
		rst_work->filename = FW_PATH_VENDOR;
	}
	*/
	// Setup our logging level
	set_bcr_log_lvl(BCR_LOGGING_LEVEL);
	set_bcr_addr(BCR_ADDRESS_FILTER);
	set_bcr_irpt_lvl(BCR_INTERRUPT_TYPE);
	set_bcr_irpt_pol(BCR_INTERRUPT_POLARITY);
#if BCR_INTERRUPT_TYPE==BCR_IRQ_TYPE_PULSE
	// Setup interrupt edge duration
	set_bcr_irpt_dur(0x3f); //one equals 0.0128, so the duratin is 0x3f * 0.0128 = 63*0.0128 = 0.8064 us
#endif

	PRINT_INFO("1. upload_firmware_fwd: start first download_wq");
	workqueue_queue_work(&rst_dl_fw_work, 0);

	return 0;
}


int upload_firmware(struct snt8100fsr *snt8100fsr, char *filename) {
	// We can't upload the firmware directly as we need to pace
	// ourselves for how fast the hardware can accept data. So we
	// must setup a background thread with a workerqueue and process
	// the upload a piece at a time.
	PRINT_INFO("START");

	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	snt8100fsr_g->chip_reset_flag = GRIP_FST_FW_DL;
	mutex_unlock(&snt8100fsr_g->ap_lock);

	// Setup our logging level
	set_bcr_log_lvl(BCR_LOGGING_LEVEL);
	set_bcr_addr(BCR_ADDRESS_FILTER);
	set_bcr_irpt_lvl(BCR_INTERRUPT_TYPE);
	set_bcr_irpt_pol(BCR_INTERRUPT_POLARITY);
#if BCR_INTERRUPT_TYPE==BCR_IRQ_TYPE_PULSE
	// Setup interrupt edge duration
	set_bcr_irpt_dur(0x3f); //one equals 0.0128, so the duratin is 0x3f * 0.0128 = 63*0.0128 = 0.8064 us
#endif

	//schedule_delayed_work(&own_work, msecs_to_jiffies(60));
	PRINT_INFO("END");
	return 0;
}

int irq_handler_fwd( void) {
	int delay;
	PRINT_INFO("Enter irq_handler_fwd");
	// Add a delay in milliseconds if our boot loader is logging output.
	// During it's logging, it can't receive data, so we delay a bit.
	// We have a known amount of delay, so it's always safe.
	if (BCR_LOGGING_LEVEL != BCR_LOGLVL_OFF) {
		delay = FIRMWARE_LOG_DELAY_MS;
	} else {
		delay = 0;
	}
	return 0;
}
void upload_wq_func(struct work_struct *work_orig) {
	static int null_count = 0;
	//static int vendor_flag_count = 0;
	/*
	if(vendor_flag_count >= UPLOAD_FW_WQ_DELAY_LIMIT){
		PRINT_INFO("over wq delay limitation");
		goto start_upload;
	}
	*/
	while(snt8100fsr_g == NULL && null_count < GRIP_WAIT_FW_PROBE){
		PRINT_INFO("null snt8100fsr_g, delay 1s");
		null_count++;
		msleep(1000);
	}
	if(snt8100fsr_g==NULL){
		PRINT_INFO("wait struct create timeout, return");
		return;
	}

	while(snt8100fsr_g->hw_id_status != 1 && null_count < GRIP_WAIT_FW_PROBE){
		PRINT_INFO("Failed to read hw_id, wait 1s");
		msleep(1000);
	}
	if(snt8100fsr_g->hw_id_status != 1 && null_count >= GRIP_WAIT_FW_PROBE){
		PRINT_ERR("Failed to read hw_id, give up fw loading!");
		return;
	}
	/*
	else if(snt8100fsr_g->fpc_vendor_flag < 0){
		PRINT_INFO("bad fpc_vendor_flag value, delay 1s");
		msleep(1000);
		schedule_delayed_work(&own_work, msecs_to_jiffies(0));
		vendor_flag_count++;
		return;
	}
	*/

	upload_firmware_internal();
	PRINT_DEBUG("SNT upload_wq_func done");
	return;
}

static void download_wq_func(struct work_struct *work_orig) {
	//download_firmware_internal((struct upload_work *)work);
	download_firmware_internal();
	PRINT_DEBUG("SNT download_wq_func done");
	return;
}

static irqreturn_t irq_handler_top(int irq, void *dev) {
	PRINT_INFO("Enter");
	return IRQ_HANDLED;
}

//extern bool g_Charger_mode;
/*
static bool g_Charger_mode = false;
static int open_firmware_file(struct upload_work *w) {
	int ret = 0, i = 0;
	PRINT_FUNC("0x%p", w);
	if(g_Charger_mode) {
		//ASUSEvtlog("[Grip] Sensor: Charger mode, don't load fw!!!");
		return -1;
	}
	// If we haven't opened the firmware file, do so
	if (w->f == 0) {
		PRINT_INFO("Opening file: %s", w->filename);
		while(i < retry_times){
			ret = file_open(w->filename, O_RDONLY, 0, &w->f);
			if(ret) {
				if( (i%5) == 0){
					PRINT_ERR("Unable to open firmware file '%s', error %d",
					w->filename, ret);
				}
				msleep(1000);
				i++;
			}else{
				break;
			}
		}
	}
	if(i > retry_times){
		//ASUSEvtlog("[Grip] Sensor: can't find firmware file!!!");
	}
	return ret;
}
*/
/*
static error_t firmware_waiting_irq( struct upload_work *w) {
	PRINT_FUNC();
	if(FIRMWARE_UPLOAD_WITH_IRQ) {
		if (w->waiting_for_irq) {
			PRINT_CRIT("Timeout waiting for interrupt. Please ensure hardware "
						 "is correctly wired and firmware image is valid.");
			w->waiting_for_irq = false;
			return E_TIMEOUT;
		}

		w->waiting_for_irq = true;

		if(snt8100fsr_g->chip_reset_flag == GRIP_FST_FW_DL){
			PRINT_INFO("own_work");
			workqueue_queue_work(&own_work, FIRMWARE_IRQ_TIMEOUT_MS);
		}else{
			PRINT_INFO("Call reset IRQ workqueue: 2s");
			workqueue_queue_work(&rst_dl_fw_work, FIRMWARE_IRQ_TIMEOUT_MS_RST);
		}
	}
	return E_SUCCESS;
}
*/
/*
uint8_t grip_array[8][30][2560];
int grip_count1[8][30];
bool skip_reload = false;
static error_t firmware_transfer_payloads( struct upload_work *w) {
	int ret, index;
	int num_write;
	uint32_t payload_write;
	uint32_t payload_size;
	uint32_t size;
	uint32_t payload_duration;
	int count1 = 0, count2 = 0;
	PRINT_FUNC();
	memset(&grip_array, -1, sizeof(grip_array));
	memset(&grip_count1, 0, sizeof(grip_count1));

	num_write = file_read(w->f, w->file_offset, (void *)w->data_out, SPI_FIFO_SIZE);
	if (num_write <= 0) {
		PRINT_INFO("EOF Reached. Firmware data uploaded.");
		return E_FINISHED;
	}

	w->file_offset += SPI_FIFO_SIZE;
	w->payload_num++;

	// Size is first long word of buffer read
	payload_size = ((uint32_t*)w->data_out)[0];
	size = payload_size;
	PRINT_INFO("Payload %d = %d Bytes (%d inc 'size' field)", w->payload_num, size, size + 4);
	PRINT_INFO("data0=0x%02x, data10x%02x", w->data_out[0], w->data_out[1]);
	// If this is first segment, then pad word is boot cfg
	if (w->payload_num == 1) {
		((uint32_t*)w->data_out)[1] = set_bcr_word();
		PRINT_INFO("0x%02x", w->data_out[1] );

		// Record the start time of the first payload to measure the total
		// time the firmware upload takes to complete.
		w->firmware_upload_time = get_time_in_ms();
		PRINT_INFO("Payload1: data0=0x%02x, data10x%02x", w->data_out[0], w->data_out[1]);
	}

	// Record the start of the transfer of this payload
	w->payload_upload_time = get_time_in_ms();

	// Write the size out to chip
	ret = sb_read_and_write(w->snt8100fsr, num_write, w->data_out, w->data_in);
	if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
		PRINT_NOTICE("Existing firmware already loaded...");
		return E_ALREADY;
	} else if (ret) {
		PRINT_ERR("sb_write() failed");
		return E_FAILURE;
	}else{
	}

	if(skip_reload){
		skip_reload = false;
		return E_FAILURE;
	}

	w->total_bytes_written += num_write;
	size -= (SPI_FIFO_SIZE - sizeof(size));

	// Fatal if read_size not /8
	if (size % SPI_FIFO_SIZE) {
		PRINT_ERR("Size not multiple of %d", SPI_FIFO_SIZE);
		return E_BADSIZE;
	}

	PRINT_INFO("payload=%d, num_write=%d, total_byte_written=0x%02x", w->payload_num, num_write, w->total_bytes_written);

	for(index = 0; index < num_write; index++){
		if(grip_count1[w->payload_num-1][count2] < 2560){
			PRINT_INFO("C1, 2:%d, %d, I:%d, 0x%02x", count1, count2, index, w->data_out[index]);
			grip_array[w->payload_num-1][count2][count1] = w->data_out[index];
			grip_count1[w->payload_num-1][count2]++;
			count1++;
		}
		if(grip_count1[w->payload_num-1][count2] >= 2560){
			PRINT_INFO("change array num to %d", count2+1);
			count2++;
			count1 = 0;
		}
	}

	if((index%8)==0){
		pr_info("snt: index=%d, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
			index,
		w->data_out[index-8],
		w->data_out[index-7],
		w->data_out[index-6],
		w->data_out[index-5],
		w->data_out[index-4],
		w->data_out[index-3],
		w->data_out[index-2],
		w->data_out[index-1]);
	}

	// Get payload and write it out in SNT_FWDL_BUF_SIZE chunks
	payload_write = 0;
	while (size != 0 && payload_write < MAX_PAYLOAD_BYTES) {
		int read_size = min((unsigned int)SNT_FWDL_BUF_SIZE, size);


		num_write = file_read(w->f, w->file_offset, (void*)w->data_out, read_size);
		if (num_write <= 0) {
			PRINT_DEBUG("EOF Reached. Stopping...");
			return E_BADREAD;
		}

		w->file_offset += read_size;
		PRINT_INFO("payload cur: num_write=%d, %s", num_write, w->data_out);
		ret = sb_read_and_write(w->snt8100fsr, num_write, w->data_out, w->data_in);
		if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
			PRINT_NOTICE("Existing firmware already loaded...");
			return E_ALREADY;
		} else if (ret) {
			PRINT_ERR("sb_write() failed");
			return E_BADWRITE;
		}
		w->total_bytes_written += num_write;
		size -= num_write;
		payload_write += num_write;
		PRINT_INFO("===================C1, 2:%d, %d",count1, count2);
		for(index = 0; index < num_write; index++){
			if(grip_count1[w->payload_num-1][count2] < 2560){
				//PRINT_INFO("count:%d, %d, 0x%02x", count1, grip_count1[w->payload_num-1][count2], w->data_out[index]);
				grip_array[w->payload_num-1][count2][count1] = w->data_out[index];
				grip_count1[w->payload_num-1][count2]++;
				count1++;
				//msleep(10);
			}
			if(grip_count1[w->payload_num-1][count2] >= 2560){
				//PRINT_INFO("change array num to %d", count2+1);
				count2++;
				count1 = 0;
			}
			if((index+1)%8==0){
				pr_info("snt: index=%d, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
					index,
				w->data_out[index-7],
				w->data_out[index-6],
				w->data_out[index-5],
				w->data_out[index-4],
				w->data_out[index-3],
				w->data_out[index-2],
				w->data_out[index-1],
				w->data_out[index]);
			}
			//msleep(1);
		}
	}
	PRINT_INFO("payload=%d, num_write=%d, total_byte_written=%u", w->payload_num, num_write, w->total_bytes_written);
	count2 = 0;
	// Calculate how long this total payload took
	payload_duration = get_time_in_ms() - w->payload_upload_time;
	if (payload_duration == 0)
		payload_duration = 1;

	PRINT_INFO("Payload %d took %dms at %d kbit/s", w->payload_num, payload_duration,
				((payload_size * 8 / payload_duration) * 1000) / 1024);

	if (w->payload_num >= MAX_PAYLOAD_NUM) {
		PRINT_DEBUG("Max Payload Reached. Stopping...");
		return E_TOOMANY;
	}

	if (FIRMWARE_UPLOAD_WITH_IRQ) {
		PRINT_INFO("Waiting for IRQ for next payload");
	} else {
		PRINT_INFO("workqueue_queue_work()");
		//workqueue_queue_work(w, FIRMWARE_UPLOAD_DELAY_MS);
		if(snt8100fsr_g->chip_reset_flag == GRIP_FST_FW_DL){
			workqueue_queue_work(&own_work, FIRMWARE_UPLOAD_DELAY_MS);
		} else {
			//PRINT_INFO("4. firmware_transfer_payloads: download wq trigger");
			workqueue_queue_work(&rst_dl_fw_work, FIRMWARE_UPLOAD_DELAY_MS);
		}
	}
	return E_SUCCESS;
}
*/

static error_t grip_load_firmware_func_overall(const unsigned char *fw_buf,int fwsize){
	int ret = 0;
	int i = 0;
	int fw_data_size = 0;
	long long int cur_total_read = 0; //total size index
	long long int need_to_read = fwsize, this_payload_size = 0;
	long long int cur_payload_read = 0;
	unsigned char buf[512];
	uint32_t payload_size;
	snt8100fsr_g->payload_count = 0;

	while(cur_total_read < fwsize){
			snt8100fsr_g->service_load_fw_status = true;
			if(fw_data_size == 0){
				fw_data_size = 8;
			}else if(fw_data_size == 8){
				fw_data_size = 512;
			}
			if(fw_data_size == 512)
				if(fw_data_size > need_to_read)
					fw_data_size = need_to_read;

			memcpy(buf, fw_buf+cur_total_read, fw_data_size*sizeof(unsigned char));
			if(fw_data_size == 8){
				payload_size = (buf[1] << 8) | buf[0];
				need_to_read = payload_size + 4;
				this_payload_size = payload_size;
				PRINT_INFO("Payload %d = %d Bytes (%d inc 'size' field)(0x%02x)", snt8100fsr_g->payload_count , payload_size, payload_size + 4, buf[0]);
				if(snt8100fsr_g->payload_count == 0){
					buf[4] = 0xfe;
					buf[5] = 0x06;
					PRINT_INFO("write special case in first payload1");
				}
			}
			//PRINT_INFO("cur fw_data index=%d, cur_payload_index=%d, read size %d, need to read=%d",
			//	cur_total_read, cur_payload_read, fw_data_size, need_to_read);
			for(i = 0; i < fw_data_size; i++){
				if(i%8==0 && (i+7) < fw_data_size){
					i+=7;
/*
						pr_info("snt: index=%d, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
							i,
							buf[i-7],
							buf[i-6],
							buf[i-5],
							buf[i-4],
							buf[i-3],
							buf[i-2],
							buf[i-1],
							buf[i]);
*/
				}
			}
			if (fw_data_size % SPI_FIFO_SIZE) {
				PRINT_ERR("Size not multiple of %d", SPI_FIFO_SIZE);
				return E_BADSIZE;
			}
			ret = sb_read_and_write(snt8100fsr_g, fw_data_size, buf, NULL);
			if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
				PRINT_NOTICE("Existing firmware already loaded...");
				snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
				snt8100fsr_g->grip_fw_loading_status = false;
				snt8100fsr_g->payload_count = 0;
				snt8100fsr_g->service_load_fw_status = false;
				return E_BADWRITE;
			} else if (ret) {
				PRINT_ERR("sb_write() failed");
				PRINT_INFO("cur index=%d, read size %d, need to read=%d",cur_total_read, fw_data_size, need_to_read);
				snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
				snt8100fsr_g->grip_fw_loading_status = false;
				snt8100fsr_g->payload_count = 0;
				snt8100fsr_g->service_load_fw_status = false;
				return E_BADWRITE;
			}
			need_to_read -= fw_data_size;
			cur_total_read += fw_data_size;
			cur_payload_read += fw_data_size;
			if(cur_payload_read >= this_payload_size){
				PRINT_INFO("payload%d done, write %d", snt8100fsr_g->payload_count, cur_payload_read);
				if(snt8100fsr_g->payload_count == 5){
					PRINT_INFO("payload over 6, return 0");
					snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
					snt8100fsr_g->grip_fw_loading_status = true;
					snt8100fsr_g->payload_count = 0;
					break;
				}else{
					snt8100fsr_g->payload_count++;
				}
				fw_data_size = 0; //reset fw_data_size for first write of each payload
				cur_payload_read = 0;
			}
	}
	return E_FINISHED;
}
static void upload_firmware_internal() {
	/*
	 * insmod driver causes firmware to be uploaded to chip
	 */
	int ret, retry_count = 20;
	error_t err = E_SUCCESS;
	const struct firmware *fw = NULL;
	PRINT_INFO("Enter");
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);

	if(snt8100fsr_g == NULL){
		PRINT_INFO("snt8100fsr is NULL, do nothing");
		mutex_unlock(&snt8100fsr_g->ap_lock);
		return;
	}

	/* prevent load fw again when load status = 1 */
	if(snt8100fsr_g->grip_fw_loading_status == true){
		mutex_unlock(&snt8100fsr_g->ap_lock);
		return;
	}

	// If we haven't opened the firmware file, do so
	// [dy] unique to upload_firmware
	// Register our interrupt handler
	if(FIRMWARE_UPLOAD_WITH_IRQ) {
		snt_irq_db.top = irq_handler_top;
		ret = gpio_request(snt8100fsr_g->hostirq_gpio, IRQ_NAME);
		gpio_direction_input(snt8100fsr_g->hostirq_gpio); //output high
		if (ret) {
			PRINT_INFO("gpio_request ERROR(%d). \n", ret);
		}
		ret = gpio_direction_input(snt8100fsr_g->hostirq_gpio);
		if (ret) {
			PRINT_INFO("gpio_direction_input ERROR(%d). \n", ret);
		}
		snt_irq_db.irq_num = gpio_to_irq(snt8100fsr_g->hostirq_gpio);
		irq_handler_register(&snt_irq_db);
	}
	PRINT_INFO("request_firmware...");
	while(retry_count > 0){
		ret = request_firmware(&fw, REQUEST_GRIP_FW_PATH, snt8100fsr_g->dev);
		if (ret) {
			if(retry_count %5 == 0)
				PRINT_ERR("Error: request_firmware failed!!!");
			msleep(1000);
			retry_count--;
		}else{
			break;
		}
	}
	PRINT_INFO("check retry_count=%d", retry_count);
	if(retry_count <= 0){
		err = E_FAILURE;
		mutex_unlock(&snt8100fsr_g->ap_lock);
		goto cleanup;
	}
	PRINT_INFO("check grip_fw_loading_status");
	if(snt8100fsr_g->grip_fw_loading_status != true){
		err = grip_load_firmware_func_overall(fw->data, fw->size);
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
	if (err >= E_FAILURE) {
		PRINT_CRIT("firmware_transfer_payloads err %d", (int)err);
	goto cleanup;
	}
	else if (err == E_FINISHED) { goto cleanup; }
	PRINT_INFO("END");
	return;
cleanup:
	PRINT_INFO("firmware loading done");
	if(FIRMWARE_UPLOAD_WITH_IRQ) {
		irq_handler_unregister(&snt_irq_db);  //[dy] unique to upload_firmware
	}

	/* ASUS BSP Clay: protec status change */
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
	mutex_unlock(&snt8100fsr_g->ap_lock);
	Grip_check_lock_status();
	if(err <= E_SUCCESS) { //[dy] unique to upload_firmware
		snt8100fsr_g->grip_fw_loading_status = true;
		MUTEX_LOCK(&snt8100fsr_g->ap_lock);
		start_event_processing(snt8100fsr_g);
		PRINT_INFO("SUCESS to load fw");
#ifdef GRIP_APPLY_ASUSEVTLOG
		ASUSEvtlog("[Grip] Sensor: Load fw Success!!!\n");
#endif
		grip_input_event_report(8888, 0, 0, 0, 0, 0, 0);
		snt8100fsr_g->fw_failed_count = 0;
		if(snt8100fsr_g->Recovery_Flag){
			queue_delayed_work(asus_wq, &rst_recovery_wk, msecs_to_jiffies(0));
		}
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}else{
		PRINT_INFO("FAILED to load fw");
#ifdef GRIP_APPLY_ASUSEVTLOG
		ASUSEvtlog("[Grip] Sensor: Load fw fail!!!\n");
#endif
		grip_input_event_report(5566, 0, 0, 0, 0, 0, 0);
		snt8100fsr_g->fw_failed_count++;
		snt8100fsr_g->grip_fw_loading_status = false;
		MUTEX_LOCK(&snt8100fsr_g->ap_lock);
		start_event_processing(snt8100fsr_g);
		//Power_Control(0);
		queue_delayed_work(asus_wq, &rst_gpio_wk, msecs_to_jiffies(1000));
		PRINT_INFO("Call reset work");
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}
	Grip_check_lock_status();
	return;
}

static void download_firmware_internal(void) {
	/*
	 * Chip reset sets event register bit FWD so driver must
	 * download chip firmware.
	 * IRQ already set up by start_event_processing
	 */
	int ret, retry_count = 20;
	error_t err = E_SUCCESS;
	const struct firmware *fw = NULL;
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_FUNC("0x%p", w);
	PRINT_INFO("Enter download_firmware_internal");
	PRINT_INFO("request_firmware...");
	while(retry_count > 0){
		ret = request_firmware(&fw, REQUEST_GRIP_FW_PATH, snt8100fsr_g->dev);
		if (ret) {
			if(retry_count %5 == 0)
				PRINT_ERR("Error: request_firmware failed!!!");
			msleep(1000);
			retry_count--;
		}else{
			break;
		}
	}
	if(retry_count <= 0){
		err = E_FAILURE;
		mutex_unlock(&snt8100fsr_g->ap_lock);
		goto cleanup;
	}
	if(snt8100fsr_g->grip_fw_loading_status != true){
		err = grip_load_firmware_func_overall(fw->data, fw->size);
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
	if (err == E_FINISHED) { goto cleanup; }
	return;
cleanup:
	PRINT_INFO("firmware loading done");

	/* ASUS BSP Clay: protec status change */
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
	mutex_unlock(&snt8100fsr_g->ap_lock);

	Grip_check_lock_status();
	// [dy] 2017-09-01 fwd done, set context_fwd to false
	// [dy] 2017-09-05 initializations after reset
	set_context_fwd_done(snt8100fsr_g);
	if(err <= E_SUCCESS) { //[dy] unique to upload_firmware
		snt8100fsr_g->grip_fw_loading_status = true;
		ASUS_Handle_Reset(snt8100fsr_g);
		snt8100fsr_g->fw_failed_count = 0;
		PRINT_INFO("Sucess to reload fw");
		grip_input_event_report(8888, 0, 0, 0, 0, 0, 0);
	}else{
		PRINT_INFO("FAILED to load fw");
#ifdef GRIP_APPLY_ASUSEVTLOG
		ASUSEvtlog("[Grip] Sensor: Retry Load fw fail!!!\n");
#endif
		grip_input_event_report(5566, 0, 0, 0, 0, 0, 0);
		snt8100fsr_g->fw_failed_count++;
		snt8100fsr_g->grip_fw_loading_status = false;
		queue_delayed_work(asus_wq, &rst_gpio_wk, msecs_to_jiffies(1000));
	}
	Grip_check_lock_status();
	return;
}
