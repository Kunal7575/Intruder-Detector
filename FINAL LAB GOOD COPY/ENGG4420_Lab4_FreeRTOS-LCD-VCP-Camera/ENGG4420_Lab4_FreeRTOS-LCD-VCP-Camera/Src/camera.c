

#include "camera.h"
#include "arducam.h"

#include "picojpeg_test.h"
#include "util.h"
#include "stlogo.h"
#include "stm32f429i_discovery_lcd.h"
#include "usbd_cdc_if.h"


#define MAX_PIC_SIZE 64000
#define BITMAP_SIZE 3600


static uint32_t captureStart;
uint8_t fifoBuffer[BURST_READ_LENGTH];
uint8_t fifoBuffer2[BURST_READ_LENGTH];
uint8_t pic_buffer[MAX_PIC_SIZE];
uint8_t pic_buffer2[MAX_PIC_SIZE];
uint8_t bitmap[BITMAP_SIZE];
int pic_index = 0;
int test = 0;
unsigned char cameraReady;

static void camera_get_image();
BaseType_t write_fifo_to_buffer(uint32_t length);

void camera_setup(){

	cameraReady = pdFALSE;

	if (   arduchip_detect()
		&& arducam_exit_standby()
		&& ov5642_detect()
	) {

		osDelay(100);

		if (!ov5642_configure()) {
			printf("camera_task: ov5642 configure failed\n\r");
			return;
		} else {
			printf("camera: setup complete\n\r");
			cameraReady = pdTRUE;
			osDelay(100);
		}
	} else {
		printf("camera: setup failed\n\r");
		cameraReady = pdTRUE;
	}
}

/**
 * Capture an image from the camera.
 */
void camera_initiate_capture(){

	uint8_t done = 0;

	printf("camera: initiate capture\n\r");

	if (!cameraReady) {
		printf("camera: set up camera before capture\n\r");
	}

	/* Initiate an image capture. */
	if (!arduchip_start_capture()) {
		printf("camera: initiate capture failed\n\r");
		return;
	}

	/* wait for capture to be done */
	captureStart = (uint32_t)xTaskGetTickCount();
	while(!arduchip_capture_done(&done) || !done){

		if ((xTaskGetTickCount() - captureStart) >= CAPTURE_TIMEOUT) {
			printf("camera: capture timeout\n\r");
			return;
		}
	}

	printf("camera: capture complete\n\r");

	camera_get_image();

	return;

}

void camera_get_image(){

	/* Determine the FIFO buffer length. */
	uint32_t length = 0;
	if (arduchip_fifo_length(&length) == pdTRUE) {
		printf("camera: captured jpeg image -> %lu bytes\n\r", length);
		write_fifo_to_buffer(length);
	} else {
		printf("camera: get fifo length failed\n\r");
	}

	return;
}


static void get_pixel(int* pDst, const uint8_t *pSrc, int luma_only, int num_comps)
{
    int r, g, b;

    // Case for grayscale (single component, just Y channel)
    if (num_comps == 1) {
        r = g = b = pSrc[0];  // If grayscale, all channels are the same
    }

    else if (luma_only) {

        int Y = pSrc[0];
        int U = pSrc[1];
        int V = pSrc[2];


        r = Y + 1.402 * (V - 128);
        g = Y - 0.344136 * (U - 128) - 0.714136 * (V - 128);
        b = Y + 1.772 * (U - 128);


        r = (r < 0) ? 0 : (r > 255) ? 255 : r;
        g = (g < 0) ? 0 : (g > 255) ? 255 : g;
        b = (b < 0) ? 0 : (b > 255) ? 255 : b;
    }
    // Case for RGB format (if the image is already RGB)
    else {
        r = pSrc[0];  // Red channel
        g = pSrc[1];  // Green channel
        b = pSrc[2];  // Blue channel
    }


    pDst[0] = r;
    pDst[1] = g;
    pDst[2] = b;
}


static void display_bitmap(const uint8_t * image, int width, int height, int comps, int start_x, int start_y, int scale)
{

	int a[3];

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			get_pixel(a, image + (y * width + x) * comps, 0, comps);
			uint32_t argb = (0xFF << 24) | (a[0] << 16) | (a[1] << 8) | (a[0]);
			for (int yy = 0; yy < scale; yy++) {
				for (int xx = 0; xx < scale; xx++) {
					BSP_LCD_DrawPixel(start_x + x * scale + xx, start_y + y * scale + yy, argb);
				}
			}
		}
	}
	uint32_t old_col = BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DrawRect(start_x, start_y, width * scale, height * scale);
	BSP_LCD_SetTextColor(old_col);
}

BaseType_t write_fifo_to_buffer(uint32_t length)
{
	if (test % 2 == 0){
	int width, height, comp;
	uint16_t chunk = 0;
	unsigned int jpeg_size = length*sizeof(uint8_t);

	if (jpeg_size >= MAX_PIC_SIZE) {
		printf("camera: not enough memory (%d/%d)\n\r", jpeg_size, MAX_PIC_SIZE);
		return pdFALSE;
	} else {
		printf("camera: reserved %d/%d\n\r", jpeg_size, MAX_PIC_SIZE);
	}

	pic_index = 0;
	for (uint16_t i = 0; length > 0; ++i) {

		chunk = MIN(length, BURST_READ_LENGTH);
		arduchip_burst_read(fifoBuffer, chunk);
		for (uint16_t j = 0; j < chunk; j++) {
			pic_buffer[pic_index] = fifoBuffer[j];
			pic_index += 1;
		}
		length -= chunk;
	}

	FILE * pic_stream = fmemopen(pic_buffer, jpeg_size, "rb");	//jpg2tga will close this, no need for fclose
	picojpg_test(pic_stream, &width, &height, &comp, bitmap);
	if(sizeof(bitmap) == BITMAP_SIZE){
		display_bitmap(bitmap, width, height, comp, 40, 100, 4);
	}else{
		return pdFALSE;
	}

	}
	else {
		int width, height, comp;
			uint16_t chunk = 0;
			unsigned int jpeg_size = length*sizeof(uint8_t);

			if (jpeg_size >= MAX_PIC_SIZE) {
				printf("camera: not enough memory (%d/%d)\n\r", jpeg_size, MAX_PIC_SIZE);
				return pdFALSE;
			} else {
				printf("camera: reserved %d/%d\n\r", jpeg_size, MAX_PIC_SIZE);
			}

			pic_index = 0;
			for (uint16_t i = 0; length > 0; ++i) {

				chunk = MIN(length, BURST_READ_LENGTH);
				arduchip_burst_read(fifoBuffer2, chunk);
				for (uint16_t j = 0; j < chunk; j++) {
					pic_buffer2[pic_index] = fifoBuffer2[j];
					pic_index += 1;
				}
				length -= chunk;
			}

			FILE * pic_stream = fmemopen(pic_buffer2, jpeg_size, "rb");	//jpg2tga will close this, no need for fclose
			picojpg_test(pic_stream, &width, &height, &comp, bitmap);
			if(sizeof(bitmap) == BITMAP_SIZE){
				display_bitmap(bitmap, width, height, comp, 40, 100, 4);
			}else{
				return pdFALSE;
			}

	}

	return pdTRUE;
}


