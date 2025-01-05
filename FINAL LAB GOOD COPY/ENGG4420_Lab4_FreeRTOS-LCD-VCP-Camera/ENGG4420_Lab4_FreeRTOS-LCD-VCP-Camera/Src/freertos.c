#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


/* USER CODE BEGIN Includes */     
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "spi.h"
#include "i2c.h"
#include "camera.h"

osMessageQId myQueue01Handle;
osMutexId myMutex01Handle;

/* USER CODE BEGIN Variables */
uint8_t RxData[256];
uint8_t Str4Display[50];

uint32_t data_received;

osThreadId TaskGUIHandle;
osThreadId TaskCOMMHandle;
osThreadId TaskBTNHandle;
osThreadId TaskSensorHandle;
osThreadId TaskAlertHandle;
osThreadId TaskCameraHandle;

osMessageQId CommQueueHandle;

osMutexId dataMutexHandle;
osMutexId printMutexHandle;

osSemaphoreId MotionSensorHandle;
osSemaphoreId DismissAlertHandle;
osSemaphoreId FakeHandle;
osSemaphoreId DisplayAlertHandle;
osSemaphoreId CaptureHandle;
osSemaphoreId ClearHandle;
int camera_on = 0;
int do_capture = 0;
int do_alert = 0;


typedef struct
{
	uint8_t Value[10];
	uint8_t Source;
}data;

data DataToSend={"Hello\0", 1};
data DataVCP={"VCP\0",2};



extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void StartTaskCOMM(void const * argument);
void StartTaskBTN(void const * argument);
void StartTaskGUI(void const * argument);

void StartTaskCamera(void const * argument);
void StartTaskSensor(void const * argument);
void StartTaskAlert(void const * argument);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {



	osMutexDef(dataMutex);
	dataMutexHandle = osMutexCreate(osMutex(dataMutex));

	osMutexDef(printMutex);
	printMutexHandle = osMutexCreate(osMutex(printMutex));

	//define the semaphores
	osSemaphoreDef(MotionSensor);
	MotionSensorHandle = osSemaphoreCreate(osSemaphore(MotionSensor), 1);

	osSemaphoreDef(DismissAlert);
	DismissAlertHandle = osSemaphoreCreate(osSemaphore(DismissAlert), 1);

	osSemaphoreDef(Imposter);
	FakeHandle = osSemaphoreCreate(osSemaphore(Imposter), 1);

	osSemaphoreDef(DisplayAlert);
	DisplayAlertHandle = osSemaphoreCreate(osSemaphore(DisplayAlert), 1);

	osSemaphoreDef(Capture);
	CaptureHandle = osSemaphoreCreate(osSemaphore(Capture), 1);

	osSemaphoreDef(ClearGUI);
	ClearHandle = osSemaphoreCreate(osSemaphore(ClearGUI), 1);


	osMessageQDef(myQueue01, 1, data);
	myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

	osMessageQDef(CommQueue, 1, &DataVCP);
	CommQueueHandle = osMessageCreate(osMessageQ(CommQueue), NULL);



	osThreadDef(TaskCOMM, StartTaskCOMM, osPriorityHigh, 0, 128);
	TaskCOMMHandle = osThreadCreate(osThread(TaskCOMM), NULL);

	osThreadDef(TaskBTN, StartTaskBTN, osPriorityLow, 0, 128);
	TaskBTNHandle = osThreadCreate(osThread(TaskBTN), NULL);

	osThreadDef(TaskGUI, StartTaskGUI, osPriorityAboveNormal, 0, 128);
	TaskGUIHandle = osThreadCreate(osThread(TaskGUI), NULL);


	osThreadDef(TaskCamera, StartTaskCamera, osPriorityNormal, 0, 128);
	TaskCameraHandle = osThreadCreate(osThread(TaskCamera), NULL);

	osThreadDef(TaskSensor, StartTaskSensor, osPriorityNormal, 0, 128);
	TaskSensorHandle = osThreadCreate(osThread(TaskSensor), NULL);

	osThreadDef(TaskAlert, StartTaskAlert, osPriorityNormal, 0, 128);
	TaskAlertHandle = osThreadCreate(osThread(TaskAlert), NULL);
}


void StartTaskCOMM(void const * argument)
{

  osEvent vcpValue;

  while(1)
  {
	  vcpValue = osMessageGet(CommQueueHandle, osWaitForever);
	  osMutexWait(dataMutexHandle, 0);
	  memcpy(Str4Display,(char *)(((data *)vcpValue.value.p)->Value), data_received+1);
	  osMutexRelease(dataMutexHandle);

	  //Type s to stop in serial terminal
	  if (Str4Display[0] == 's')
	  {
		  printf("False Alarm\n\r");
		  do_alert = 0;


	  }
  }
}


void StartTaskGUI(void const * argument)
{
  while(1)
  {


	  BSP_LCD_SetFont(&Font16);
	  BSP_LCD_SetTextColor(LCD_COLOR_PURPLE);
	  BSP_LCD_DisplayStringAtLine(2, (uint8_t *)" Real Time Group 14");
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_DrawHLine(0, 55, 240);

	  osSemaphoreWait(CaptureHandle, osWaitForever);
	  camera_initiate_capture();


	  while(!(osSemaphoreWait(DisplayAlertHandle, 1))){
		  BSP_LCD_SetTextColor(LCD_COLOR_RED);
		  BSP_LCD_DisplayStringAtLine(4, (uint8_t *)" Criminal Detected:(");
		  osSemaphoreRelease(ClearHandle);
	  }


	  osDelay(200);
	  BSP_LCD_DisplayStringAtLine(4, (uint8_t *)"                                                                                                  ");

  }
}

void StartTaskBTN(void const * argument)
{

  while(1) {

	  osSemaphoreWait(ClearHandle, 0);
	  if(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == GPIO_PIN_SET){

		  printf("clear\n\r");

		  do_alert = 0;
		  while(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN)==GPIO_PIN_SET);
	  }
	  osDelay(50);
  }
}

void StartTaskSensor(void const * argument)
{
	int i = 0;
	while (1)
	{
	  if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET)) // if the pin is HIGH
	  {
		  i++;
		  if(i > 6){
			  i = 0;
			  while ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET)){//wait for pin to go low
				  osSemaphoreRelease(MotionSensorHandle);
				  do_alert =1;
			  }
		  }
	  }

	  //osDelay(50);
	}
}


void StartTaskCamera(void const * argument)
{

	while (1) {

		if (!camera_on) {
			camera_setup();
			camera_on = 1;
		}

		//start capture if and only the intruder is detected
		printf("\n\r");

		if(osSemaphoreWait(FakeHandle, 1) && !do_alert){
			osDelay(2000);
		}
		else{
			printf("Intruder\n\r");
			osSemaphoreRelease(CaptureHandle);
			osDelay(100);

		}

	}
}


void StartTaskAlert(void const * argument)
{
	while (1) {

		osSemaphoreWait(MotionSensorHandle, osWaitForever);
		while(do_alert){
			osSemaphoreRelease(FakeHandle);
			osSemaphoreRelease(DisplayAlertHandle);


			HAL_GPIO_WritePin(GPIOG,LD3_Pin|LD4_Pin,GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin (GPIOG,LD3_Pin|LD4_Pin, 0); // LED OFF
			HAL_Delay(200);
		}
				//camera timing stuff

	}
}
