/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "lwip.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "menu.h"
#include "string.h"
#include "tftp_server.h"
#include "flash_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
 * @brief Retargets the C library printf function to the USART
 * @param None
 * @retval None
 */
extern struct netif gnetif;
typedef  void (*pFunction)(void);
pFunction JumpToApp;
uint32_t jump_addr;
uint32_t write_count = 0;
uint32_t crc_compare;
uint32_t file_length = 0;
uint32_t save_version, save_magic_number, save_just_updated;
//uint32_t *just_updated = (uint32_t *)0x08020010;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

PUTCHAR_PROTOTYPE;

void print_ip(char *msg, ip_addr_t *ip);
void print_ip_settings(u32_t *ip, u32_t *mask, u32_t *gw);
void StartingMsg();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
#define APP_FW_ADDR ADDR_FLASH_SECTOR_2_BANK1
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define magic_number (uint32_t)0x08020004
int version;
int software_version;
int is_loaded = 0;
int will_update = -1;
uint32_t filelength = 0;
uint32_t crc;

int flash_handle;
struct FlashHandle {
	uint32_t dest;
	uint8_t is_erased;
};

void* OpenCallback(const char* fname, const char* mode, u8_t write);
int ReadCallback(void* handle, void* buf, int bytes);
int WriteCallback(void* handle, struct pbuf* p);
int WriteCallback2(void* handle, struct pbuf* p);
void CloseCallback(void* handle);
void CloseCallbackReset(void* handle);
void BootLoader();
int is_firstwrite = 1;
int is_newestver;
int sent_version = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_LWIP_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
  version = *(uint32_t *)0x08020000;
  printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  printf("                                                                                                                                                                        (((((/                                         *(((((.\n");
  printf("                                                                                                                                                                        ((((((((((/                               /((((((((((,\n");
  printf("                                                                                                                                                                       .((((((((((((((((,                   .((((((((((((((((,\n");
  printf("                                                                                                                                                                       .(((((((((((((((((((((*         .(((((((((((((((((((((,\n");
  printf("                                                                                                                                                                       .((((((((((((((((((((.    /(((((((((((((((((((((((((((,\n");
  printf("                                                                                                                                                                       .################/  .#################################,\n");
  printf("                                                                                                                                                                       .############/ .######################################,\n");
  printf("                                                                                                                                                                       .#####################################################,\n");
  printf("                                                                                                                                                                             *#########################################/.\n");
  printf("                                                                                                                                                                                  *###############################/\n");
  printf("                                                                                                                                                                                         %%%%%%%%%%%%%%%%#%%##.\n");
  printf("                                                                                                                                                                                             /%%%%%%%%%%.\n\n\n\n\n");
  printf("                                                                                                                                   ______   __    __   ______   ________  __              _______    ______   _______    ______   ________  ______   ______    ______\n");
  printf("                                                                                                                                  /      \\ /  \\  /  | /      \\ /        |/  |            /       \\  /      \\ /       \\  /      \\ /        |/      | /      \\  /      \\\n");
  printf("                                                                                                                                 /$$$$$$  |$$  \\ $$ |/$$$$$$  |$$$$$$$$/ $$ |            $$$$$$$  |/$$$$$$  |$$$$$$$  |/$$$$$$  |$$$$$$$$/ $$$$$$/ /$$$$$$  |/$$$$$$  |\n");
  printf("                                                                                                                                 $$ |__$$ |$$$  \\$$ |$$ | _$$/ $$ |__    $$ |            $$ |__$$ |$$ |  $$ |$$ |__$$ |$$ |  $$ |   $$ |     $$ |  $$ |  $$/ $$ \\__$$/\n");
  printf("                                                                                                                                 $$    $$ |$$$$  $$ |$$ |/    |$$    |   $$ |            $$    $$< $$ |  $$ |$$    $$< $$ |  $$ |   $$ |     $$ |  $$ |      $$      \\ \n");
  printf("                                                                                                                                 $$$$$$$$ |$$ $$ $$ |$$ |$$$$ |$$$$$/    $$ |            $$$$$$$  |$$ |  $$ |$$$$$$$  |$$ |  $$ |   $$ |     $$ |  $$ |   __  $$$$$$  |\n");
  printf("                                                                                                                                 $$ |  $$ |$$ |$$$$ |$$ \\__$$ |$$ |_____ $$ |_____       $$ |  $$ |$$ \\__$$ |$$ |__$$ |$$ \\__$$ |   $$ |    _$$ |_ $$ \\__/  |/  \\__$$ |\n");
  printf("                                                                                                                                 $$ |  $$ |$$ | $$$ |$$    $$/ $$       |$$       |      $$ |  $$ |$$    $$/ $$    $$/ $$    $$/    $$ |   / $$   |$$    $$/ $$    $$/\n");
  printf("                                                                                                                                 $$/   $$/ $$/   $$/  $$$$$$/  $$$$$$$$/ $$$$$$$$/       $$/   $$/  $$$$$$/  $$$$$$$/   $$$$$$/     $$/    $$$$$$/  $$$$$$/   $$$$$$/\n");

  if(*(uint32_t *)0x08020000 == 1){
	  printf("\n");
	  printf("                                                                                                                                                                                                                      __\n");
	  printf("                                                                                                                                                                                                                    _/  |\n");
	  printf("                                                                                                                                                                                __     __  ______    ______        / $$ |\n");
	  printf("                                                                                                                                                                               /  \\   /  |/      \\  /      \\       $$$$ |\n");
	  printf("                                                                                                                                                                               $$  \\ /$$//$$$$$$  |/$$$$$$  |        $$ |\n");
	  printf("                                                                                                                                                                                $$  /$$/ $$    $$ |$$ |  $$/         $$ |\n");
	  printf("                                                                                                                                                                                 $$ $$/  $$$$$$$$/ $$ | __          _$$ |_\n");
	  printf("                                                                                                                                                                                  $$$/   $$       |$$ |/  |        / $$   |\n");
	  printf("                                                                                                                                                                                   $/     $$$$$$$/ $$/ $$/         $$$$$$/\n");
  }

  else if(*(uint32_t *)0x08020000 == 2){
  	  printf("\n");
  	  printf("                                                                                                                                                                                                                    ______\n");
  	  printf("                                                                                                                                                                                                                   /      \\\n");
  	  printf("                                                                                                                                                                               __     __  ______    ______        /$$$$$$  |\n");
  	  printf("                                                                                                                                                                              /  \\   /  |/      \\  /      \\       $$____$$ |\n");
  	  printf("                                                                                                                                                                              $$  \\ /$$//$$$$$$  |/$$$$$$  |       /    $$/\n");
  	  printf("                                                                                                                                                                               $$  /$$/ $$    $$ |$$ |  $$/       /$$$$$$/\n");
  	  printf("                                                                                                                                                                                $$ $$/  $$$$$$$$/ $$ | __         $$ |_____\n");
  	  printf("                                                                                                                                                                                 $$$/   $$       |$$ |/  |        $$       |\n");
  	  printf("                                                                                                                                                                                  $/     $$$$$$$/ $$/ $$/         $$$$$$$$/\n");
    }

  else if(*(uint32_t *)0x08020000 == 3){
	  printf("\n");
  	  printf("                                                                                                                                                                                                                    ______\n");
  	  printf("                                                                                                                                                                                                                   /      \\\n");
  	  printf("                                                                                                                                                                               __     __  ______    ______        /$$$$$$  |\n");
	  printf("                                                                                                                                                                              /  \\   /  |/      \\  /      \\       $$____$$ |\n");
	  printf("                                                                                                                                                                              $$  \\ /$$//$$$$$$  |/$$$$$$  |        /   $$<\n");
	  printf("                                                                                                                                                                               $$  /$$/ $$    $$ |$$ |  $$/        _$$$$$  |\n");
	  printf("                                                                                                                                                                                $$ $$/  $$$$$$$$/ $$ | __         /  \\__$$ |\n");
	  printf("                                                                                                                                                                                 $$$/   $$       |$$ |/  |        $$    $$/\n");
	  printf("                                                                                                                                                                                  $/     $$$$$$$/ $$/ $$/          $$$$$$/\n");
      }
  printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"); // 79 lines per window
  HAL_Delay(1000);

  /* editted from here */

  save_version = *(uint32_t *)0x08020000;
  save_magic_number = *(uint32_t *)0x08020004;
  save_just_updated = *(uint32_t *)0x08020008;

//  printf("%d %d %d\n", save_version, save_magic_number, save_just_updated);


  if(save_just_updated == 13579){

	  HAL_FLASH_Unlock();
	  FLASH_If_Init();
	  uint32_t SectorError;
	  FLASH_EraseInitTypeDef pEraseInit;
	  pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	  pEraseInit.Sector = FLASH_SECTOR_1;
	  pEraseInit.NbSectors = 1;
	  pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	  pEraseInit.Banks = FLASH_BANK_1;
	  if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK) {
	  /* Error occurred while sector erase */
		  printf("Error!\n");
	  }
	  HAL_FLASH_Lock();

	  uint32_t save_validation_set[3] = {save_version, save_magic_number, 24680};

	  FLASH_If_Init();
	  HAL_FLASH_Unlock();
	  if(FLASH_If_Write((uint32_t)0x08020000, save_validation_set, 3) != HAL_OK) printf("error!!!\n");
	  HAL_FLASH_Lock();

	  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
	  jump_addr = *(__IO uint32_t*) (APP_FW_ADDR + 4);
		JumpToApp = (pFunction) jump_addr;

	  __set_MSP(*(__IO uint32_t*) APP_FW_ADDR);
	  JumpToApp();
  }
 /* to here */

  else{
	  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_SET);
	  BootLoader();
  }





//  printf("\n");
//  printf("initial magic number : %d\n", *(uint32_t*)magic_number);
//  printf("current software version : %d\n", *(uint32_t *)0x08020000);
//  if ((HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) && (*(uint32_t *)magic_number == 12345678)) { // reboot without uploading
////	  printf("\nJumping to firmware area\n");
//	  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
//	  jump_addr = *(__IO uint32_t*) (APP_FW_ADDR + 4);
//      JumpToApp = (pFunction) jump_addr;
//
//	  __set_MSP(*(__IO uint32_t*) APP_FW_ADDR);
//	  JumpToApp();
//  } else {
////	  printf("loading bootloader\n");
//	  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_SET);
//	  BootLoader(); // if block -> original code
//
//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  StartingMsg();
//  print_ip_settings(&gnetif.ip_addr.addr, &gnetif.netmask.addr, &gnetif.gw.addr);
  while (1)
  {
//	  MX_LWIP_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE{
	if (ch == '\n') HAL_UART_Transmit(&huart3, (uint8_t*)"\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}

void print_ip(char *msg, ip_addr_t *ip)
{
	printf(msg);
	printf("%d.%d.%d.%d\n", ip4_addr1(ip), ip4_addr2(ip), ip4_addr3(ip), ip4_addr4(ip));
}

void print_ip_settings(u32_t *ip, u32_t *mask, u32_t *gw)
{
	print_ip("Board IP: ", (ip_addr_t *)ip);
	print_ip("Netmask : ", (ip_addr_t *)mask);
	print_ip("Gateway : ", (ip_addr_t *)gw);
	printf("\n");
}


void StartingMsg()
{
	char buf[1024];
	sprintf(buf, "\n\nSTM32 ETHERNET TEST: Clk=%dMHz", (int)(HAL_RCC_GetHCLKFreq()/1000000));
	printf("%s\n\n",buf);
}

void* OpenCallback(const char* fname, const char* mode, u8_t write)
{
//	printf("File name: %s\n", fname);

	struct FlashHandle* fh = malloc(sizeof(struct FlashHandle));
	fh->dest = APP_FW_ADDR;
	fh->is_erased = 0;

	return (void*)fh;
}

int ReadCallback(void* handle, void* buf, int bytes)
{
	uint32_t *xbuf = (uint32_t *)buf;
//	printf("%d\n", sent_version);
	if(sent_version == 0)	*xbuf = save_version;
	else{
		if(save_magic_number == 12345678)	*xbuf = 1;
		else *xbuf = 0;
	}
	// or
//	uint32_t xbuf[2] = (uint32_t *)buf;
//	xbuf = {version, save_magic_number};
//	printf("sending version info\n");
	return 1;
}

int WriteCallback2(void* handle, struct pbuf* p){
	will_update = *(uint32_t *)p->payload;
	return 0;
}

int WriteCallback(void* handle, struct pbuf* p)
{
//	printf("write\n");
	if(is_firstwrite){
		printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
		printf("                                                                                                 __    __                  __              __                      __                  _______                                                                                        \n");
		printf("                                                                                                /  |  /  |                /  |            /  |                    /  |                /       \\                                                                                       \n");
		printf("                                                                                                $$ |  $$ |  ______    ____$$ |  ______   _$$ |_     ______        $$/  _______        $$$$$$$  | ______    ______    ______    ______    ______    _______  _______                   \n");
		printf("                                                                                                $$ |  $$ | /      \\  /    $$ | /      \\ / $$   |   /      \\       /  |/       \\       $$ |__$$ |/      \\  /      \\  /      \\  /      \\  /      \\  /       |/       |                  \n");
		printf("                                                                                                $$ |  $$ |/$$$$$$  |/$$$$$$$ | $$$$$$  |$$$$$$/   /$$$$$$  |      $$ |$$$$$$$  |      $$    $$//$$$$$$  |/$$$$$$  |/$$$$$$  |/$$$$$$  |/$$$$$$  |/$$$$$$$//$$$$$$$/                   \n");
		printf("                                                                                                $$ |  $$ |$$ |  $$ |$$ |  $$ | /    $$ |  $$ | __ $$    $$ |      $$ |$$ |  $$ |      $$$$$$$/ $$ |  $$/ $$ |  $$ |$$ |  $$ |$$ |  $$/ $$    $$ |$$      \\$$      \\                   \n");
		printf("                                                                                                $$ \\__$$ |$$ |__$$ |$$ \\__$$ |/$$$$$$$ |  $$ |/  |$$$$$$$$/       $$ |$$ |  $$ |      $$ |     $$ |      $$ \\__$$ |$$ \\__$$ |$$ |      $$$$$$$$/  $$$$$$  |$$$$$$  |       __  __  __ \n");
		printf("                                                                                                $$    $$/ $$    $$/ $$    $$ |$$    $$ |  $$  $$/ $$       |      $$ |$$ |  $$ |      $$ |     $$ |      $$    $$/ $$    $$ |$$ |      $$       |/     $$//     $$/       /  |/  |/  |\n");
		printf("                                                                                                 $$$$$$/  $$$$$$$/   $$$$$$$/  $$$$$$$/    $$$$/   $$$$$$$/       $$/ $$/   $$/       $$/      $$/        $$$$$$/   $$$$$$$ |$$/        $$$$$$$/ $$$$$$$/ $$$$$$$/        $$/ $$/ $$/ \n");
		printf("                                                                                                          $$ |                                                                                                     /  \\__$$ |                                                         \n");
		printf("                                                                                                          $$ |                                                                                                     $$    $$/                                                          \n");
		printf("                                                                                                          $$/                                                                                                       $$$$$$/                                                           \n");
		printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
		software_version = *(uint32_t *)p->payload;
//		printf("arrived software version : %d\n", software_version);
		if(software_version == version){
			is_newestver = 0;
			is_firstwrite = 0;
		}
		else is_newestver = 1;
	}
	if(is_newestver){
		struct FlashHandle* fh = (struct FlashHandle*)handle;
		uint32_t res;
		if (fh->is_erased == 0) {
//			printf("Erase start\n");
			res = FLASH_If_Erase(APP_FW_ADDR);
//			printf("Erase result: %d\n", (int)res);
//			printf("Flash start\n");
			fh->is_erased = 1;
		}

		/* reading commuication packet */
		if(is_firstwrite){
			crc_compare = *((uint32_t *)p->payload + 4);
//			printf("First Communication Packet arrived\n");
//			printf("                                                                              #");
//			HAL_Delay(30);
		}
		else{
//			printf("Flash %3dB @ %p: ", (int)p->len, (void*)fh->dest);
			res = FLASH_If_Write(fh->dest, (uint32_t*)p->payload, p->len/sizeof(uint32_t));
			filelength = filelength + p->len;
			if (res == FLASHIF_OK) {
//				printf("done\n");
//				printf("#");
//				HAL_Delay(50);
			} else {
//				printf("failed\n");
			}
		}

		if(is_firstwrite){
			is_firstwrite = 0;
		}
		else{
			fh->dest += p->len;
		}
		return res;
	}
//	printf("Already newest version\n");
	return FLASHIF_OK;
}


void CloseCallback(void* handle)
{
	struct FlashHandle* fh = (struct FlashHandle*)handle;
	free(fh);
//	printf("Close\n");
	sent_version++;

}

void CloseCallbackReset(void* handle)
{
	struct FlashHandle* fh = (struct FlashHandle*)handle;
	free(fh);
//	printf("Close\n");
	is_loaded = 1;
//	printf("close\n");
}

void BootLoader() {
//	print_ip_settings(&gnetif.ip_addr.addr, &gnetif.netmask.addr, &gnetif.gw.addr);
//	printf("Bootloader on.. checking version..\n");
	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	printf("                                                                                                    __                                  __  __                            _______                         __      __                            __                           \n");
	printf("                                                                                                   /  |                                /  |/  |                          /       \\                       /  |    /  |                          /  |                          \n");
	printf("                                                                                                   $$ |        ______    ______    ____$$ |$$/  _______    ______        $$$$$$$  |  ______    ______   _$$ |_   $$ |  ______    ______    ____$$ |  ______    ______        \n");
	printf("                                                                                                   $$ |       /      \\  /      \\  /    $$ |/  |/       \\  /      \\       $$ |__$$ | /      \\  /      \\ / $$   |  $$ | /      \\  /      \\  /    $$ | /      \\  /      \\       \n");
	printf("                                                                                                   $$ |      /$$$$$$  | $$$$$$  |/$$$$$$$ |$$ |$$$$$$$  |/$$$$$$  |      $$    $$< /$$$$$$  |/$$$$$$  |$$$$$$/   $$ |/$$$$$$  | $$$$$$  |/$$$$$$$ |/$$$$$$  |/$$$$$$  |      \n");
	printf("                                                                                                   $$ |      $$ |  $$ | /    $$ |$$ |  $$ |$$ |$$ |  $$ |$$ |  $$ |      $$$$$$$  |$$ |  $$ |$$ |  $$ |  $$ | __ $$ |$$ |  $$ | /    $$ |$$ |  $$ |$$    $$ |$$ |  $$/       \n");
	printf("                                                                                                   $$ |_____ $$ \\__$$ |/$$$$$$$ |$$ \\__$$ |$$ |$$ |  $$ |$$ \\__$$ |      $$ |__$$ |$$ \\__$$ |$$ \\__$$ |  $$ |/  |$$ |$$ \\__$$ |/$$$$$$$ |$$ \\__$$ |$$$$$$$$/ $$ | __  __  __ \n");
	printf("                                                                                                   $$       |$$    $$/ $$    $$ |$$    $$ |$$ |$$ |  $$ |$$    $$ |      $$    $$/ $$    $$/ $$    $$/   $$  $$/ $$ |$$    $$/ $$    $$ |$$    $$ |$$       |$$ |/  |/  |/  |\n");
	printf("                                                                                                   $$$$$$$$/  $$$$$$/   $$$$$$$/  $$$$$$$/ $$/ $$/   $$/  $$$$$$$ |      $$$$$$$/   $$$$$$/   $$$$$$/     $$$$/  $$/  $$$$$$/   $$$$$$$/  $$$$$$$/  $$$$$$$/ $$/ $$/ $$/ $$/ \n");
	printf("                                                                                                                                                         /  \\__$$ |                                                                                                          \n");
	printf("                                                                                                                                                         $$    $$/                                                                                                           \n");
	printf("                                                                                                                                                          $$$$$$/                                                                                                            \n");
	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	struct tftp_context tftpctx1 = {
		  OpenCallback,
		  CloseCallback,
		  ReadCallback,
		  WriteCallback
	};
	tftp_init(&tftpctx1);
	while(sent_version < 2){
		MX_LWIP_Process();
	}
//	printf("tftp connection closed 1\n");
	tftp_cleanup();

	struct tftp_context tftpctx2 = {
			OpenCallback,
			CloseCallback,
			ReadCallback,
			WriteCallback2
	};
	tftp_init(&tftpctx2);
	while(will_update == -1){
		MX_LWIP_Process();
	}
	tftp_cleanup();
	if(!will_update){
//		printf("Already newest version!!\n");
		HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
		jump_addr = *(__IO uint32_t*) (APP_FW_ADDR + 4);
		JumpToApp = (pFunction) jump_addr;

		__set_MSP(*(__IO uint32_t*) APP_FW_ADDR);
		JumpToApp();
	}


	struct tftp_context tftpctx = {

		  OpenCallback,
		  CloseCallbackReset,
		  ReadCallback,
		  WriteCallback

	};

	/* initialize configuration sector */
	HAL_FLASH_Unlock();
	FLASH_If_Init();
	uint32_t SectorError;
	FLASH_EraseInitTypeDef pEraseInit;
	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	pEraseInit.Sector = FLASH_SECTOR_1;
	pEraseInit.NbSectors = 1;
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	pEraseInit.Banks = FLASH_BANK_1;
	if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK) {
	  /* Error occurred while sector erase */
	  printf("Error!\n");
	}
	HAL_FLASH_Lock();


	tftp_init(&tftpctx);
	FLASH_If_Init();
//	StartingMsg();
//	print_ip_settings(&gnetif.ip_addr.addr, &gnetif.netmask.addr, &gnetif.gw.addr);
	while (!is_loaded)
	{
	  MX_LWIP_Process();
	}
	tftp_cleanup();
	/* crc check */

	if(is_newestver){
		crc = HAL_CRC_Calculate(&hcrc, APP_FW_ADDR, filelength);
		crc = ~crc;

//		printf("file length : %d\n", filelength);
//		printf("crc value : %x\n", crc);
//		printf("received crc value : %x\n", crc_compare);
		uint32_t validation_set[3] = {software_version, 12345678, 13579};


		if(crc == crc_compare){
//			printf("\n! received valid data !\n");
			FLASH_If_Init();
			HAL_FLASH_Unlock();
//			FLASH_If_Init();
			if(FLASH_If_Write((uint32_t)0x08020000, validation_set, 3) != HAL_OK) printf("error!!!\n");
			HAL_FLASH_Lock();
//			printf("magic number updated to : %d\n", *(uint32_t *)magic_number);
//			printf("software version updated to : %d\n", *(uint32_t *)0x08020000);
//			printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
			for(int i = 0; i <= 81; i++){
				printf("\n");
				HAL_Delay(50);
			}
			HAL_Delay(1000);
		}
		else{
			HAL_Delay(1000);
//			printf("\n! received broken data !\n");

			printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
			printf(" _______                                 __                            __        _______                       __                                  _______               __                      __  __\n");
			printf("/       \\                               /  |                          /  |      /       \\                     /  |                                /       \\             /  |                    /  |/  |\n");
			printf("$$$$$$$  |  ______    _______   ______  $$/  __     __  ______    ____$$ |      $$$$$$$  |  ______    ______  $$ |   __   ______   _______        $$$$$$$  |  ______   _$$ |_     ______        $$ |$$ |\n");
			printf("$$ |__$$ | /      \\  /       | /      \\ /  |/  \\   /  |/      \\  /    $$ |      $$ |__$$ | /      \\  /      \ $$ |  /  | /      \\ /       \\       $$ |  $$ | /      \\ / $$   |   /      \\       $$ |$$ |\n");
			printf("$$    $$< /$$$$$$  |/$$$$$$$/ /$$$$$$  |$$ |$$  \\ /$$//$$$$$$  |/$$$$$$$ |      $$    $$< /$$$$$$  |/$$$$$$  |$$ |_/$$/ /$$$$$$  |$$$$$$$  |      $$ |  $$ | $$$$$$  |$$$$$$/    $$$$$$  |      $$ |$$ |\n");
			printf("$$$$$$$  |$$    $$ |$$ |      $$    $$ |$$ | $$  /$$/ $$    $$ |$$ |  $$ |      $$$$$$$  |$$ |  $$/ $$ |  $$ |$$   $$<  $$    $$ |$$ |  $$ |      $$ |  $$ | /    $$ |  $$ | __  /    $$ |      $$/ $$/ \n");
			printf("$$ |  $$ |$$$$$$$$/ $$ \\_____ $$$$$$$$/ $$ |  $$ $$/  $$$$$$$$/ $$ \\__$$ |      $$ |__$$ |$$ |      $$ \\__$$ |$$$$$$  \\ $$$$$$$$/ $$ |  $$ |      $$ |__$$ |/$$$$$$$ |  $$ |/  |/$$$$$$$ |       __  __ \n");
			printf("$$ |  $$ |$$       |$$       |$$       |$$ |   $$$/   $$       |$$    $$ |      $$    $$/ $$ |      $$    $$/ $$ | $$  |$$       |$$ |  $$ |      $$    $$/ $$    $$ |  $$  $$/ $$    $$ |      /  |/  |\n");
			printf("$$/   $$/  $$$$$$$/  $$$$$$$/  $$$$$$$/ $$/     $/     $$$$$$$/  $$$$$$$/       $$$$$$$/  $$/        $$$$$$/  $$/   $$/  $$$$$$$/ $$/   $$/       $$$$$$$/   $$$$$$$/    $$$$/   $$$$$$$/       $$/ $$/ \n");
			printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

		}
		HAL_NVIC_SystemReset();
	}
	else{
		uint32_t validation_set[3] = {version, 12345678, 13579};
		FLASH_If_Init();
		HAL_FLASH_Unlock();
//		FLASH_If_Init();
		if(FLASH_If_Write((uint32_t)0x08020000, validation_set, 3) != HAL_OK) printf("error!!!\n");
		HAL_FLASH_Lock();

		HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
		jump_addr = *(__IO uint32_t*) (APP_FW_ADDR + 4);
		JumpToApp = (pFunction) jump_addr;

		__set_MSP(*(__IO uint32_t*) APP_FW_ADDR);
		JumpToApp();

	}
}


/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

