/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "my_audio_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
QSPI_HandleTypeDef hqspi;

SAI_HandleTypeDef hsai_BlockA1;

/* USER CODE BEGIN PV */
// Example: A very simple 16-bit, 1kHz square wave at 16kHz sample rate
//#define SINE_BUFFER_SIZE 32
//int16_t sine_wave_buffer[SINE_BUFFER_SIZE] = {
//     30000,  30000,  30000,  30000,  30000,  30000,  30000,  30000,
//     30000,  30000,  30000,  30000,  30000,  30000,  30000,  30000,
//    -30000, -30000, -30000, -30000, -30000, -30000, -30000, -30000,
//    -30000, -30000, -30000, -30000, -30000, -30000, -30000, -30000
//};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SAI1_Init(void);
static void MX_QUADSPI_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// This handle is declared by CubeMX in main.c
extern QSPI_HandleTypeDef hqspi;

uint8_t QSPI_ReadStatusRegister(uint8_t reg_num)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg_val;
  uint8_t instruction = 0x05; // Default to Read Status Reg 1

  if (reg_num == 2) {
    instruction = 0x35; // Read Status Reg 2
  } else if (reg_num == 3) {
    instruction = 0x15; // Read Status Reg 3
  }

  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = instruction;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.NbData            = 1;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_QSPI_Receive(&hqspi, &reg_val, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  return reg_val;
}


/**
  * @brief  This function sends a Write Enable command (0x06)
  */
static void QSPI_WriteEnable(void)
{
  QSPI_CommandTypeDef sCommand;

  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = 0x06; // Write Enable
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function polls the flash's "Write in Progress" (WIP) bit.
  * It will not return until the flash is ready for new commands.
  */
static void QSPI_AutoPoll_WIP(void)
{
  QSPI_CommandTypeDef sCommand;
  QSPI_AutoPollingTypeDef sPoll;

  // Configure the command to read Status Register 1 (0x05)
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = 0x05; // Read Status Register 1
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  //... (other settings don't matter for this command)

  // Configure the polling
  sPoll.Mask           = 0x01; // Mask to check only the WIP bit (bit 0)
  sPoll.Match          = 0x00; // We wait until the WIP bit is 0
  sPoll.MatchMode      = QSPI_MATCH_MODE_AND;
  sPoll.StatusBytesSize= 1;
  sPoll.Interval       = 0x10;
  sPoll.AutomaticStop  = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sPoll, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Erases a 4KB sector of the flash at a given address
  */
static void QSPI_SectorErase(uint32_t SectorAddress)
{
  QSPI_CommandTypeDef sCommand;

  // 1. Send Write Enable
  QSPI_WriteEnable();

  // 2. Configure the Sector Erase command (0x20)
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = 0x20; // Sector Erase
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.Address           = SectorAddress;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  //... (other settings)

  // 3. Send the command
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  // 4. Wait for the erase to finish (can take ~45ms)
  QSPI_AutoPoll_WIP();
}

// We'll need these helper functions from your previous test
extern void QSPI_WriteEnable(void);
extern void QSPI_AutoPoll_WIP(void);

/**
  * @brief  Enables Quad SPI mode on the W25Q128.
  * It does this by setting the "QE" bit in Status Register 2.
  */
static void QSPI_EnableQuadMode(void)
{
    QSPI_CommandTypeDef sCommand;
    uint8_t reg_data = 0x02; // 0x02 is the value to set the QE bit
    uint8_t status_reg_1;

    // 1. Send Write Enable
    QSPI_WriteEnable();

    // ----------------- DEBUG CHECK -----------------
    // Read back Status Register 1 immediately
    status_reg_1 = QSPI_ReadStatusRegister(1);

    // The WEL bit (bit 1) MUST be set. The value should be 0x02.
    // (Bit 0 is WIP, should be 0. Bit 1 is WEL, should be 1)
    if (status_reg_1 != 0x02)
    {
      // If you land here, the Write Enable command FAILED.
      // The chip ignored it. Check the reasons below.
      Error_Handler();
    }
    // -----------------------------------------------

    // 2. Configure the "Write Status Register 2" command (0x31)
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = 0x31; // Write Status Register 2
    // ... (rest of the command config) ...

    if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        Error_Handler();
    }

    // 3. Send the 0x02 byte
    if (HAL_QSPI_Transmit(&hqspi, &reg_data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
    // This is where you are failing.
        Error_Handler();
    }

    // 4. Wait for the write to finish
    QSPI_AutoPoll_WIP();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_SAI1_Init();
  MX_QUADSPI_Init();


  /* USER CODE BEGIN 2 */

    uint8_t status;

    // 1. Check initial status
    status = QSPI_ReadStatusRegister(1);
    // Place breakpoint here: 'status' should be 0x00

    // 2. Try to enable write
    QSPI_WriteEnable();

    // 3. Check if Write Enable Latch (WEL) bit is set
    status = QSPI_ReadStatusRegister(1);
    // Place breakpoint here: 'status' MUST be 0x02.
    // The WEL bit (bit 1) must be 1.
    //
    // If 'status' is still 0x00, your Write Enable command failed.
    // This confirms write protection (either HW pin or SW bits).

    if ((status & 0x02) == 0)
    {
      // WEL bit is NOT set. Write Enable failed.
      // Most likely HW /WP pin is LOW or SW protection is on.
      Error_Handler();
    }

//    // --- If you get here, Write Enable worked ---
//
//    // 4. Now, try to send the erase command (this is part of QSPI_SectorErase)
//    QSPI_CommandTypeDef sEraseCommand;
//    sEraseCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
//    sEraseCommand.Instruction       = 0x20; // Sector Erase
//    sEraseCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
//    sEraseCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
//    sEraseCommand.Address           = 0x000000; // The address you want to erase
//    sEraseCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//    sEraseCommand.DataMode          = QSPI_DATA_NONE;
//    sEraseCommand.DummyCycles       = 0;
//    //... (other settings)
//
//    if (HAL_QSPI_Command(&hqspi, &sEraseCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    // 5. Check if the Write in Progress (WIP) bit is now set
//    status = QSPI_ReadStatusRegister(1);
//    // Place breakpoint here: 'status' MUST be 0x01 or 0x03.
//    // The WIP bit (bit 0) must be 1, showing the erase has started.
//    // The WEL bit (bit 1) will be auto-cleared back to 0.
//
//    if ((status & 0x01) == 0)
//    {
//      // WIP bit is NOT set. The flash ignored the erase command.
//      // This confirms write protection.
//      Error_Handler();
//    }
//
//    // 6. Now, try to poll for completion
//    QSPI_AutoPoll_WIP();
//
//    // 7. Check if WIP bit is clear
//    status = QSPI_ReadStatusRegister(1);
//    // Place breakpoint here: 'status' should be 0x00.
//    // This proves the erase finished and polling worked.
//
//    if (status != 0x00)
//    {
//      // Polling failed
//      Error_Handler();
//    }


    QSPI_EnableQuadMode();


      // --- Step 2: Configure and Enable Memory-Mapped Mode ---

      QSPI_CommandTypeDef sCommand;
      QSPI_MemoryMappedTypeDef sMemMappedCfg;

      // Configure the "Fast Read Quad Output" command (0x6B)
      // This is the command the QSPI hardware will auto-send
      // when you read from 0x90000000.

      sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
      sCommand.Instruction       = 0x6B;                      // Fast Read Quad Output
      sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
      sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;    // 16MB chip = 24-bit address
      sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      sCommand.DataMode          = QSPI_DATA_4_LINES;       // *** This is the "Quad" part! ***
      sCommand.DummyCycles       = 8;                         // W25Q128 needs 8 dummies for 0x6B
      sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
      sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
      sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

      // Configure the memory-mapped struct
      sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE; // No timeout

      // Enable memory-mapping!
      if (HAL_QSPI_MemoryMapped(&hqspi, &sCommand, &sMemMappedCfg) != HAL_OK)
      {
        // Failed to enter memory-mapped mode
        Error_Handler();
      }

      __NOP(); // Success breakpoint

      volatile uint8_t* p_flash = (volatile uint8_t*)0x90000000;

        uint8_t read_data[3];

        // Read the data just like a normal array
        // For each read, the QSPI hardware does the *full* 0x6B command
        // automatically in the background.
        read_data[0] = p_flash[0];
        read_data[1] = p_flash[1];
        read_data[2] = p_flash[2];


        // --- Step 4: Check the results ---
        if (read_data[0] == 0x00 && read_data[1] == 0x01 && read_data[2] == 0x02)
        {
          // Success!
          __NOP(); // Place breakpoint here
        }
        else
        {
          // Failure!
          // 1. Check your QE bit function.
          // 2. Check your 0x6B command settings (especially DummyCycles).
          // 3. Check your IO2 and IO3 wiring.
          Error_Handler();
        }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // new
      // The size is in 16-bit words,
      // but our size variable is in 8-bit bytes, so divide by 2.
      // https://community.st.com/t5/stm32-mcus-products/hal-sai-transmit-does-not-stop-the-i2s-clocks-when-complete/m-p/177117/highlight/true#M36926
      uint16_t audio_samples_count = g_nosoup_data_size / 2;

      // This function will block until the entire buffer is sent
//      HAL_SAI_Transmit(&hsai_BlockA1, (uint8_t*)g_nosoup_data, audio_samples_count, HAL_MAX_DELAY);


//      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//      HAL_Delay(1000);
//      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//      HAL_Delay(1000);

//      HAL_SuspendTick();
//      HAL_PWREx_EnterSTOP1Mode(PWR_SLEEPENTRY_WFI); // Stop2 Mode
//
//      HAL_ResumeTick();
//
//      // 5. Clear the Wake-up Source
//      __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_13); // Clear the EXTI Flag
//
//      // Wait a moment to ensure the user fully released the button (debounce)
//      HAL_Delay(200);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 4;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_MONOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
