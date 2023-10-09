#include <limits>
#include <bit>
#include <string_view>
#include <type_traits>
#include <array>

#include <cstdio>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#  include "main.h"
#pragma GCC diagnostic pop

extern "C" int __io_putchar(int ch) { return ITM_SendChar(ch); }

namespace aux
{
    inline auto get(auto volatile& reg, auto mask) noexcept {
        return (reg & mask);
    }
    inline void drop(auto volatile& reg) noexcept {
    	[[maybe_unused]] auto volatile tmp = reg;
    }
    inline void reset(auto volatile& reg, auto val = 0) noexcept {
        reg = val;
    }
    inline void set(auto volatile& reg, auto val) noexcept {
    	reg = reg | val;
    }
    inline void set(auto volatile& reg, auto val, auto clear) noexcept {
    	reg = (reg & ~clear) | val;
    }
    template <auto VALUE, auto MASK>
    inline void set_with_delay(auto volatile& reg) noexcept {
    	constexpr auto absolute_value = VALUE << std::countr_zero(MASK);
    	if constexpr (1 < std::popcount(MASK)) {
    		set(reg, absolute_value, MASK);
    	}
    	else {
    		set(reg, absolute_value);
    	}
    	drop(reg);
    }

    template <class INTERFACE, size_t BASE,
              class REGISTER, size_t INDEX, size_t OFFSET,
              REGISTER... MASK>
    class sfrx {
    public:
        using interface_type = INTERFACE;
        using register_type = REGISTER;

    public:
        static constexpr auto address = BASE + OFFSET + INDEX * sizeof (register_type);
        static constexpr auto digits = std::numeric_limits<register_type>::digits;
        static constexpr auto nof_masks = sizeof... (MASK);
        static constexpr auto combined_mask = (MASK | ...);
        static constexpr auto combined_clear = ((std::has_single_bit(MASK) ? 0 : MASK) | ...);

    private:
        static inline register_type volatile& load_register() noexcept {
            return *reinterpret_cast<register_type volatile *const>(address);
        }

        template <register_type... VALUE>
        static void check_values() noexcept {
            static_assert(sizeof... (MASK) == sizeof... (VALUE));
            static_assert(0 == ((VALUE & ~(MASK >> std::countr_zero(MASK))) | ...));
        }

    public:
        static auto load() noexcept {
            return std::array<register_type, nof_masks>{
                (get(load_register(), MASK) >> std::countr_zero(MASK))...
            };
        }

        template <register_type... VALUE>
        static void confirm() noexcept {
            check_values<VALUE...>();
            constexpr register_type combined_absolute_value = ((VALUE << std::countr_zero(MASK)) | ...);
            while (get(load_register(), combined_mask) != combined_absolute_value) continue;
        }

        template <register_type... VALUE>
        static void reset() noexcept {
            check_values<VALUE...>();
            constexpr REGISTER combined_absolute_value = ((VALUE << std::countr_zero(MASK)) | ...);
            set(load_register(), combined_absolute_value);
        }

        template <register_type... VALUE>
        static void store() noexcept {
            check_values<VALUE...>();
            constexpr REGISTER combined_absolute_value = ((VALUE << std::countr_zero(MASK)) | ...);
            if constexpr (combined_clear) {
                set(load_register(), combined_absolute_value, combined_clear);
            }
            else {
                set(load_register(), combined_absolute_value);
            }
        }
        template <register_type... VALUE>
        static void store_confirm() noexcept {
            store<VALUE...>();
            confirm<VALUE...>();
        }

        template <register_type... VALUE>
        static void store_delay() noexcept {
            check_values<VALUE...>();
            (set_with_delay<VALUE, MASK>(load_register()), ...);
        }

    public:
        static auto debug_string() noexcept {
            return __PRETTY_FUNCTION__;
        }

    private:
        static_assert(std::is_unsigned_v<register_type>);
        static_assert(0 < nof_masks);
        static_assert(((digits == std::countl_zero(MASK) + std::popcount(MASK) + std::countr_zero(MASK)) && ...));
        static_assert(std::popcount(combined_mask) == (std::popcount(MASK) + ...));
    };
} // ::aux

#define SFRX_IMPL_REC_9(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_9(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_REC_8(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_9(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_REC_7(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_8(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_REC_6(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_7(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_REC_5(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_6(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_REC_4(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_5(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_REC_3(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_4(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_REC_2(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_3(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_REC_1(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_2(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_REC_0(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_1(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_APPLY(BIN,L,R,...) BIN(L,R)__VA_OPT__(,SFRX_IMPL_REC_0(BIN,L,__VA_ARGS__))
#define SFRX_IMPL_CONCAT_MSK(SFR,M) SFR##_##M##_Msk
#define SFRX(I,R,...)                                                    \
    aux::sfrx<std::remove_pointer_t<decltype (I)>,                       \
              std::string_view("FLASH") == #I ? FLASH_R_BASE : I##_BASE, \
              std::remove_cv_t<decltype (I->R)>,                         \
              0,                                                         \
              offsetof(std::remove_pointer_t<decltype (I)>, R),          \
              SFRX_IMPL_APPLY(SFRX_IMPL_CONCAT_MSK, I##_##R, __VA_ARGS__)>
#define SFxRX(I,x,R,...)                                                    \
    aux::sfrx<std::remove_pointer_t<decltype (I##x)>,                       \
              std::string_view("FLASH") == #I ? FLASH_R_BASE : I##x##_BASE, \
              std::remove_cv_t<decltype (I##x->R)>,                         \
              0,                                                            \
              offsetof(std::remove_pointer_t<decltype (I##x)>, R),          \
              SFRX_IMPL_APPLY(SFRX_IMPL_CONCAT_MSK, I##_##R, __VA_ARGS__)>
#define SFRnX(I,R,n,...)                                                     \
    aux::sfrx<std::remove_pointer_t<decltype (I)>,                           \
              std::string_view("FLASH") == #I ? FLASH_R_BASE : I##_BASE,     \
              std::remove_reference_t<std::remove_cv_t<decltype (I->R[0])>>, \
              (n-1),                                                         \
              offsetof(std::remove_pointer_t<decltype (I)>, R),              \
              SFRX_IMPL_APPLY(SFRX_IMPL_CONCAT_MSK, I##_##R##n, __VA_ARGS__)>
#define SFxRnX(I,x,R,n,...)                                                     \
    aux::sfrx<std::remove_pointer_t<decltype (I##x)>,                           \
              std::string_view("FLASH") == #I ? FLASH_R_BASE : I##x##_BASE,     \
              std::remove_reference_t<std::remove_cv_t<decltype (I##x->R[0])>>, \
              (n-1),                                                            \
              offsetof(std::remove_pointer_t<decltype (I##x)>, R),              \
              SFRX_IMPL_APPLY(SFRX_IMPL_CONCAT_MSK, I##_##R##n, __VA_ARGS__)>


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

inline auto sleep_for_ms(uint32_t delay_ms) noexcept {
	//uint32_t delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(delay).count();
	if (delay_ms == 0) delay_ms++;
	aux::drop(SysTick->CTRL);
	while (delay_ms) {
		if (auto [flag] = SFRX(SysTick, CTRL, COUNTFLAG)::load(); flag != 0) {
		  delay_ms--;
		}
	}
}

int main(void)
{
    SFRX(RCC, APB2ENR, SYSCFGEN)::store_delay<1>();
    SFRX(RCC, APB1ENR, PWREN)::store_delay<1>();
//  /* MCU Configuration--------------------------------------------------------*/
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (auto [c13i] = SFxRX(GPIO, C, IDR, ID13)::load(); c13i) {
		  sleep_for_ms(500);

		  USART2->DR = '*';
	  }
	  else {
		  sleep_for_ms(125);

		  USART2->DR = '-';
	  }
	  SFxRX(USART, 2, SR, TXE)::confirm<1>();

	  if (auto [a05o] = SFxRX(GPIO, A, ODR, OD5)::load(); a05o) {
		  SFxRX(GPIO, A, BSRR, BS5, BR5)::reset<0, 1>();
	  }
	  else {
		  SFxRX(GPIO, A, BSRR, BS5, BR5)::reset<1, 0>();
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    SFRX(FLASH, ACR, LATENCY)::store_confirm<2>();
//  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
//  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
//  {
//  }
    SFRX(PWR, CR, VOS)::store<3>();
//  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    SFRX(RCC, CR, HSITRIM)::store<16>();
//  LL_RCC_HSI_SetCalibTrimming(16);
    SFRX(RCC, CR, HSION)::store<1>();
//  LL_RCC_HSI_Enable();

    SFRX(RCC, CR, HSIRDY)::confirm<1>();
//   /* Wait till HSI is ready */
//  while(LL_RCC_HSI_IsReady() != 1)
//  {
//
//  }
    SFRX(RCC, PLLCFGR, PLLSRC, PLLM, PLLN, PLLP)::store<0, 16, 336, 1>();
//  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_16, 336, LL_RCC_PLLP_DIV_4);
    SFRX(RCC, CR, PLLON)::store_confirm<1>();
//  LL_RCC_PLL_Enable();
//   /* Wait till PLL is ready */
//  while(LL_RCC_PLL_IsReady() != 1)
//  {
//
//  }
    SFRX(PWR, CSR, VOSRDY)::confirm<1>();
//  while (LL_PWR_IsActiveFlag_VOS() == 0)
//  {
//  }
    SFRX(RCC, CFGR, HPRE, PPRE1, PPRE2, SW)::store<0, 4, 0, 2>();
//  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
//  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
//  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
//  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    SFRX(RCC, CFGR, SWS)::confirm<2>();
//   /* Wait till System clock is ready */
//  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
//  {
//
//  }
    constexpr uint32_t HCLK_FREQ = 84'000'000;
    constexpr uint32_t TICKS_RES = 1'000;
    SysTick->LOAD = HCLK_FREQ / TICKS_RES - 1;
    SysTick->VAL = 0;
    SFRX(SysTick, CTRL, CLKSOURCE, ENABLE)::reset<1, 1>();
//  LL_Init1msTick(84000000);
    SystemCoreClock = HCLK_FREQ;
//  LL_SetSystemCoreClock(84000000);
    SFRX(RCC, DCKCFGR, TIMPRE)::store<0>();
//  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
	SFRX(RCC, APB1ENR, USART2EN)::store_delay<1>();
//  /* Peripheral clock enable */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	SFRX(RCC, AHB1ENR, GPIOAEN)::store_delay<1>();
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	SFxRX(GPIO, A, OSPEEDR, OSPEED2, OSPEED3)::store<3, 3>();
	SFxRX(GPIO, A, OTYPER, OT2, OT3)::store<0, 0>();
	SFxRX(GPIO, A, PUPDR, PUPD2, PUPD3)::store<0, 0>();
//	SFxRnX(GPIO, A, AFR, 1, AFSEL2, AFSEL3)::store<0, 0>();
	aux::set(GPIOA->AFR[0],
			 (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos),
			 GPIO_AFRL_AFSEL2_Msk | GPIO_AFRL_AFSEL3_Msk);
	SFxRX(GPIO, A, MODER, MODER2, MODER3)::store<2, 2>();
//  /**USART2 GPIO Configuration
//  PA2   ------> USART2_TX
//  PA3   ------> USART2_RX
//  */
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {};
//  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  LL_USART_InitTypeDef USART_InitStruct = {};
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
    SFxRX(USART, 2, CR2, LINEN, CLKEN)::store<0, 0>();
//  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    SFRX(RCC, AHB1ENR, GPIOCEN, GPIOHEN, GPIOAEN, GPIOBEN)::store_delay<1,1,1,1>();
//  /* GPIO Ports Clock Enable */
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    SFxRX(GPIO, A, BSRR, BR5)::store<1>();
//  /**/
//  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

    SFRnX(SYSCFG, EXTICR, 4, EXTI13)::store<2>();
//  /**/
//  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

    SFRX(EXTI, EMR, MR13)::store<0>();
    SFRX(EXTI, IMR, MR13)::store<1>();
    SFRX(EXTI, RTSR, TR13)::store<0>();
    SFRX(EXTI, FTSR, TR13)::store<1>();

//  /**/
//  LL_EXTI_InitTypeDef EXTI_InitStruct = {};
//  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
//  EXTI_InitStruct.LineCommand = ENABLE;
//  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
//  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
//  LL_EXTI_Init(&EXTI_InitStruct);

    SFxRX(GPIO, C, PUPDR, PUPD13)::store<0>();
//  /**/
//  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

    SFxRX(GPIO, C, MODER, MODER13)::store<0>();
//  /**/
//  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

    SFxRX(GPIO, A, OSPEEDR, OSPEED5)::store<0>();
    SFxRX(GPIO, A, OTYPER, OT5)::store<0>();
    SFxRX(GPIO, A, PUPDR, PUPD5)::store<0>();
    SFxRX(GPIO, A, MODER, MODER5)::store<1>();

//  /**/
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {};
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
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
