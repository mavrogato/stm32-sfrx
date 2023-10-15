#include <limits>
#include <bit>
#include <string_view>
#include <type_traits>
#include <array>

#include <cstdio>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#  include <stm32f411xe.h>
#pragma GCC diagnostic pop
#include <core_cm4.h>

extern "C" int __io_putchar(int ch) { return ITM_SendChar(ch); }

namespace sfr
{
    using word = uint32_t;

    template <class T>
    constexpr bool has_single_continued_pop(T x) noexcept {
        constexpr size_t digits = std::numeric_limits<uint8_t>::digits * sizeof (T);
        return digits == std::popcount(x) + std::countr_zero(x) + std::countl_zero(x);
    }

    /////////////////////////////////////////////////////////////////////////////
    template <class R>
    concept as_sfr = std::unsigned_integral<R> && std::is_volatile_v<R>;

    namespace impl
    {
        template <as_sfr R>
        inline auto get(R& reg) noexcept {
            return reg;
        }
        template <as_sfr R, class T = std::remove_cv_t<R>>
        inline auto get(R& reg, T msk) noexcept {
            return reg & msk;
        }
        // template <class R, class T = std::remove_cv_t<R>>
        // inline void spin(R& reg, T val) noexcept {
        //         while (reg != val) continue;
        // }
        template <class R, class T = std::remove_cv_t<R>>
        inline void spin(R& reg, T val, T msk) noexcept {
            while ((reg & msk) != val) continue;
        }
        template <as_sfr R>
        inline void drop(R& reg) noexcept {
            [[maybe_unused]] R tmp = reg;
            static_assert(std::is_volatile_v<decltype (tmp)>);
        }
        // template <as_sfr R, class T = std::remove_cv_t<R>>
        // inline void drain(R& reg, T msk) noexcept {
        //         [[maybe_unused]] R tmp = reg & msk;
        //         static_assert(std::is_volatile_v<decltype (tmp)>);
        // }
        template <class R, class T = std::remove_cv_t<R>>
        inline void reset(R& reg, T val) noexcept {
            reg = val;
        }
        template <class R, class T = std::remove_cv_t<R>>
        inline void set(R& reg, T val) noexcept {
            reg = reg | val;
        }
        template <class R, class T = std::remove_cv_t<R>>
        inline void set(R& reg, T val, T msk) noexcept {
            reg = (reg & ~msk) | val;
        }
        template <class R, class T = std::remove_cv_t<R>>
        inline void set_and_drop(R& reg, T val) noexcept {
            reg = reg | val;
            drop(reg);
        }
        template <class R, class T = std::remove_cv_t<R>>
        inline void set_and_drop(R& reg, T val, T msk) noexcept {
            reg = (reg & msk) | val;
            drop(reg);
        }
    }

    /////////////////////////////////////////////////////////////////////////////
    template <word... args>
    class masked_value {
        static_assert(0 < sizeof... (args), "Empty not allowed.");
        static_assert(0 == (1 & sizeof... (args)), "Even elements expected.");

        static constexpr auto arguments_array = std::array<word, sizeof... (args)>{ args... };

    public:
        static constexpr size_t N = sizeof... (args) >> 1;

        template <size_t I>
        static constexpr word mask = std::get<I*2>(arguments_array);
        template <size_t I>
        static constexpr word value = std::get<I*2+1>(arguments_array);
        template <size_t I>
        static constexpr word absolute_value = value<I> << std::countr_zero(mask<I>);
        template <size_t I>
        static constexpr word clear_mask = std::has_single_bit(mask<I>) ? 0 : mask<I>;

        static constexpr auto combined = []<size_t... I>(std::index_sequence<I...>) noexcept {
            static_assert((has_single_continued_pop(mask<I>) && ...),
                "The mask must have single continued range poped up.");
            static_assert(std::popcount((mask<I> | ...)) == (std::popcount(mask<I>) + ...),
                "Overlapped masks.");
            static_assert(((0 == (absolute_value<I> & ~mask<I>)) && ...),
                "Value overflow.");
            return std::array<word, 3> {
                (mask<I> | ...),
                (absolute_value<I> | ...),
                (clear_mask<I> | ...),
            };
        }(std::make_index_sequence<N>());

        static constexpr word combined_mask = std::get<0>(combined);
        static constexpr word combined_value = std::get<1>(combined);
        static constexpr word combined_clear = std::get<2>(combined);

    public:
        template <as_sfr R, size_t... I>
        static inline auto load(R& reg, std::index_sequence<I...>) noexcept {
            static_assert(combined_mask <= std::numeric_limits<R>::max());
            std::remove_cv_t<R> tmp = reg;
            return std::array<std::remove_cv_t<R>, N> {
                ((tmp & mask<I>) >> std::countr_zero(mask<I>))...
            };
        }
        template <as_sfr R>
        static inline void spin(R& reg) noexcept {
            static_assert(combined_mask <= std::numeric_limits<R>::max());
            impl::spin(reg, combined_value, combined_mask);
        }
        template <as_sfr R>
        static inline void reset(R& reg) noexcept {
            static_assert(combined_mask <= std::numeric_limits<R>::max());
            impl::reset(reg, combined_value);
        }
        template <as_sfr R>
        static inline void store(R& reg) noexcept {
            static_assert(combined_mask <= std::numeric_limits<R>::max());
            if constexpr (combined_clear) {
                impl::set(reg, combined_value, combined_clear);
            }
            else {
                impl::set(reg, combined_value);
            }
        }
        template <as_sfr R>
        static inline void store_spin(R& reg) noexcept {
            store(reg);
            impl::spin(reg, combined_value, combined_mask);
        }
        template <as_sfr R, size_t... I>
        static inline void store_drop(R& reg, std::index_sequence<I...>) noexcept {
            static_assert(combined_mask <= std::numeric_limits<R>::max());
            (((std::has_single_bit(mask<I>)
                    ? impl::set(reg, absolute_value<I>)
                    : impl::set(reg, absolute_value<I>, mask<I>)),
                impl::drop(reg)), ...);
        }
    };

//    template <word... args>
//    inline auto load(auto volatile& reg) noexcept {
//        constexpr size_t N = sizeof... (args);
////        return []<size_t... I>(std::index_sequence<I...>) noexcept {
////            return std::array<word, 2*N> {
////                args...
////            };
////        }(std::make_index_sequence<N>());
//        return masked_value<args...>::load(reg);
//    }
    template <word... args>
    inline auto load(auto volatile& reg) noexcept {
        constexpr size_t N = sizeof... (args);
        return masked_value<args...>::load(reg, std::make_index_sequence<N/2>());
    }
    template <word... args>
    inline void spin(auto volatile& reg) noexcept {
        masked_value<args...>::spin(reg);
    }
    template <word... args>
    inline void reset(auto volatile& reg) noexcept {
        masked_value<args...>::reset(reg);
    }
    template <word... args>
    inline void store(auto volatile& reg) noexcept {
        masked_value<args...>::store(reg);
    }
    template <word... args>
    inline void store_spin(auto volatile& reg) noexcept {
        masked_value<args...>::store_spin(reg);
    }
    template <word... args>
    inline void store_drop(auto volatile& reg) noexcept {
        masked_value<args...>::store_drop(reg, std::make_index_sequence<sizeof... (args) / 2>());
    }

    /////////////////////////////////////////////////////////////////////////////
    static_assert(masked_value<3, 1, 4, 1>::combined_mask == 7);
    static_assert(masked_value<3, 1, 4, 1>::combined_value == 5);
    static_assert(masked_value<3, 1, 4, 1>::combined_clear == 3);

} // ::sfr

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

inline void sleep_for_ms(uint32_t delay_ms) noexcept {
    //uint32_t delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(delay).count();
    if (delay_ms != std::numeric_limits<decltype (delay_ms)>::max()) ++delay_ms;
    sfr::impl::drop(SysTick->CTRL);
    while (delay_ms--) {
        sfr::spin<SysTick_CTRL_COUNTFLAG_Msk, 1>(SysTick->CTRL);
    }
}

int main(void)
{
    sfr::store_drop<RCC_APB2ENR_SYSCFGEN, 1>(RCC->APB2ENR);
    sfr::store_drop<RCC_APB1ENR_PWREN, 1>(RCC->APB1ENR);
//  /* MCU Configuration--------------------------------------------------------*/
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(0x00000007u);

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
    for (;;) {
        if (auto [i13] = sfr::load<GPIO_IDR_ID13, 0>(GPIOC->IDR); i13) {
            sleep_for_ms(500);
            USART2->DR = '*';
        }
        else {
            sleep_for_ms(125);
            USART2->DR = '-';
        }
        sfr::spin<USART_SR_TXE, 1>(USART2->SR);

        if (auto [o5] = sfr::load<GPIO_ODR_OD5, 0>(GPIOA->ODR); o5) {
            sfr::reset<
                GPIO_BSRR_BS5, 0,
                GPIO_BSRR_BR5, 1
            >(GPIOA->BSRR);
        }
        else {
            sfr::reset<
                GPIO_BSRR_BS5, 1,
                GPIO_BSRR_BR5, 0
            >(GPIOA->BSRR);
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
    sfr::store_spin<FLASH_ACR_LATENCY, 2>(FLASH->ACR);
//  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
//  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
//  {
//  }
    sfr::store<PWR_CR_VOS, 3>(PWR->CR);
//  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    sfr::store<
        RCC_CR_HSITRIM, 16,
        RCC_CR_HSION, 1
    >(RCC->CR);
//  LL_RCC_HSI_SetCalibTrimming(16);
//  LL_RCC_HSI_Enable();

    sfr::spin<RCC_CR_HSIRDY, 1>(RCC->CR);
//   /* Wait till HSI is ready */
//  while(LL_RCC_HSI_IsReady() != 1)
//  {
//
//  }
    sfr::store<
        RCC_PLLCFGR_PLLSRC, 0,
        RCC_PLLCFGR_PLLM, 16,
        RCC_PLLCFGR_PLLN, 336,
        RCC_PLLCFGR_PLLP, 1
    >(RCC->PLLCFGR);
//  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_16, 336, LL_RCC_PLLP_DIV_4);
    sfr::store_spin<RCC_CR_PLLON, 1>(RCC->CR);
//  LL_RCC_PLL_Enable();
//   /* Wait till PLL is ready */
//  while(LL_RCC_PLL_IsReady() != 1)
//  {
//
//  }
    sfr::spin<PWR_CSR_VOSRDY, 1>(PWR->CSR);
//  while (LL_PWR_IsActiveFlag_VOS() == 0)
//  {
//  }
    sfr::store<
        RCC_CFGR_HPRE, 0,
        RCC_CFGR_PPRE1, 4,
        RCC_CFGR_PPRE2, 0,
        RCC_CFGR_SW, 2
    >(RCC->CFGR);
//  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
//  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
//  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
//  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    sfr::spin<RCC_CFGR_SWS, 2>(RCC->CFGR);
//   /* Wait till System clock is ready */
//  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
//  {
//
//  }
    constexpr uint32_t HCLK_FREQ = 84'000'000;
    constexpr uint32_t TICKS_DIV = 1'000;
    SysTick->LOAD = HCLK_FREQ / TICKS_DIV - 1;
    SysTick->VAL = 0;
    sfr::reset<
        SysTick_CTRL_CLKSOURCE_Msk, 1,
        SysTick_CTRL_ENABLE_Msk, 1
    >(SysTick->CTRL);
//  LL_Init1msTick(84000000);
    SystemCoreClock = HCLK_FREQ;
//  LL_SetSystemCoreClock(84000000);
    sfr::store<RCC_DCKCFGR_TIMPRE, 0>(RCC->DCKCFGR);
//  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    sfr::store_drop<RCC_APB1ENR_USART2EN, 1>(RCC->APB1ENR);
//  /* Peripheral clock enable */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    sfr::store_drop<RCC_AHB1ENR_GPIOAEN, 1>(RCC->AHB1ENR);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    sfr::store<
        GPIO_OSPEEDR_OSPEED2, 3,
        GPIO_OSPEEDR_OSPEED3, 3
    >(GPIOA->OSPEEDR);
    sfr::store<
        GPIO_OTYPER_OT2, 0,
        GPIO_OTYPER_OT3, 0
    >(GPIOA->OTYPER);
    sfr::store<
        GPIO_PUPDR_PUPD2, 0,
        GPIO_PUPDR_PUPD3, 0
    >(GPIOA->PUPDR);
    sfr::store<
        GPIO_AFRL_AFSEL2, 7,
        GPIO_AFRL_AFSEL3, 7
    >(GPIOA->AFR[0]);
    sfr::store<
        GPIO_MODER_MODER2, 2,
        GPIO_MODER_MODER3, 2
    >(GPIOA->MODER);
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
    if (auto [ue] = sfr::load<USART_CR1_UE, 0>(USART2->CR1); !ue) {
        sfr::store<
            USART_CR1_M, 0,
            USART_CR1_PCE, 0,
            USART_CR1_PS, 0,
            USART_CR1_TE, 1,
            USART_CR1_RE, 1,
            USART_CR1_OVER8, 0
        >(USART2->CR1);
        sfr::store<USART_CR2_STOP, 0>(USART2->CR2);
        sfr::store<
            USART_CR3_RTSE, 0,
            USART_CR3_CTSE, 0
        >(USART2->CR3);

        uint32_t sclk_freq;
        switch (auto [sws] = sfr::load<RCC_CFGR_SWS, 0>(RCC->CFGR); sws) {
            case 0:
                sclk_freq = HSI_VALUE;
                break;
            case 1:
                sclk_freq = HSE_VALUE;
                break;
            case 2: {
                auto input_freq = RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC ? HSE_VALUE : HSI_VALUE;
                auto [pllm, plln, pllp] = sfr::load<
                    RCC_PLLCFGR_PLLM, 0,
                    RCC_PLLCFGR_PLLN, 0,
                    RCC_PLLCFGR_PLLP, 0
                >(RCC->PLLCFGR);
                sclk_freq = (input_freq / pllm * plln) / ((pllp + 1) * 2);
                break;
            }
            default:
                sclk_freq = HSI_VALUE;
        }
        auto [hpre, ppre] = sfr::load<
            RCC_CFGR_HPRE, 0,
            RCC_CFGR_PPRE1, 0
        >(RCC->CFGR);
        uint32_t hclk_freq = sclk_freq >> AHBPrescTable[hpre];
        uint32_t pclk_freq = hclk_freq >> APBPrescTable[ppre];
        auto div = ((uint32_t)((((uint64_t)(pclk_freq))*25)/(4*((uint64_t)(115200)))));
        auto divmant = div/100;
        auto divfreq = (((div - divmant * 100) * 16) + 50) / 100;
        USART2->BRR = (divmant << 4) + (divfreq & 0xf0) + (divfreq & 0x0f);
    }
//  LL_USART_InitTypeDef USART_InitStruct = {};
//  USART_InitStruct.BaudRate = 115200;
//  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
//  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
//  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
//  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
//  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
//  LL_USART_Init(USART2, &USART_InitStruct);
    sfr::store<
        USART_CR2_LINEN, 0,
        USART_CR2_CLKEN, 0
    >(USART2->CR2);
//  LL_USART_ConfigAsyncMode(USART2);
    sfr::store<USART_CR1_UE, 1>(USART2->CR1);
  //LL_USART_Enable(USART2);
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
    sfr::store_drop<
        RCC_AHB1ENR_GPIOCEN, 1,
        RCC_AHB1ENR_GPIOHEN, 1,
        RCC_AHB1ENR_GPIOAEN, 1,
        RCC_AHB1ENR_GPIOBEN, 1
    >(RCC->AHB1ENR);
//  /* GPIO Ports Clock Enable */
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    sfr::store<GPIO_BSRR_BR5, 1>(GPIOA->BSRR);
//  /**/
//  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

    sfr::store<SYSCFG_EXTICR4_EXTI13, 2>(SYSCFG->EXTICR[3]);
//  /**/
//  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

    sfr::store<EXTI_EMR_MR13, 0>(EXTI->EMR);
    sfr::store<EXTI_IMR_MR13, 1>(EXTI->IMR);
    sfr::store<EXTI_RTSR_TR13, 0>(EXTI->RTSR);
    sfr::store<EXTI_FTSR_TR13, 1>(EXTI->FTSR);
//  /**/
//  LL_EXTI_InitTypeDef EXTI_InitStruct = {};
//  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
//  EXTI_InitStruct.LineCommand = ENABLE;
//  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
//  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
//  LL_EXTI_Init(&EXTI_InitStruct);

    sfr::store<GPIO_PUPDR_PUPD13, 0>(GPIOC->PUPDR);
//  /**/
//  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

    sfr::store<GPIO_MODER_MODER13, 0>(GPIOC->MODER);
//  /**/
//  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

    sfr::store<GPIO_OSPEEDR_OSPEED5, 0>(GPIOA->OSPEEDR);
    sfr::store<GPIO_OTYPER_OT5, 0>(GPIOA->OTYPER);
    sfr::store<GPIO_PUPDR_PUPD5, 0>(GPIOA->PUPDR);
    sfr::store<GPIO_MODER_MODER5, 1>(GPIOA->MODER);
//  /**/
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {};
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}
