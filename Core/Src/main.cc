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
            [[maybe_unused]] R volatile tmp = reg;
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

    void compile_time_evaluation_failure(char const*...);

    template <std::unsigned_integral T> constexpr auto max = std::numeric_limits<T>::max();
    template <std::unsigned_integral T> constexpr auto digits = std::numeric_limits<T>::digits;
    template <std::unsigned_integral T>
    constexpr bool is_valid_mask(T x) noexcept {
        // has single continued pop, and empty not allowed.
        return digits<T> == std::popcount(x) + std::countr_zero(x) + std::countl_zero(x);
    }

//    template <class R>
//    concept as_sfr = std::unsigned_integral<R> && std::is_volatile_v<R>;

    using word = uint32_t;

    struct mask_value_list {
    public:
        template <size_t NN, size_t N = (NN >> 1)>
        consteval mask_value_list(word const (&mva)[NN]) noexcept
            : msk{max<word>}
            , val{}
            , clr{msk}
            {
                static_assert(0 < NN, "empty not allowed");
                static_assert(NN == 2*N, "even arguments required");
                [&]<size_t... I>(std::index_sequence<I...>) noexcept {
                    std::array<word const, N> ma{mva[2*I]...};
                    std::array<word const, N> va{(mva[2*I+1] << std::countr_zero(ma[I]))...};
                    this->msk = (ma[I] | ...);
                    this->val = (va[I] | ...);
                    this->clr = ((ma[I] ^ va[I]) | ...);
                    if (!(is_valid_mask(ma[I]) && ...)) {
                        compile_time_evaluation_failure("invalid mask");
                    }
                    if (std::popcount(msk) != (std::popcount(ma[I]) + ...)) {
                        compile_time_evaluation_failure("mask overlapped");
                    }
                    if (!(((va[I] & ~ma[I]) == 0) && ...)) {
                        compile_time_evaluation_failure("value overflowed");
                    }
                }(std::make_index_sequence<N>());
            }
    public:
        template <std::integral... Args>
        consteval mask_value_list(Args... args) noexcept
            : mask_value_list{{static_cast<word>(args)...}}
        {
        }

    public:
        word msk;
        word val;
        word clr;
    };

    constexpr void set(as_sfr auto& reg, mask_value_list mva) noexcept {
        reg = (reg & ~mva.clr) | mva.val;
    }
    constexpr void dip(as_sfr auto const& reg, word msk = max<word>) noexcept {
        [[maybe_unused]] auto volatile tmp = reg & msk;
    }
    constexpr void spn(as_sfr auto const& reg, mask_value_list mva) noexcept {
        while (mva.val != (reg & mva.msk)) continue;
    }
    constexpr void rst(as_sfr auto& reg, mask_value_list mva) noexcept {
        reg = mva.val;
    }

    template <size_t N>
    [[nodiscard]]
    constexpr auto val(as_sfr auto const& reg, word const (&ma)[N]) noexcept {
        if constexpr (N == 1) {
            return (reg & ma[0]) >> std::countr_zero(ma[0]);
        }
        else {
            return [&]<size_t... I>(std::index_sequence<I...>) noexcept {
                return std::array<word, N> {
                    ((reg & ma[I]) >> std::countr_zero(ma[I]))...
                };
            }(std::make_index_sequence<N>());
        }
    }

} // ::sfr

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

inline void sleep_for_ms(uint32_t delay_ms) noexcept {
    //uint32_t delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(delay).count();
    if (delay_ms != std::numeric_limits<decltype (delay_ms)>::max()) ++delay_ms;
    sfr::dip(SysTick->CTRL);
    while (delay_ms--) {
        sfr::spn(SysTick->CTRL, {SysTick_CTRL_COUNTFLAG_Msk, 1});
    }
}

int main(void)
{
    sfr::set(RCC->APB2ENR, {RCC_APB2ENR_SYSCFGEN, 1});
    sfr::dip(RCC->APB2ENR);
    sfr::set(RCC->APB1ENR, {RCC_APB1ENR_PWREN, 1});
    sfr::dip(RCC->APB1ENR);
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
        if (sfr::val(GPIOC->IDR, {GPIO_IDR_ID13})) {
            sleep_for_ms(500);
            sfr::rst(USART2->DR, {USART_DR_DR, '*'});
        }
        else {
            sleep_for_ms(125);
            sfr::rst(USART2->DR, {USART_DR_DR, '-'});
        }
        sfr::spn(USART2->SR, {USART_SR_TXE, 1});
        if (sfr::val(GPIOA->ODR, {GPIO_ODR_OD5})) {
            sfr::rst(GPIOA->BSRR, {
                GPIO_BSRR_BS5, 0,
                GPIO_BSRR_BR5, 1,
            });
        }
        else {
            sfr::rst(GPIOA->BSRR, {
                GPIO_BSRR_BS5, 1,
                GPIO_BSRR_BR5, 0,
            });
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
    sfr::set(FLASH->ACR, {FLASH_ACR_LATENCY, 2});
    sfr::spn(FLASH->ACR, {FLASH_ACR_LATENCY, 2});
//  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
//  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
//  {
//  }
    sfr::set(PWR->CR, {PWR_CR_VOS, 3});
//  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    sfr::set(RCC->CR, {
        RCC_CR_HSITRIM, 16,
        RCC_CR_HSION, 1,
    });
//  LL_RCC_HSI_SetCalibTrimming(16);
//  LL_RCC_HSI_Enable();

    sfr::spn(RCC->CR, {RCC_CR_HSIRDY, 1});
//   /* Wait till HSI is ready */
//  while(LL_RCC_HSI_IsReady() != 1)
//  {
//
//  }
    sfr::set(RCC->PLLCFGR, {
        RCC_PLLCFGR_PLLSRC, 0,
        RCC_PLLCFGR_PLLM, 16,
        RCC_PLLCFGR_PLLN, 336,
        RCC_PLLCFGR_PLLP, 1,
    });
//  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_16, 336, LL_RCC_PLLP_DIV_4);
    sfr::set(RCC->CR, {RCC_CR_PLLON, 1});
    sfr::spn(RCC->CR, {RCC_CR_PLLON, 1});
//  LL_RCC_PLL_Enable();
//   /* Wait till PLL is ready */
//  while(LL_RCC_PLL_IsReady() != 1)
//  {
//
//  }
    sfr::spn(PWR->CSR, {PWR_CSR_VOSRDY, 1});
//  while (LL_PWR_IsActiveFlag_VOS() == 0)
//  {
//  }
    sfr::set(RCC->CFGR, {
        RCC_CFGR_HPRE, 0,
        RCC_CFGR_PPRE1, 4,
        RCC_CFGR_PPRE2, 0,
        RCC_CFGR_SW, 2,
    });
//  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
//  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
//  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
//  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    sfr::spn(RCC->CFGR, {RCC_CFGR_SWS, 2});
//   /* Wait till System clock is ready */
//  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
//  {
//
//  }
    sfr::rst(SysTick->LOAD, {SysTick_LOAD_RELOAD_Msk, 84'000 - 1});
    sfr::rst(SysTick->VAL, {SysTick_VAL_CURRENT_Msk, 0});
    sfr::rst(SysTick->CTRL, {
        SysTick_CTRL_CLKSOURCE_Msk, 1,
        SysTick_CTRL_ENABLE_Msk, 1,
    });
//  LL_Init1msTick(84000000);
    SystemCoreClock = 84'000'000;
//  LL_SetSystemCoreClock(84000000);
    sfr::set(RCC->DCKCFGR, {RCC_DCKCFGR_TIMPRE, 0});
//  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    sfr::set(RCC->APB1ENR, {RCC_APB1ENR_USART2EN, 1});
    sfr::dip(RCC->APB1ENR);
//  /* Peripheral clock enable */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOAEN, 1});
    sfr::dip(RCC->AHB1ENR);
    //sfr::store_drop<RCC_AHB1ENR_GPIOAEN, 1>(RCC->AHB1ENR);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    sfr::set(GPIOA->OSPEEDR, {
        GPIO_OSPEEDR_OSPEED2, 3,
        GPIO_OSPEEDR_OSPEED3, 3,
    });
    sfr::set(GPIOA->OTYPER, {
        GPIO_OTYPER_OT2, 0,
        GPIO_OTYPER_OT3, 0,
    });
    sfr::set(GPIOA->PUPDR, {
        GPIO_PUPDR_PUPD2, 0,
        GPIO_PUPDR_PUPD3, 0,
    });
    sfr::set(GPIOA->AFR[0], {
        GPIO_AFRL_AFSEL2, 7,
        GPIO_AFRL_AFSEL3, 7,
    });
    sfr::set(GPIOA->MODER, {
        GPIO_MODER_MODER2, 2,
        GPIO_MODER_MODER3, 2,
    });
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
    if (!sfr::val(USART2->CR1, {USART_CR1_UE})) {
        sfr::set(USART2->CR1, {
            USART_CR1_M, 0,
            USART_CR1_PCE, 0,
            USART_CR1_PS, 0,
            USART_CR1_TE, 1,
            USART_CR1_RE, 1,
            USART_CR1_OVER8, 0,
        });
        sfr::set(USART2->CR2, {USART_CR2_STOP, 0});
        sfr::set(USART2->CR3, {
            USART_CR3_RTSE, 0,
            USART_CR3_CTSE, 0,
        });

        uint32_t sclk_freq;
        switch (sfr::val(RCC->CFGR, {RCC_CFGR_SWS})) {
            case 0:
                sclk_freq = HSI_VALUE;
                break;
            case 1:
                sclk_freq = HSE_VALUE;
                break;
            case 2: {
                auto input_freq = RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC ? HSE_VALUE : HSI_VALUE;
                auto [pllm, plln, pllp] = sfr::val(RCC->PLLCFGR, {
                    RCC_PLLCFGR_PLLM,
                    RCC_PLLCFGR_PLLN,
                    RCC_PLLCFGR_PLLP,
                });
                sclk_freq = (input_freq / pllm * plln) / ((pllp + 1) * 2);
                break;
            }
            default:
                sclk_freq = HSI_VALUE;
        }
        auto [hpre, ppre] = sfr::val(RCC->CFGR, {
            RCC_CFGR_HPRE,
            RCC_CFGR_PPRE1,
        });
        static constexpr uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
        static constexpr uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
        uint32_t hclk_freq = sclk_freq >> AHBPrescTable[hpre];
        uint32_t pclk_freq = hclk_freq >> APBPrescTable[ppre];
        auto div = ((uint32_t)((((uint32_t)(pclk_freq))*25)/(4*((uint32_t)(115200)))));
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
    sfr::set(USART2->CR2, {
        USART_CR2_LINEN, 0,
        USART_CR2_CLKEN, 0,
    });
//  LL_USART_ConfigAsyncMode(USART2);
    sfr::set(USART2->CR1, {USART_CR1_UE, 1});
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
    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOCEN, 1});
    sfr::dip(RCC->AHB1ENR);
    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOHEN, 1});
    sfr::dip(RCC->AHB1ENR);
    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOAEN, 1});
    sfr::dip(RCC->AHB1ENR);
    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOBEN, 1});
    sfr::dip(RCC->AHB1ENR);
//    sfr::store_drop<
//        RCC_AHB1ENR_GPIOCEN, 1,
//        RCC_AHB1ENR_GPIOHEN, 1,
//        RCC_AHB1ENR_GPIOAEN, 1,
//        RCC_AHB1ENR_GPIOBEN, 1
//    >(RCC->AHB1ENR);
//  /* GPIO Ports Clock Enable */
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    sfr::set(GPIOA->BSRR, {GPIO_BSRR_BR5, 1});
//  /**/
//  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

    sfr::set(SYSCFG->EXTICR[3], {SYSCFG_EXTICR4_EXTI13, 2});
//  /**/
//  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

    sfr::set(EXTI->EMR, {EXTI_EMR_MR13, 0});
    sfr::set(EXTI->IMR, {EXTI_IMR_MR13, 1});
    sfr::set(EXTI->RTSR, {EXTI_RTSR_TR13, 0});
    sfr::set(EXTI->FTSR, {EXTI_FTSR_TR13, 1});
//  /**/
//  LL_EXTI_InitTypeDef EXTI_InitStruct = {};
//  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
//  EXTI_InitStruct.LineCommand = ENABLE;
//  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
//  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
//  LL_EXTI_Init(&EXTI_InitStruct);

    sfr::set(GPIOC->PUPDR, {GPIO_PUPDR_PUPD13, 0});
//  /**/
//  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

    sfr::set(GPIOC->MODER, {GPIO_MODER_MODER13, 0});
//  /**/
//  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

    sfr::set(GPIOA->OSPEEDR, {GPIO_OSPEEDR_OSPEED5, 0});
    sfr::set(GPIOA->OTYPER, {GPIO_OTYPER_OT5, 0});
    sfr::set(GPIOA->PUPDR, {GPIO_PUPDR_PUPD5, 0});
    sfr::set(GPIOA->MODER, {GPIO_MODER_MODER5, 1});
//  /**/
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {};
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}
