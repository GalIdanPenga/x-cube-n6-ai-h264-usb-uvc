# STM32N6 Framework Project - Tech Lead Strategy

## Executive Summary

This document outlines the complete strategy for building a reusable STM32N6 + FreeRTOS firmware framework, based on ST's `x-cube-n6-ai-h264-usb-uvc` reference project. The goal is to create a layered architecture (HAL → FAL → SVC → APP) that can be reused across multiple products.

**Timeline Estimate**: 16-20 weeks for full framework  
**Team Size**: 3-5 firmware engineers  
**Base Project**: [x-cube-n6-ai-h264-usb-uvc](https://github.com/STMicroelectronics/x-cube-n6-ai-h264-usb-uvc)

---

## 1. Repository Strategy

### 1.1 Repository Structure

We'll use a **multi-repo approach** with Git submodules for maximum reusability:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         REPOSITORY ARCHITECTURE                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    Product Repositories                              │    │
│  │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐                 │    │
│  │  │ product-foo  │ │ product-bar  │ │ product-baz  │  (future)       │    │
│  │  └──────┬───────┘ └──────┬───────┘ └──────┬───────┘                 │    │
│  └─────────┼────────────────┼────────────────┼─────────────────────────┘    │
│            │                │                │                               │
│            └────────────────┼────────────────┘                               │
│                             ▼                                                │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    n6-app-template                                   │    │
│  │         Template repository for new products                         │    │
│  │         (includes submodules for FAL, SVC, BSP)                      │    │
│  └──────────────────────────────┬──────────────────────────────────────┘    │
│                                 │                                            │
│            ┌────────────────────┼────────────────────┐                       │
│            ▼                    ▼                    ▼                       │
│  ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐             │
│  │     n6-fal       │ │     n6-svc       │ │     n6-bsp       │             │
│  │  (FAL drivers)   │ │   (Services)     │ │ (Board Support)  │             │
│  └──────────────────┘ └──────────────────┘ └──────────────────┘             │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    n6-tools (optional)                               │    │
│  │         Build scripts, CI/CD, testing tools, linters                 │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Repository Definitions

| Repository | Description | Visibility | Branching |
|------------|-------------|------------|-----------|
| **n6-fal** | Firmware Abstraction Layer - thread-safe peripheral drivers | Private | main + develop + feature/* |
| **n6-svc** | Services Layer - reusable software components | Private | main + develop + feature/* |
| **n6-bsp** | Board Support Packages for different boards | Private | main + develop + feature/* |
| **n6-app-template** | Template for new product applications | Private | main only |
| **n6-tools** | Build tools, CI scripts, code generators | Private | main + develop |
| **product-xxx** | Actual product repositories (use template) | Private | main + develop + release/* |

### 1.3 Repository Details

#### n6-fal (Firmware Abstraction Layer)
```
n6-fal/
├── README.md
├── LICENSE
├── CHANGELOG.md
├── CMakeLists.txt              # CMake support
├── fal.mk                      # Makefile include
├── Inc/
│   ├── fal.h                   # Main include (includes all)
│   ├── fal_common.h            # Types, return codes
│   ├── fal_config_template.h   # Config template (copy to project)
│   ├── fal_gpio.h
│   ├── fal_uart.h
│   ├── fal_spi.h
│   ├── fal_i2c.h
│   ├── fal_i3c.h
│   ├── fal_adc.h
│   ├── fal_tim.h
│   ├── fal_rtc.h
│   ├── fal_dma.h
│   ├── fal_flash.h             # External flash
│   ├── fal_ram.h               # External RAM
│   ├── fal_eth.h
│   ├── fal_usb.h
│   ├── fal_csi.h               # MIPI CSI-2
│   ├── fal_isp.h
│   ├── fal_npu.h               # Neural-ART
│   ├── fal_h264.h
│   └── ...
├── Src/
│   └── [corresponding .c files]
├── Tests/
│   ├── Unity/                  # Unity test framework
│   └── test_fal_*.c
└── Docs/
    └── API.md
```

#### n6-svc (Services Layer)
```
n6-svc/
├── README.md
├── CMakeLists.txt
├── svc.mk
├── Inc/
│   ├── svc.h
│   ├── svc_common.h
│   │
│   ├── os/                     # OS Abstraction
│   │   ├── svc_os.h
│   │   ├── svc_task.h
│   │   ├── svc_mutex.h
│   │   ├── svc_queue.h
│   │   ├── svc_timer.h
│   │   └── svc_event.h
│   │
│   ├── data/                   # Data Structures
│   │   ├── svc_ringbuf.h
│   │   ├── svc_pool.h
│   │   ├── svc_list.h
│   │   └── svc_fifo.h
│   │
│   ├── protocol/               # Protocol Engines
│   │   ├── svc_msg_parser.h
│   │   ├── svc_msg_builder.h
│   │   ├── svc_msg_router.h
│   │   ├── svc_cobs.h
│   │   └── svc_protocol.h
│   │
│   ├── storage/                # Storage Abstractions
│   │   ├── svc_nvs.h
│   │   ├── svc_kvstore.h
│   │   └── svc_filesystem.h
│   │
│   ├── util/                   # Utilities
│   │   ├── svc_crc.h
│   │   ├── svc_log.h
│   │   ├── svc_cli.h
│   │   ├── svc_assert.h
│   │   └── svc_debug.h
│   │
│   ├── time/                   # Time & Scheduling
│   │   ├── svc_time.h
│   │   ├── svc_calendar.h
│   │   ├── svc_scheduler.h
│   │   └── svc_timeout.h
│   │
│   ├── fsm/                    # State Machines
│   │   ├── svc_fsm.h
│   │   └── svc_hsm.h
│   │
│   └── dsp/                    # Math & DSP
│       ├── svc_filter.h
│       ├── svc_pid.h
│       └── svc_stats.h
├── Src/
│   └── [organized by subfolder]
└── Tests/
    └── test_svc_*.c
```

#### n6-bsp (Board Support)
```
n6-bsp/
├── README.md
├── CMakeLists.txt
├── STM32N6570-DK/              # Discovery Kit
│   ├── Inc/
│   │   ├── bsp_config.h
│   │   ├── bsp_led.h
│   │   ├── bsp_button.h
│   │   ├── bsp_camera.h
│   │   └── bsp_display.h
│   ├── Src/
│   ├── STM32CubeMX/            # CubeMX project
│   └── LinkerScripts/
│       ├── stm32n657xx_ram.ld
│       └── stm32n657xx_flash.ld
│
├── NUCLEO-N657X0-Q/            # Nucleo Board
│   ├── Inc/
│   ├── Src/
│   ├── STM32CubeMX/
│   └── LinkerScripts/
│
└── Custom-Board-Template/       # Template for custom boards
    └── ...
```

#### n6-app-template (Application Template)
```
n6-app-template/
├── README.md
├── .gitmodules                 # Submodule definitions
├── CMakeLists.txt
├── Makefile
├── .clang-format
├── .clang-tidy
│
├── Lib/                        # Submodules
│   ├── n6-fal/                 # → git submodule
│   ├── n6-svc/                 # → git submodule
│   └── n6-bsp/                 # → git submodule
│
├── Drivers/                    # ST Drivers (from CubeMX)
│   ├── STM32N6xx_HAL_Driver/
│   └── CMSIS/
│
├── Middlewares/
│   ├── FreeRTOS/
│   └── USBX/                   # If needed
│
├── App/                        # Application code
│   ├── Inc/
│   │   ├── app_config.h
│   │   ├── app_tasks.h
│   │   └── FreeRTOSConfig.h
│   └── Src/
│       ├── main.c
│       ├── app_freertos.c
│       └── stm32n6xx_it.c
│
├── Config/
│   ├── fal_config.h            # FAL configuration
│   ├── svc_config.h            # SVC configuration
│   └── board_config.h          # Board selection
│
├── Scripts/
│   ├── flash.sh
│   ├── debug.sh
│   └── sign.sh
│
└── Doc/
    └── README.md
```

---

## 2. Jira Project Structure

### 2.1 Epic Hierarchy

```
PROJECT: N6FW (N6 Firmware Framework)
│
├── EPIC: N6FW-INFRA     "Infrastructure & Setup"
├── EPIC: N6FW-FAL       "FAL - Firmware Abstraction Layer"
├── EPIC: N6FW-SVC       "SVC - Services Layer"
├── EPIC: N6FW-BSP       "BSP - Board Support Package"
├── EPIC: N6FW-TEMPLATE  "Application Template"
├── EPIC: N6FW-DOC       "Documentation"
├── EPIC: N6FW-TEST      "Testing & Validation"
└── EPIC: N6FW-CI        "CI/CD Pipeline"
```

### 2.2 Complete Jira Issue List

---

#### EPIC: N6FW-INFRA - Infrastructure & Setup

| Key | Type | Summary | Priority | Sprint | Story Points |
|-----|------|---------|----------|--------|--------------|
| N6FW-1 | Story | Create Git repositories (n6-fal, n6-svc, n6-bsp, n6-app-template) | Critical | 1 | 2 |
| N6FW-2 | Story | Set up branch protection rules and PR templates | High | 1 | 1 |
| N6FW-3 | Story | Create .clang-format and .clang-tidy configurations | High | 1 | 2 |
| N6FW-4 | Story | Set up STM32CubeIDE project structure | Critical | 1 | 3 |
| N6FW-5 | Story | Configure Makefile build system | High | 1 | 3 |
| N6FW-6 | Story | Configure CMake build system (alternative) | Medium | 2 | 3 |
| N6FW-7 | Story | Clone and analyze x-cube-n6-ai-h264-usb-uvc reference | Critical | 1 | 2 |
| N6FW-8 | Task | Document base project structure and dependencies | High | 1 | 2 |
| N6FW-9 | Task | Identify reusable components from reference project | High | 1 | 3 |
| N6FW-10 | Story | Set up linker scripts for RAM and Flash execution | Critical | 1 | 3 |
| N6FW-11 | Story | Configure FreeRTOS with STM32N6 optimizations | Critical | 1 | 5 |
| N6FW-12 | Task | Define memory map (4.2MB SRAM partitioning) | High | 1 | 3 |
| N6FW-13 | Task | Create FreeRTOSConfig.h template | High | 1 | 2 |

---

#### EPIC: N6FW-FAL - Firmware Abstraction Layer (Core Drivers)

**Sub-Epic: FAL-CORE - Core System**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-20 | Story | [FAL] Create fal_common.h - types, return codes, macros | Critical | 2 | 2 |
| N6FW-21 | Story | [FAL] Create fal_config.h template | Critical | 2 | 1 |
| N6FW-22 | Story | [FAL] Implement fal_gpio - GPIO driver with atomic ops | Critical | 2 | 3 |
| N6FW-23 | Story | [FAL] Implement fal_exti - External interrupt driver | High | 2 | 3 |
| N6FW-24 | Story | [FAL] Implement fal_dma - DMA channel manager | Critical | 2 | 5 |
| N6FW-25 | Story | [FAL] Implement fal_iwdg - Watchdog driver | High | 3 | 2 |
| N6FW-26 | Story | [FAL] Implement fal_pwr - Power/sleep mode driver | High | 4 | 5 |
| N6FW-27 | Story | [FAL] Implement fal_clk - Clock switching driver | High | 4 | 3 |
| N6FW-28 | Story | [FAL] Implement fal_cache - L1 cache management | Critical | 2 | 3 |

**Sub-Epic: FAL-COMM - Communication Peripherals**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-30 | Story | [FAL] Implement fal_uart - UART with DMA & stream buffers | Critical | 2 | 8 |
| N6FW-31 | Story | [FAL] Implement fal_spi - SPI master with mutex | Critical | 2 | 5 |
| N6FW-32 | Story | [FAL] Implement fal_spi_slave - SPI slave driver | Medium | 4 | 5 |
| N6FW-33 | Story | [FAL] Implement fal_i2c - I2C master with mutex | Critical | 3 | 5 |
| N6FW-34 | Story | [FAL] Implement fal_i3c - I3C driver (new for N6) | Medium | 5 | 8 |
| N6FW-35 | Story | [FAL] Implement fal_fdcan - CAN FD driver | Medium | 5 | 5 |
| N6FW-36 | Story | [FAL] Implement fal_eth - Gigabit Ethernet + TSN | High | 6 | 13 |
| N6FW-37 | Story | [FAL] Implement fal_usb_dev - USB device driver | High | 5 | 8 |
| N6FW-38 | Story | [FAL] Implement fal_sdmmc - SD/eMMC driver | Medium | 6 | 5 |

**Sub-Epic: FAL-TIMING - Timing & Clocks**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-40 | Story | [FAL] Implement fal_tim - General timer driver | Critical | 3 | 5 |
| N6FW-41 | Story | [FAL] Implement fal_tim_adv - Advanced timer (PWM) | High | 3 | 5 |
| N6FW-42 | Story | [FAL] Implement fal_lptim - Low-power timer | Medium | 4 | 3 |
| N6FW-43 | Story | [FAL] Implement fal_rtc - RTC with alarms | High | 3 | 5 |
| N6FW-44 | Story | [FAL] Implement fal_pwm_out - PWM output driver | High | 3 | 3 |
| N6FW-45 | Story | [FAL] Implement fal_pwm_cap - Input capture driver | Medium | 4 | 3 |

**Sub-Epic: FAL-ANALOG - Analog Peripherals**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-50 | Story | [FAL] Implement fal_adc - ADC with DMA | High | 3 | 5 |
| N6FW-51 | Story | [FAL] Implement fal_dac - DAC driver | Low | 5 | 3 |
| N6FW-52 | Story | [FAL] Implement fal_comp - Comparator driver | Low | 6 | 2 |
| N6FW-53 | Story | [FAL] Implement fal_temp - Internal temp sensor | Low | 4 | 2 |

**Sub-Epic: FAL-MEM - Memory Drivers (Critical for N6)**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-60 | Story | [FAL] Implement fal_xspi - HexaSPI/OctoSPI base driver | Critical | 2 | 8 |
| N6FW-61 | Story | [FAL] Implement fal_flash - External NOR flash driver | Critical | 2 | 8 |
| N6FW-62 | Story | [FAL] Implement fal_flash_nand - External NAND driver | Medium | 5 | 8 |
| N6FW-63 | Story | [FAL] Implement fal_ram - External PSRAM/SDRAM driver | Critical | 3 | 8 |
| N6FW-64 | Story | [FAL] Implement fal_fmc - Flexible Memory Controller | Medium | 5 | 5 |

**Sub-Epic: FAL-SEC - Security & Crypto**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-70 | Story | [FAL] Implement fal_rng - True RNG driver | High | 4 | 2 |
| N6FW-71 | Story | [FAL] Implement fal_cryp - Crypto accelerator | Medium | 6 | 5 |
| N6FW-72 | Story | [FAL] Implement fal_hash - Hash accelerator | Medium | 6 | 3 |
| N6FW-73 | Story | [FAL] Implement fal_pka - Public key accelerator | Low | 7 | 5 |

**Sub-Epic: FAL-MEDIA - Media & Graphics (N6 Specific)**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-80 | Story | [FAL] Implement fal_dcmipp - Camera interface | High | 4 | 8 |
| N6FW-81 | Story | [FAL] Implement fal_csi - MIPI CSI-2 driver | High | 4 | 8 |
| N6FW-82 | Story | [FAL] Implement fal_isp - Image Signal Processor | High | 5 | 8 |
| N6FW-83 | Story | [FAL] Implement fal_ltdc - LCD controller | Medium | 6 | 5 |
| N6FW-84 | Story | [FAL] Implement fal_dma2d - Chrom-ART 2D | Medium | 6 | 5 |
| N6FW-85 | Story | [FAL] Implement fal_neochrom - 2.5D GPU | Low | 7 | 8 |
| N6FW-86 | Story | [FAL] Implement fal_jpeg - JPEG codec | Medium | 6 | 5 |
| N6FW-87 | Story | [FAL] Implement fal_h264 - H.264 encoder | Medium | 6 | 8 |

**Sub-Epic: FAL-AI - AI/NPU (N6 Specific)**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-90 | Story | [FAL] Implement fal_npu - Neural-ART accelerator | High | 5 | 13 |
| N6FW-91 | Story | [FAL] Integrate STEdgeAI runtime | High | 5 | 8 |
| N6FW-92 | Story | [FAL] Create NPU job queue and async API | High | 6 | 5 |

---

#### EPIC: N6FW-SVC - Services Layer

**Sub-Epic: SVC-OS - OS Abstraction**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-100 | Story | [SVC] Implement svc_os - FreeRTOS wrapper API | Critical | 2 | 5 |
| N6FW-101 | Story | [SVC] Implement svc_task - Task creation helpers | Critical | 2 | 3 |
| N6FW-102 | Story | [SVC] Implement svc_mutex - Mutex utilities | Critical | 2 | 2 |
| N6FW-103 | Story | [SVC] Implement svc_queue - Queue utilities | Critical | 2 | 3 |
| N6FW-104 | Story | [SVC] Implement svc_timer - Software timer wrapper | High | 3 | 3 |
| N6FW-105 | Story | [SVC] Implement svc_event - Event group wrapper | High | 3 | 2 |

**Sub-Epic: SVC-DATA - Data Structures**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-110 | Story | [SVC] Implement svc_ringbuf - Thread-safe ring buffer | Critical | 2 | 3 |
| N6FW-111 | Story | [SVC] Implement svc_pool - Fixed-size memory pool | High | 3 | 5 |
| N6FW-112 | Story | [SVC] Implement svc_fifo - FIFO queue | High | 3 | 2 |
| N6FW-113 | Story | [SVC] Implement svc_list - Linked list utilities | Medium | 4 | 3 |

**Sub-Epic: SVC-PROTO - Protocol Engines**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-120 | Story | [SVC] Implement svc_msg_parser - Message framing/parsing | Critical | 3 | 5 |
| N6FW-121 | Story | [SVC] Implement svc_msg_builder - Message construction | Critical | 3 | 3 |
| N6FW-122 | Story | [SVC] Implement svc_msg_router - Message routing engine | Critical | 4 | 5 |
| N6FW-123 | Story | [SVC] Implement svc_cobs - COBS encoding | Medium | 4 | 2 |
| N6FW-124 | Story | [SVC] Implement svc_protocol - Protocol state machine | High | 4 | 5 |

**Sub-Epic: SVC-STORE - Storage Services**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-130 | Story | [SVC] Implement svc_nvs - NV storage abstraction | High | 4 | 5 |
| N6FW-131 | Story | [SVC] Implement svc_kvstore - Key-value store | High | 4 | 5 |
| N6FW-132 | Story | [SVC] Integrate LittleFS file system | Medium | 5 | 5 |
| N6FW-133 | Story | [SVC] Implement svc_datalog - Data logging framework | Medium | 5 | 5 |

**Sub-Epic: SVC-UTIL - Utilities**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-140 | Story | [SVC] Implement svc_crc - CRC8/16/32 | Critical | 2 | 2 |
| N6FW-141 | Story | [SVC] Implement svc_log - Logging framework | Critical | 2 | 5 |
| N6FW-142 | Story | [SVC] Implement svc_assert - Assert framework | Critical | 2 | 2 |
| N6FW-143 | Story | [SVC] Implement svc_debug - Debug utilities | High | 3 | 3 |
| N6FW-144 | Story | [SVC] Implement svc_cli - CLI engine | Medium | 5 | 8 |
| N6FW-145 | Story | [SVC] Implement svc_base64 - Base64 codec | Low | 5 | 1 |
| N6FW-146 | Story | [SVC] Implement svc_json - Lightweight JSON parser | Low | 6 | 5 |

**Sub-Epic: SVC-TIME - Time & Scheduling**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-150 | Story | [SVC] Implement svc_time - Time utilities | High | 3 | 3 |
| N6FW-151 | Story | [SVC] Implement svc_calendar - Calendar calculations | High | 4 | 3 |
| N6FW-152 | Story | [SVC] Implement svc_scheduler - Periodic job scheduler | High | 4 | 5 |
| N6FW-153 | Story | [SVC] Implement svc_timeout - Timeout manager | High | 3 | 3 |

**Sub-Epic: SVC-FSM - State Machines**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-160 | Story | [SVC] Implement svc_fsm - Finite state machine framework | High | 4 | 5 |
| N6FW-161 | Story | [SVC] Implement svc_hsm - Hierarchical state machine | Medium | 5 | 8 |

**Sub-Epic: SVC-DSP - Math & DSP**
| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-170 | Story | [SVC] Implement svc_filter - Digital filters | Medium | 5 | 5 |
| N6FW-171 | Story | [SVC] Implement svc_pid - PID controller | Medium | 5 | 3 |
| N6FW-172 | Story | [SVC] Implement svc_stats - Statistics calculations | Low | 6 | 3 |
| N6FW-173 | Story | [SVC] Implement svc_interp - Interpolation utilities | Low | 6 | 2 |

---

#### EPIC: N6FW-BSP - Board Support Package

| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-200 | Story | [BSP] Create STM32N6570-DK board support | Critical | 2 | 5 |
| N6FW-201 | Story | [BSP] Create NUCLEO-N657X0-Q board support | High | 3 | 5 |
| N6FW-202 | Story | [BSP] Implement bsp_led for all boards | High | 2 | 2 |
| N6FW-203 | Story | [BSP] Implement bsp_button for all boards | High | 2 | 2 |
| N6FW-204 | Story | [BSP] Implement bsp_camera - Camera module support | High | 4 | 5 |
| N6FW-205 | Story | [BSP] Create custom board template | Medium | 5 | 3 |
| N6FW-206 | Story | [BSP] Create CubeMX base project for DK | Critical | 1 | 3 |
| N6FW-207 | Story | [BSP] Create CubeMX base project for Nucleo | High | 2 | 3 |
| N6FW-208 | Story | [BSP] Create optimized linker scripts | Critical | 2 | 5 |

---

#### EPIC: N6FW-TEMPLATE - Application Template

| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-220 | Story | Create n6-app-template repository structure | High | 4 | 3 |
| N6FW-221 | Story | Configure git submodules for FAL/SVC/BSP | High | 4 | 2 |
| N6FW-222 | Story | Create template main.c with task initialization | High | 4 | 3 |
| N6FW-223 | Story | Create template FreeRTOSConfig.h | High | 4 | 2 |
| N6FW-224 | Story | Create build scripts (make, cmake) | High | 4 | 3 |
| N6FW-225 | Story | Create flash/debug scripts | High | 4 | 2 |
| N6FW-226 | Story | Create "Hello World" example with LED + UART | High | 4 | 3 |
| N6FW-227 | Story | Create template README with getting started guide | High | 5 | 2 |

---

#### EPIC: N6FW-DOC - Documentation

| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-240 | Story | Write architecture overview document | High | 3 | 3 |
| N6FW-241 | Story | Write coding standards document | High | 2 | 3 |
| N6FW-242 | Story | Write FAL API documentation | High | 5 | 5 |
| N6FW-243 | Story | Write SVC API documentation | High | 6 | 5 |
| N6FW-244 | Story | Write "Getting Started" guide | High | 5 | 3 |
| N6FW-245 | Story | Write "Porting Guide" for new boards | Medium | 7 | 3 |
| N6FW-246 | Story | Create Doxygen configuration | Medium | 4 | 2 |
| N6FW-247 | Story | Write FreeRTOS best practices guide | High | 3 | 3 |

---

#### EPIC: N6FW-TEST - Testing & Validation

| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-260 | Story | Set up Unity test framework | High | 2 | 2 |
| N6FW-261 | Story | Write unit tests for svc_ringbuf | High | 3 | 2 |
| N6FW-262 | Story | Write unit tests for svc_crc | High | 3 | 1 |
| N6FW-263 | Story | Write unit tests for svc_fsm | High | 5 | 3 |
| N6FW-264 | Story | Write unit tests for svc_msg_parser | High | 4 | 3 |
| N6FW-265 | Story | Create hardware-in-loop test for fal_uart | Medium | 5 | 5 |
| N6FW-266 | Story | Create hardware-in-loop test for fal_spi | Medium | 5 | 5 |
| N6FW-267 | Story | Create integration test suite | Medium | 6 | 8 |
| N6FW-268 | Story | Create stress test for FreeRTOS configuration | High | 5 | 5 |

---

#### EPIC: N6FW-CI - CI/CD Pipeline

| Key | Type | Summary | Priority | Sprint | SP |
|-----|------|---------|----------|--------|-----|
| N6FW-280 | Story | Set up GitHub Actions / GitLab CI | High | 3 | 3 |
| N6FW-281 | Story | Configure build pipeline for all targets | High | 3 | 3 |
| N6FW-282 | Story | Configure clang-format check in CI | High | 3 | 1 |
| N6FW-283 | Story | Configure clang-tidy static analysis | High | 3 | 2 |
| N6FW-284 | Story | Configure unit test execution in CI | High | 4 | 2 |
| N6FW-285 | Story | Set up artifact publishing (hex files) | Medium | 5 | 2 |
| N6FW-286 | Story | Configure automatic version tagging | Medium | 5 | 2 |
| N6FW-287 | Story | Set up code coverage reporting | Low | 6 | 3 |

---

## 3. Sprint Planning

### Sprint Overview (2-week sprints)

| Sprint | Focus | Key Deliverables |
|--------|-------|------------------|
| **Sprint 1** | Foundation | Repos, CubeMX, FreeRTOS config, linker scripts, base project analysis |
| **Sprint 2** | Core FAL + SVC | GPIO, UART, SPI, DMA, external flash, OS wrappers, ringbuf, CRC, logging |
| **Sprint 3** | Timers + Comms | Timers, RTC, ADC, I2C, message parser, time utilities |
| **Sprint 4** | Storage + Protocol | NVS, KV store, FSM, scheduler, message router, camera basics |
| **Sprint 5** | Advanced FAL | I3C, NPU integration, CLI, more tests |
| **Sprint 6** | Media + Graphics | ISP, H264, JPEG, Ethernet, integration tests |
| **Sprint 7** | Polish + Docs | Documentation, porting guide, optimization |
| **Sprint 8** | Template + Examples | App template finalization, example applications |

### Sprint 1 Detail (Weeks 1-2)

**Goal**: Get a minimal FreeRTOS application running on STM32N6570-DK

| Issue | Assignee | Days |
|-------|----------|------|
| N6FW-1: Create Git repositories | Lead | 0.5 |
| N6FW-7: Clone and analyze reference project | All | 2 |
| N6FW-8: Document base project structure | Dev1 | 1 |
| N6FW-9: Identify reusable components | Dev2 | 1 |
| N6FW-206: Create CubeMX base project | Dev1 | 2 |
| N6FW-10: Linker scripts | Dev2 | 2 |
| N6FW-11: FreeRTOS configuration | Dev1 | 3 |
| N6FW-12: Memory map definition | Lead | 1 |
| N6FW-3: Coding standards (.clang-format) | Lead | 1 |
| N6FW-5: Makefile build system | Dev2 | 2 |

**Exit Criteria**:
- [ ] Blinky LED running on FreeRTOS
- [ ] UART console working
- [ ] Build system functional
- [ ] Memory map documented

### Sprint 2 Detail (Weeks 3-4)

**Goal**: Core drivers operational, basic SVC layer

| Issue | Assignee | Days |
|-------|----------|------|
| N6FW-20: fal_common.h | Lead | 1 |
| N6FW-22: fal_gpio | Dev1 | 2 |
| N6FW-24: fal_dma | Dev2 | 3 |
| N6FW-30: fal_uart | Dev1 | 4 |
| N6FW-31: fal_spi | Dev2 | 3 |
| N6FW-60: fal_xspi | Dev3 | 4 |
| N6FW-61: fal_flash | Dev3 | 4 |
| N6FW-100: svc_os | Lead | 3 |
| N6FW-110: svc_ringbuf | Lead | 2 |
| N6FW-140: svc_crc | Dev1 | 1 |
| N6FW-141: svc_log | Dev2 | 3 |
| N6FW-260: Unity test setup | Lead | 1 |
| N6FW-280: CI pipeline | Lead | 2 |

**Exit Criteria**:
- [ ] UART TX/RX with DMA working
- [ ] SPI communication verified
- [ ] External flash read/write working
- [ ] Logging system operational
- [ ] CI building successfully

---

## 4. Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| STM32N6 HAL/BSP instability (new chip) | High | High | Start with ST reference code, minimize custom HAL usage |
| External flash XIP complexity | Medium | High | Allocate extra time, use ST examples |
| FreeRTOS + Azure RTOS USBX conflict | Medium | Medium | Keep USB separate, consider alternatives |
| Cache coherency issues with DMA | High | Medium | Strict memory barriers, dedicated DMA regions |
| NPU integration complexity | Medium | High | Start with ST's AI runtime, wrap don't rewrite |
| Thread-safety bugs | High | Medium | Code reviews, stress testing, static analysis |
| Scope creep | Medium | Medium | Strict sprint goals, defer non-essentials |

---

## 5. Team Responsibilities

| Role | Name | Focus Areas |
|------|------|-------------|
| **Tech Lead** | TBD | Architecture, code reviews, blocking issues, CI/CD |
| **Dev 1** | TBD | FAL communication drivers (UART, SPI, I2C) |
| **Dev 2** | TBD | FAL timing, analog, DMA, external memory |
| **Dev 3** | TBD | FAL media (camera, NPU, H264), BSP |
| **Dev 4** | TBD | SVC layer, protocol engines, testing |

---

## 6. Definition of Done

### For Stories (Drivers/Modules)
- [ ] Code compiles without warnings (-Wall -Wextra)
- [ ] Passes clang-tidy static analysis
- [ ] Follows coding standards (.clang-format)
- [ ] Unit tests written and passing (where applicable)
- [ ] Hardware tested on STM32N6570-DK
- [ ] API documented (Doxygen comments)
- [ ] Code reviewed by at least 1 team member
- [ ] No known race conditions or thread-safety issues
- [ ] Merged to develop branch

### For Epics
- [ ] All stories complete
- [ ] Integration tested
- [ ] Documentation updated
- [ ] Example code created

---

## 7. Key Decisions Needed

| Decision | Options | Recommendation | Owner | Due |
|----------|---------|----------------|-------|-----|
| USB stack | Azure RTOS USBX vs TinyUSB | Keep USBX (already in reference) | Lead | Sprint 1 |
| File system | LittleFS vs FatFS | LittleFS (better for flash) | Lead | Sprint 4 |
| CLI framework | Custom vs microRL | Custom (simpler, smaller) | Dev4 | Sprint 5 |
| JSON parser | cJSON vs custom | Custom lightweight | Dev4 | Sprint 6 |
| Build system primary | Makefile vs CMake | Both (Makefile primary) | Lead | Sprint 1 |
| Git strategy | Submodules vs monorepo | Submodules (reusability) | Lead | Sprint 1 |

---

## 8. Immediate Action Items

### Week 1 Checklist

- [ ] **Day 1**: Create all Git repositories
- [ ] **Day 1**: Clone x-cube-n6-ai-h264-usb-uvc reference
- [ ] **Day 1-2**: All developers study reference project
- [ ] **Day 2**: Create Confluence/Wiki space for documentation
- [ ] **Day 2**: Set up Jira project and import issues
- [ ] **Day 3**: Kick-off meeting - review architecture
- [ ] **Day 3**: Assign Sprint 1 issues
- [ ] **Day 3-5**: CubeMX project setup
- [ ] **Day 4-5**: Get basic FreeRTOS running
- [ ] **Day 5**: Sprint 1 planning refinement

### Prerequisites

- [ ] STM32N6570-DK boards available (1 per developer)
- [ ] STM32CubeIDE v1.17+ installed
- [ ] STM32CubeProgrammer v2.18+ installed
- [ ] STEdgeAI v2.2+ installed (for NPU)
- [ ] Git access configured
- [ ] Jira project created

---

## 9. Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| FAL driver coverage | 100% of planned | Count of completed drivers |
| SVC module coverage | 100% of planned | Count of completed modules |
| Unit test coverage | >70% | Code coverage tool |
| Build time | <60 seconds | CI metrics |
| Flash footprint | <512KB (core framework) | Binary size |
| RAM footprint | <64KB (core framework) | Linker map |
| Time to new product | <2 weeks | From template to working build |
| Bug escape rate | <5 per sprint | Post-release bugs found |

---

## Appendix A: Jira Import CSV Format

```csv
Summary,Issue Type,Priority,Epic Link,Story Points,Sprint,Labels
"Create Git repositories (n6-fal, n6-svc, n6-bsp, n6-app-template)",Story,Critical,N6FW-INFRA,2,Sprint 1,infrastructure
"Set up branch protection rules and PR templates",Story,High,N6FW-INFRA,1,Sprint 1,infrastructure
...
```

---

## Appendix B: Reference Links

- [x-cube-n6-ai-h264-usb-uvc](https://github.com/STMicroelectronics/x-cube-n6-ai-h264-usb-uvc)
- [STM32N6 Product Page](https://www.st.com/en/microcontrollers-microprocessors/stm32n6-series.html)
- [STM32N6570-DK](https://www.st.com/en/evaluation-tools/stm32n6570-dk.html)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [STEdgeAI](https://www.st.com/en/development-tools/stedgeai-core.html)

---

*Document Version: 1.0*  
*Created: Tech Lead Strategy for STM32N6 Framework*
