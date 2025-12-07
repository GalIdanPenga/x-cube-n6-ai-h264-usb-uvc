# STM32N6 Framework Architecture
## Based on x-cube-n6-ai-h264-usb-uvc Reference Project

## Executive Summary

This document outlines a complete embedded firmware framework for STM32N6 microcontrollers, built upon ST's reference project `x-cube-n6-ai-h264-usb-uvc`. The architecture uses **ThreadX (Azure RTOS)** as ST's primary RTOS choice for STM32N6, with full integration of USBX, FileX, and the AI/Vision pipeline.

The framework introduces three custom layers: **FAL** (Firmware Abstraction Layer), **SVC** (Services Layer), and **APP** (Application Layer) on top of ST's BSP/HAL.

---

## 1. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              APP (Application)                               │
│   Product-specific: managers, handlers, state machines, business logic      │
│   Examples: led_mngr, wifi_mngr, msg_handler, sys_status, telem_mngr        │
├─────────────────────────────────────────────────────────────────────────────┤
│                              SVC (Services)                                  │
│   Reusable software: protocols, utilities, data structures, frameworks      │
│   Examples: svc_ringbuf, svc_msg_parser, svc_fsm, svc_log, svc_crc          │
├─────────────────────────────────────────────────────────────────────────────┤
│                        FAL (Firmware Abstraction)                            │
│   Thread-safe HW drivers: peripheral wrappers with RTOS primitives          │
│   Examples: fal_uart, fal_spi, fal_npu, fal_flash, fal_eth                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                    ST Middleware (from x-cube-n6 reference)                  │
│   ThreadX, USBX, FileX/LevelX, NetXDuo, H.264, ISP Library, AI Runtime      │
├─────────────────────────────────────────────────────────────────────────────┤
│                        HAL/LL (Hardware Abstraction)                         │
│   ST's HAL/LL drivers: CubeMX generated, low-level peripheral access        │
├─────────────────────────────────────────────────────────────────────────────┤
│                              Hardware                                        │
│              STM32N6 (Cortex-M55 + Neural-ART + NeoChrom)                   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Layer Dependency Rules

```
APP  →  can use  →  SVC, FAL (not HAL/Middleware directly)
SVC  →  can use  →  FAL, ThreadX APIs (not HAL directly)
FAL  →  can use  →  HAL/LL, ThreadX APIs, ST Middleware
```

### What Comes from Reference Project (Keep As-Is)

These components from `x-cube-n6-ai-h264-usb-uvc` should be preserved:
- `STM32Cube_FW_N6/` - ST's firmware package
- ThreadX kernel and configuration
- USBX device/host stacks
- H.264 encoder library
- ISP library
- AI runtime (Neural-ART)
- Boot loader (FSBL)
- Linker scripts and memory map
- Camera drivers (IMX335, VD66GY, VD55G1)

---

## 2. Key Design Principles for ThreadX

### 2.1 Thread-Safety Patterns (ThreadX)

| Pattern | ThreadX API | Use Case |
|---------|-------------|----------|
| **Mutex** | `TX_MUTEX` | Shared resource protection (peripherals, data structures) |
| **Semaphore** | `TX_SEMAPHORE` | Counting semaphore, resource pools |
| **Event Flags** | `TX_EVENT_FLAGS_GROUP` | Multiple event synchronization, state machines |
| **Queue** | `TX_QUEUE` | Inter-thread message passing (fixed-size) |
| **Block Pool** | `TX_BLOCK_POOL` | Fixed-size memory allocation |
| **Byte Pool** | `TX_BYTE_POOL` | Variable-size memory allocation |
| **Timer** | `TX_TIMER` | Periodic/one-shot software timers |

### 2.2 ISR Design Rules (ThreadX)

```c
// Pattern: Deferred interrupt processing with ThreadX
void PERIPHERAL_IRQHandler(void) {
    // Minimal ISR work: clear flag, grab data
    uint32_t data = READ_REG(PERIPHERAL->DR);
    CLEAR_FLAG(PERIPHERAL->SR, FLAG);
    
    // Signal thread via semaphore (ISR-safe)
    tx_semaphore_put(&peripheral_sem);
    
    // Or use event flags
    tx_event_flags_set(&peripheral_events, DATA_READY_FLAG, TX_OR);
}

// Thread waiting for interrupt
void peripheral_thread_entry(ULONG param) {
    while (1) {
        tx_semaphore_get(&peripheral_sem, TX_WAIT_FOREVER);
        // Process data
    }
}
```

### 2.3 Resource Ownership Model

- **Single Owner**: Each peripheral has ONE owning thread
- **Request/Response**: Other threads request service via queues
- **No Direct Access**: Threads never touch hardware directly (except owner)
- **Preemption Threshold**: Use for critical sections without full mutex overhead

---

## 3. Directory Structure (Based on Reference Project)

```
x-cube-n6-your-project/
│
├── Binary/                         # Pre-built binaries
│   ├── ai_fsbl.hex                # First Stage Boot Loader
│   ├── network_data.hex           # AI model weights
│   └── your_app.hex               # Application
│
├── Doc/                            # Documentation
│
├── STM32Cube_FW_N6/               # ST's firmware package (FROM REFERENCE)
│   ├── Drivers/
│   │   ├── STM32N6xx_HAL_Driver/  # HAL drivers
│   │   ├── CMSIS/
│   │   └── BSP/
│   └── Middlewares/
│       ├── ST/
│       │   ├── threadx/           # ThreadX RTOS
│       │   ├── usbx/              # USB stack
│       │   ├── filex/             # File system
│       │   ├── levelx/            # Flash wear leveling
│       │   ├── netxduo/           # Network stack
│       │   ├── STM32_AI_Runtime/  # NPU runtime
│       │   ├── STM32_ISP_Library/ # ISP algorithms
│       │   └── STM32_VENC_Library/# H.264 encoder
│       └── Third_Party/
│
├── FAL/                           # FIRMWARE ABSTRACTION LAYER (YOUR CODE)
│   ├── Inc/
│   │   ├── fal_common.h           # Common types, return codes
│   │   ├── fal_config.h           # Build-time config
│   │   ├── fal_uart.h
│   │   ├── fal_spi.h
│   │   ├── fal_i2c.h
│   │   ├── fal_gpio.h
│   │   ├── fal_adc.h
│   │   ├── fal_flash.h            # External flash
│   │   ├── fal_ram.h              # External RAM
│   │   ├── fal_npu.h              # Neural-ART wrapper
│   │   └── ...
│   └── Src/
│       └── [corresponding .c files]
│
├── SVC/                           # SERVICES LAYER (YOUR CODE)
│   ├── Inc/
│   │   ├── svc_common.h
│   │   ├── svc_os.h               # ThreadX wrappers (portable API)
│   │   ├── svc_ringbuf.h
│   │   ├── svc_msg_parser.h
│   │   ├── svc_msg_router.h
│   │   ├── svc_fsm.h
│   │   ├── svc_log.h
│   │   ├── svc_crc.h
│   │   ├── svc_config.h           # Configuration framework
│   │   ├── svc_nvs.h              # Non-volatile storage
│   │   └── ...
│   └── Src/
│       └── [corresponding .c files]
│
├── App/                           # APPLICATION LAYER (YOUR CODE)
│   ├── Inc/
│   │   ├── app_config.h           # Application configuration
│   │   ├── app_threads.h          # Thread declarations
│   │   ├── sys_status.h           # System state machine
│   │   ├── sys_flags.h            # Event flags
│   │   ├── led_mngr.h
│   │   ├── telem_mngr.h
│   │   ├── wifi_mngr.h
│   │   ├── gnss_mngr.h
│   │   ├── msg_handler.h
│   │   ├── storage_mngr.h
│   │   └── ...
│   └── Src/
│       ├── app_main.c             # tx_application_define()
│       ├── app_threads.c          # Thread creation
│       └── [corresponding .c files]
│
├── Core/                          # CubeMX generated (FROM REFERENCE)
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32n6xx_hal_conf.h
│   │   ├── stm32n6xx_it.h
│   │   └── tx_user.h              # ThreadX config
│   └── Src/
│       ├── main.c
│       ├── stm32n6xx_it.c
│       ├── stm32n6xx_hal_msp.c
│       └── system_stm32n6xx.c
│
├── Lib/                           # Pre-built libraries (FROM REFERENCE)
│   ├── NetworkRuntime/            # AI model runtime
│   └── ISP/                       # ISP tuned parameters
│
├── EWARM/                         # IAR project files
├── STM32CubeIDE/                  # STM32CubeIDE project files
├── Makefile                       # GCC Makefile
└── README.md
```

---

## 4. FAL (Firmware Abstraction Layer) - Complete Driver List

All FAL drivers are thread-safe wrappers over HAL, using ThreadX primitives.

### 4.1 Core System Drivers

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **System Init** | `fal_sys` | Clock tree, power domains, cache | Runs before scheduler |
| **Assert/Fault** | `fal_assert` | Hard fault, assert handling | Save context, optional restart |
| **Watchdog** | `fal_iwdg` | Independent watchdog | Dedicated feed thread or distributed |
| **Power Manager** | `fal_pwr` | Sleep modes, voltage scaling | Low-power timer integration |
| **Reset Manager** | `fal_rst` | Reset cause detection, soft reset | Persist data across reset |
| **Clock Manager** | `fal_clk` | Dynamic clock switching | Notify dependent drivers |
| **Cache Manager** | `fal_cache` | L1 I/D cache control | DMA coherency handling |
| **MPU Manager** | `fal_mpu` | Memory protection config | Thread stack protection |

### 4.2 Memory Drivers (Critical for Flashless STM32N6)

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **External Flash** | `fal_xflash` | HexaSPI/OctoSPI NOR/NAND | Mutex protection, XIP considerations |
| **External RAM** | `fal_xram` | PSRAM/SDRAM via FMC | DMA-safe regions, cache coherency |
| **Internal SRAM** | `fal_sram` | 4.2MB SRAM management | Block/Byte pool regions |
| **Memory Pool** | `fal_mempool` | Fixed-size block allocator | `TX_BLOCK_POOL` |
| **Flash Storage** | `fal_storage` | Wear leveling, data persistence | FileX/LevelX integration |

### 4.3 Communication Drivers

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **UART** | `fal_uart` | UART/USART/LPUART | TX queue, RX with semaphore, DMA |
| **SPI Master** | `fal_spi` | SPI master mode | Mutex per bus, chip select handling |
| **SPI Slave** | `fal_spi_slave` | SPI slave mode | ISR + semaphore pattern |
| **I2C** | `fal_i2c` | I2C master | Mutex per bus, timeout handling |
| **I3C** | `fal_i3c` | NEW: I3C interface | Hot-join handling, IBI support |
| **USB Device** | `fal_usb_dev` | USB FS/HS device | USBX integration |
| **USB Host** | `fal_usb_host` | USB host mode | USBX integration |
| **Ethernet** | `fal_eth` | Gigabit Ethernet + TSN | NetXDuo integration |
| **CAN FD** | `fal_fdcan` | CAN FD interface | TX/RX queues, filter config |
| **SDMMC** | `fal_sdmmc` | SD/eMMC interface | FileX integration |

### 4.4 Timing & Synchronization

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **Timer Manager** | `fal_tim` | General-purpose timers | `TX_TIMER` integration |
| **Advanced Timer** | `fal_tim_adv` | PWM, complementary outputs | Motor control patterns |
| **Low-Power Timer** | `fal_lptim` | LPTIM1/2 | Low-power wake source |
| **RTC** | `fal_rtc` | Real-time clock, alarms | Calendar events, wake source |
| **Tick Manager** | `fal_tick` | System tick abstraction | ThreadX tick hook |

### 4.5 Analog & Mixed Signal

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **ADC** | `fal_adc` | 12-bit ADC (x2) | DMA + semaphore, continuous mode |
| **DAC** | `fal_dac` | DAC outputs | Waveform generation |
| **Comparator** | `fal_comp` | Analog comparators | Event flags notification |
| **Op-Amp** | `fal_opamp` | Internal op-amps | Configure and forget |
| **Temperature** | `fal_temp` | Internal temp sensor | Periodic sampling |

### 4.6 Digital I/O

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **GPIO** | `fal_gpio` | GPIO control | Atomic bit operations |
| **EXTI** | `fal_exti` | External interrupts | ISR-to-thread via semaphore |
| **PWM Output** | `fal_pwm_out` | PWM generation | Duty cycle queue |
| **PWM Capture** | `fal_pwm_cap` | Input capture | Measurement via event flags |
| **Encoder** | `fal_encoder` | Quadrature encoder | Position counter access |

### 4.7 Security & Crypto

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **Crypto** | `fal_cryp` | AES, DES, etc. | Mutex, large block handling |
| **Hash** | `fal_hash` | SHA, MD5 | Streaming support |
| **RNG** | `fal_rng` | True random generator | Pre-fill entropy pool |
| **PKA** | `fal_pka` | Public key accelerator | Async operation, callback |
| **SAES** | `fal_saes` | Secure AES | TrustZone integration |
| **TrustZone** | `fal_tz` | Secure/Non-secure boundary | Partition configuration |

### 4.8 Display & Graphics (N6 Specific)

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **LTDC** | `fal_ltdc` | LCD-TFT controller | Frame sync event flags |
| **NeoChrom** | `fal_neochrom` | 2.5D GPU accelerator | Job queue, completion semaphore |
| **DMA2D** | `fal_dma2d` | Chrom-ART accelerator | Async blit operations |
| **JPEG** | `fal_jpeg` | JPEG codec | Encode/decode with semaphore |
| **H264** | `fal_h264` | H.264 encoder | Frame buffer via queue |

### 4.9 Camera & Vision (N6 Specific)

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **DCMI** | `fal_dcmi` | Parallel camera interface | Frame buffer, DMA |
| **CSI** | `fal_csi` | MIPI CSI-2 interface | Frame ready event flags |
| **ISP** | `fal_isp` | Image signal processor | Pipeline configuration |

### 4.10 AI/NPU (N6 Specific)

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **Neural-ART** | `fal_npu` | NPU accelerator | Inference queue, completion semaphore |
| **AI Model** | `fal_ai_model` | Model loading, management | Memory allocation |
| **AI Pipeline** | `fal_ai_pipe` | Vision pipeline integration | Camera→ISP→NPU→Output |

### 4.11 DMA & System

| Module | File | Description | ThreadX Pattern |
|--------|------|-------------|-----------------|
| **DMA** | `fal_dma` | GPDMA controller | Channel allocation, callback |
| **MDMA** | `fal_mdma` | Master DMA | Large transfers |
| **CRC** | `fal_crc` | CRC calculator | Mutex for shared use |
| **WWDG** | `fal_wwdg` | Window watchdog | Careful timing |
| **Backup** | `fal_bkp` | Backup registers/SRAM | Persist across reset |

---

## 5. SVC (Services Layer) - Complete Module List

Reusable software components, not hardware-specific, portable across projects.

### 5.1 OS Abstraction (Portable API over ThreadX)

| Module | File | Description |
|--------|------|-------------|
| **OS Wrapper** | `svc_os` | Thread creation, delay, tick helpers |
| **Mutex Wrapper** | `svc_mutex` | Simplified mutex API with timeout |
| **Semaphore Wrapper** | `svc_sem` | Binary/counting semaphore helpers |
| **Queue Wrapper** | `svc_queue` | Message queue utilities |
| **Event Flags** | `svc_event` | Event groups wrapper |
| **Timer Wrapper** | `svc_timer` | Software timer management |

### 5.2 Data Structures

| Module | File | Description |
|--------|------|-------------|
| **Ring Buffer** | `svc_ringbuf` | Thread-safe circular buffer |
| **FIFO Queue** | `svc_fifo` | Generic FIFO (non-RTOS) |
| **Memory Pool** | `svc_pool` | Fixed-size block allocator |
| **Linked List** | `svc_list` | Intrusive linked list |
| **Hash Map** | `svc_map` | Key-value storage |
| **Bit Array** | `svc_bitarray` | Compact bit manipulation |

### 5.3 Protocol Engines

| Module | File | Description |
|--------|------|-------------|
| **Message Parser** | `svc_msg_parser` | Binary message framing/parsing |
| **Message Builder** | `svc_msg_builder` | Message construction |
| **Message Router** | `svc_msg_router` | Route messages to handlers |
| **Protocol FSM** | `svc_protocol` | Protocol state machine |
| **COBS Codec** | `svc_cobs` | Consistent Overhead Byte Stuffing |
| **SLIP Codec** | `svc_slip` | Serial Line IP framing |
| **HDLC Framing** | `svc_hdlc` | HDLC-like framing |

### 5.4 Data Management

| Module | File | Description |
|--------|------|-------------|
| **Config Framework** | `svc_config` | Runtime configuration system |
| **NVS Abstraction** | `svc_nvs` | Non-volatile storage interface |
| **KV Store** | `svc_kvstore` | Key-value persistent storage |
| **Data Logger** | `svc_datalog` | Structured data logging |
| **File Abstraction** | `svc_file` | Portable file API (over FileX) |

### 5.5 Utilities

| Module | File | Description |
|--------|------|-------------|
| **CRC** | `svc_crc` | CRC8/16/32 calculations |
| **Checksum** | `svc_checksum` | Fletcher, Adler, XOR checksums |
| **Base64** | `svc_base64` | Base64 encode/decode |
| **Hex Utils** | `svc_hex` | Hex string conversion |
| **JSON Parser** | `svc_json` | Lightweight JSON parser |
| **CLI Engine** | `svc_cli` | Command-line interface framework |
| **Assert** | `svc_assert` | Assert macros with logging |
| **Debug** | `svc_debug` | Debug output utilities |
| **Log** | `svc_log` | Logging framework (levels, sinks) |
| **String Utils** | `svc_string` | Safe string manipulation |

### 5.6 Time & Scheduling

| Module | File | Description |
|--------|------|-------------|
| **Time Utils** | `svc_time` | Epoch, conversions, formatting |
| **Calendar** | `svc_calendar` | Date/time calculations |
| **Scheduler** | `svc_scheduler` | Periodic job scheduler |
| **Timeout Manager** | `svc_timeout` | Timeout tracking |
| **Delay Utils** | `svc_delay` | Non-blocking delay patterns |
| **Rate Limiter** | `svc_ratelimit` | Request rate limiting |

### 5.7 State Machines

| Module | File | Description |
|--------|------|-------------|
| **FSM Framework** | `svc_fsm` | Finite state machine engine |
| **HSM Framework** | `svc_hsm` | Hierarchical state machine |
| **Event Dispatcher** | `svc_evtdisp` | Event-driven dispatch |

### 5.8 Math & DSP

| Module | File | Description |
|--------|------|-------------|
| **Filter** | `svc_filter` | IIR, FIR, moving average |
| **PID Controller** | `svc_pid` | PID control algorithm |
| **Interpolation** | `svc_interp` | Linear, cubic interpolation |
| **Statistics** | `svc_stats` | Mean, std dev, min/max tracking |
| **Fixed Point** | `svc_fixedpt` | Fixed-point math utilities |

---

## 6. APP (Application Layer) - Complete Module List

Product-specific code, not portable, uses FAL and SVC.

### 6.1 Thread Definitions

| Thread | Priority | Stack | Description |
|--------|----------|-------|-------------|
| **System Monitor** | 1 (low) | 512 | Stack watermark, heap check |
| **Watchdog Feed** | 2 | 256 | IWDG feed (or distributed) |
| **Storage** | 3 | 1024 | Flash write queue processing |
| **Telemetry TX** | 4 | 1024 | Outgoing telemetry |
| **Telemetry RX** | 4 | 1024 | Incoming message processing |
| **Sensor Sampling** | 5 | 512 | ADC, I2C sensors |
| **GNSS** | 5 | 1024 | GNSS data processing |
| **WiFi** | 5 | 2048 | WiFi stack |
| **Ethernet** | 5 | 2048 | NetXDuo processing |
| **USB** | 6 | 2048 | USBX processing |
| **Display** | 6 | 1024 | GUI rendering |
| **Camera** | 7 | 2048 | Frame capture |
| **AI Inference** | 8 | 4096 | NPU job management |
| **Control Loop** | 9 (high) | 512 | Real-time control |

### 6.2 System Management

| Module | File | Description |
|--------|------|-------------|
| **App Main** | `app_main` | `tx_application_define()`, thread creation |
| **Thread Registry** | `app_threads` | Thread handles, creation functions |
| **System Status** | `sys_status` | Global state machine |
| **System Flags** | `sys_flags` | Event flag management |
| **Error Handler** | `sys_error` | Centralized error management |
| **Boot Manager** | `sys_boot` | Boot sequence, mode selection |

### 6.3 Device Managers

| Module | File | Description |
|--------|------|-------------|
| **LED Manager** | `led_mngr` | LED patterns, status indication |
| **Button Manager** | `btn_mngr` | Debounce, press detection |
| **Buzzer Manager** | `buzzer_mngr` | Audio patterns |
| **Display Manager** | `display_mngr` | Screen content management |
| **Power Manager** | `power_mngr` | Sleep/wake policy |
| **Battery Manager** | `battery_mngr` | Charging, SoC estimation |

### 6.4 Communication Managers

| Module | File | Description |
|--------|------|-------------|
| **Console Manager** | `console_mngr` | Debug CLI interface |
| **Telemetry Master** | `telem_master_mngr` | Telemetry orchestration |
| **Telemetry TX** | `telem_tx_mngr` | Outbound message handling |
| **Telemetry RX** | `telem_rx_mngr` | Inbound message handling |
| **WiFi Manager** | `wifi_mngr` | WiFi connection, provisioning |
| **Ethernet Manager** | `eth_mngr` | Network stack management |
| **GNSS Manager** | `gnss_mngr` | GPS/GNSS processing |
| **BLE Manager** | `ble_mngr` | Bluetooth LE (if applicable) |

### 6.5 Data Managers

| Module | File | Description |
|--------|------|-------------|
| **Storage Manager** | `storage_mngr` | Data persistence, FileX operations |
| **Config Manager** | `config_mngr` | Runtime configuration |
| **Data Tables** | `data_tables` | Shared data structures |
| **Data Logger** | `logger_mngr` | Event/data logging |
| **Backup Manager** | `backup_mngr` | Backup/restore routines |

### 6.6 Timing & Events

| Module | File | Description |
|--------|------|-------------|
| **Tick Manager** | `tick_mngr` | Application tick distribution |
| **Calendar Events** | `calendar_events` | Scheduled event handling |
| **Sync Manager** | `sync_mngr` | Time synchronization |
| **Delay Logic** | `delay_logic` | Timeout management |
| **Scheduler Manager** | `scheduler_mngr` | Periodic task scheduling |

### 6.7 Message Handling

| Module | File | Description |
|--------|------|-------------|
| **Message Handler** | `msg_handler` | Central message dispatcher |
| **Message Router** | `msg_router` | Route messages to handlers |
| **Message Tables** | `msg_tables` | Message ID definitions |
| **Config Actions** | `msg_config_action` | Configuration message handlers |
| **Critical Actions** | `msg_critical_action` | Critical command handlers |
| **Manage Actions** | `msg_manage_action` | Management message handlers |
| **Test Actions** | `msg_test_action` | Test/debug message handlers |
| **ACK Manager** | `msg_ack_mngr` | Acknowledgment handling |
| **Message Queue** | `msg_queue` | Message queuing system |
| **Message Utils** | `msg_utils` | Serialization, checksums |

### 6.8 AI/Vision (N6 Specific)

| Module | File | Description |
|--------|------|-------------|
| **AI Manager** | `ai_mngr` | NPU job scheduling |
| **Vision Pipeline** | `vision_pipe` | Camera→ISP→NPU flow |
| **Object Detector** | `obj_detect` | Detection post-processing |
| **Model Manager** | `model_mngr` | Model loading, switching |

### 6.9 Debug & Diagnostics

| Module | File | Description |
|--------|------|-------------|
| **Debug Manager** | `debug_mngr` | Debug output, logging levels |
| **Assert Handler** | `assert_handler` | Assert/fault processing |
| **Trace Manager** | `trace_mngr` | Execution tracing |
| **Stats Collector** | `stats_mngr` | Performance statistics |
| **Memory Monitor** | `mem_monitor` | Heap/stack monitoring |

---

## 7. ThreadX Configuration (tx_user.h)

### 7.1 Key Settings

```c
/* STM32N6 specific - 800MHz Cortex-M55 */

/* Performance optimizations */
#define TX_MAX_PRIORITIES                       32
#define TX_MINIMUM_STACK                        256
#define TX_TIMER_TICKS_PER_SECOND               1000

/* Enable all features */
#define TX_TIMER_PROCESS_IN_ISR                 /* Faster timer processing */
#define TX_REACTIVATE_INLINE                    /* Inline timer reactivation */
#define TX_DISABLE_STACK_FILLING                /* Disable for production */
#define TX_INLINE_THREAD_RESUME_SUSPEND         /* Inline for performance */

/* Event flags */
#define TX_EVENT_FLAGS_ENABLE_PERFORMANCE_INFO

/* Enable run-time stats (disable for production) */
#define TX_THREAD_ENABLE_PERFORMANCE_INFO
#define TX_BLOCK_POOL_ENABLE_PERFORMANCE_INFO
#define TX_BYTE_POOL_ENABLE_PERFORMANCE_INFO
#define TX_QUEUE_ENABLE_PERFORMANCE_INFO
#define TX_SEMAPHORE_ENABLE_PERFORMANCE_INFO
#define TX_MUTEX_ENABLE_PERFORMANCE_INFO

/* Stack checking (disable for production) */
#define TX_ENABLE_STACK_CHECKING

/* Low power support */
#define TX_LOW_POWER                            /* Enable low-power mode */
#define TX_LOW_POWER_TIMER_SETUP                /* Custom timer setup */
#define TX_LOW_POWER_TICKLESS                   /* Tickless idle */

/* Cortex-M55 specific */
#define TX_PORT_USE_BASEPRI                     /* Use BASEPRI for critical sections */
```

### 7.2 Memory Configuration for 4.2MB SRAM

```c
/* In tx_user.h or linker script */

/* ThreadX byte pool for dynamic allocation */
#define TX_BYTE_POOL_SIZE                       (512 * 1024)  /* 512KB */

/* Block pools for fixed-size allocations */
#define SMALL_BLOCK_POOL_SIZE                   (64 * 1024)   /* 64KB, 64-byte blocks */
#define MEDIUM_BLOCK_POOL_SIZE                  (128 * 1024)  /* 128KB, 256-byte blocks */
#define LARGE_BLOCK_POOL_SIZE                   (256 * 1024)  /* 256KB, 1KB blocks */
```

---

## 8. Common Patterns

### 8.1 FAL Driver Template

```c
/* fal_spi.h */
#ifndef FAL_SPI_H
#define FAL_SPI_H

#include "fal_common.h"
#include "tx_api.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    TX_MUTEX mutex;
    TX_SEMAPHORE tx_complete;
    TX_SEMAPHORE rx_complete;
    DMA_HandleTypeDef *hdma_tx;
    DMA_HandleTypeDef *hdma_rx;
} fal_spi_t;

typedef struct {
    SPI_TypeDef *instance;
    uint32_t baudrate;
    uint8_t mode;
    // ... other config
} fal_spi_config_t;

fal_status_t fal_spi_init(fal_spi_t *drv, const fal_spi_config_t *config);
fal_status_t fal_spi_deinit(fal_spi_t *drv);
fal_status_t fal_spi_transfer(fal_spi_t *drv, const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms);
fal_status_t fal_spi_transmit(fal_spi_t *drv, const uint8_t *tx, size_t len, uint32_t timeout_ms);
fal_status_t fal_spi_receive(fal_spi_t *drv, uint8_t *rx, size_t len, uint32_t timeout_ms);

#endif
```

### 8.2 APP Manager Template

```c
/* sensor_mngr.h */
#ifndef SENSOR_MNGR_H
#define SENSOR_MNGR_H

#include "app_common.h"
#include "tx_api.h"

/* Message types for this manager */
typedef enum {
    SENSOR_MSG_START_SAMPLING,
    SENSOR_MSG_STOP_SAMPLING,
    SENSOR_MSG_GET_DATA,
    SENSOR_MSG_SET_INTERVAL,
} sensor_msg_type_t;

typedef struct {
    sensor_msg_type_t type;
    union {
        uint32_t interval_ms;
        void *response_ptr;
    } payload;
} sensor_msg_t;

/* Public API */
void sensor_mngr_init(void);
void sensor_mngr_thread_entry(ULONG param);
UINT sensor_mngr_request(sensor_msg_t *msg, ULONG timeout);

#endif
```

### 8.3 ISR Callback Pattern (ThreadX)

```c
/* In stm32n6xx_it.c or FAL driver */
void SPI1_IRQHandler(void) {
    if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE)) {
        /* Signal completion via semaphore */
        tx_semaphore_put(&fal_spi1.tx_complete);
    }
    
    HAL_SPI_IRQHandler(&hspi1);
}
```

### 8.4 Thread Entry Pattern

```c
/* In app_threads.c */
void sensor_thread_entry(ULONG param) {
    sensor_msg_t msg;
    UINT status;
    
    /* Initialize manager */
    sensor_mngr_init();
    
    while (1) {
        /* Wait for message with timeout */
        status = tx_queue_receive(&sensor_queue, &msg, TX_WAIT_FOREVER);
        
        if (status == TX_SUCCESS) {
            switch (msg.type) {
                case SENSOR_MSG_START_SAMPLING:
                    /* Handle start */
                    break;
                case SENSOR_MSG_GET_DATA:
                    /* Handle data request */
                    break;
                default:
                    break;
            }
        }
    }
}
```

---

## 9. Development Checklist

### Phase 1: Setup (Based on Reference Project)
- [ ] Clone `x-cube-n6-ai-h264-usb-uvc` as base
- [ ] Verify build and flash with reference project
- [ ] Understand existing memory map and linker script
- [ ] Create FAL, SVC, App directories
- [ ] Set up build system to include new directories
- [ ] Test basic "hello world" in `tx_application_define()`

### Phase 2: FAL Core Drivers
- [ ] Implement `fal_common.h` (types, status codes, macros)
- [ ] Implement `fal_gpio` with atomic operations
- [ ] Implement `fal_uart` with DMA and semaphores
- [ ] Implement `fal_spi` with mutex protection
- [ ] Implement `fal_i2c` with mutex protection
- [ ] Implement `fal_tim` and `fal_tick`
- [ ] Implement `fal_adc` with DMA
- [ ] Test all FAL drivers in multi-thread environment

### Phase 3: SVC Layer
- [ ] Implement `svc_os` wrappers (portable over ThreadX)
- [ ] Implement `svc_ringbuf` (thread-safe)
- [ ] Implement `svc_log` (logging framework)
- [ ] Implement `svc_crc` and `svc_checksum`
- [ ] Implement `svc_msg_parser` (port from existing)
- [ ] Implement `svc_fsm` (state machine framework)
- [ ] Implement `svc_cli` (debug console)

### Phase 4: APP Communication
- [ ] Implement `msg_handler` with queue-based dispatch
- [ ] Port message tables and action handlers
- [ ] Implement telemetry managers
- [ ] Test message throughput and latency
- [ ] Implement `console_mngr` for debugging

### Phase 5: Storage & Persistence
- [ ] Wrap FileX with `svc_file` abstraction
- [ ] Implement `svc_nvs` (non-volatile storage)
- [ ] Implement `storage_mngr` using FileX/LevelX
- [ ] Port `config_mngr`
- [ ] Test data persistence across reset

### Phase 6: Application Logic
- [ ] Implement device managers (LED, buttons, etc.)
- [ ] Implement `sys_status` state machine
- [ ] Implement calendar events with `TX_TIMER`
- [ ] Port remaining application logic

### Phase 7: AI/Vision Pipeline (Leverage Reference)
- [ ] Study existing camera/ISP/NPU pipeline in reference
- [ ] Create FAL wrappers for NPU (`fal_npu`)
- [ ] Implement `ai_mngr` for job scheduling
- [ ] Integrate with your application flow

### Phase 8: Optimization & Hardening
- [ ] Profile CPU usage with ThreadX performance info
- [ ] Tune stack sizes using `tx_thread_info_get()`
- [ ] Enable low-power mode with `TX_LOW_POWER`
- [ ] Stress test under load
- [ ] Code review for thread-safety
- [ ] Remove debug/performance macros for production

---

## 10. Summary Module Count

| Layer | Category | Module Count |
|-------|----------|--------------|
| **FAL** | Core System | 8 |
| **FAL** | Memory | 5 |
| **FAL** | Communication | 10 |
| **FAL** | Timing | 5 |
| **FAL** | Analog | 5 |
| **FAL** | Digital I/O | 5 |
| **FAL** | Security | 6 |
| **FAL** | Display/Graphics | 5 |
| **FAL** | Camera/Vision | 3 |
| **FAL** | AI/NPU | 3 |
| **FAL** | DMA/System | 5 |
| **FAL Total** | | **~60 modules** |
| **SVC** | OS Abstraction | 6 |
| **SVC** | Data Structures | 6 |
| **SVC** | Protocols | 7 |
| **SVC** | Data Management | 5 |
| **SVC** | Utilities | 10 |
| **SVC** | Time/Scheduling | 6 |
| **SVC** | State Machines | 3 |
| **SVC** | Math/DSP | 5 |
| **SVC Total** | | **~48 modules** |
| **APP** | System Management | 6 |
| **APP** | Device Managers | 6 |
| **APP** | Comm Managers | 8 |
| **APP** | Data Managers | 5 |
| **APP** | Timing/Events | 5 |
| **APP** | Message Handling | 10 |
| **APP** | AI/Vision | 4 |
| **APP** | Debug | 5 |
| **APP Total** | | **~49 modules** |
| **Grand Total** | | **~157 modules** |

---

## 11. Next Steps

1. **Review this architecture** with your team, identify gaps specific to your product
2. **Prioritize modules** - Which are essential for MVP? Start with FAL basics
3. **Create coding standards** document - Naming, error handling, documentation, review process
4. **Clone reference project** and get it building
5. **Start Phase 1** - Create directory structure and build integration
6. **Consider CMSIS-RTOS v2** - If you want future portability to FreeRTOS

---

## 12. Reference Links

- **Base Project**: https://github.com/STMicroelectronics/x-cube-n6-ai-h264-usb-uvc
- **STM32CubeN6**: https://github.com/STMicroelectronics/STM32CubeN6
- **X-CUBE-FREERTOS**: https://github.com/STMicroelectronics/x-cube-freertos (if you switch)
- **ThreadX Documentation**: https://learn.microsoft.com/en-us/azure/rtos/threadx/

---

*Document Version: 2.0*
*Target: STM32N6 with ThreadX (Azure RTOS)*
*Base Project: x-cube-n6-ai-h264-usb-uvc*
