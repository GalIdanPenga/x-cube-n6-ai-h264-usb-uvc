# STM32N6 Framework - Kickoff Checklist

## Day 1 Actions

### Repositories to Create
```bash
# Create these repos in your Git hosting (GitHub/GitLab/Bitbucket)
n6-fal          # Firmware Abstraction Layer
n6-svc          # Services Layer  
n6-bsp          # Board Support Package
n6-app-template # Application template (uses submodules)
n6-tools        # Build scripts, CI tools (optional)
```

### Clone Reference Project
```bash
git clone https://github.com/STMicroelectronics/x-cube-n6-ai-h264-usb-uvc.git
cd x-cube-n6-ai-h264-usb-uvc
```

### Jira Epics to Create
| Epic Key | Name |
|----------|------|
| N6FW-INFRA | Infrastructure & Setup |
| N6FW-FAL | FAL - Firmware Abstraction Layer |
| N6FW-SVC | SVC - Services Layer |
| N6FW-BSP | BSP - Board Support Package |
| N6FW-TEMPLATE | Application Template |
| N6FW-DOC | Documentation |
| N6FW-TEST | Testing & Validation |
| N6FW-CI | CI/CD Pipeline |

---

## Sprint 1 Must-Have Issues

| # | Issue | Priority | Assignee |
|---|-------|----------|----------|
| 1 | Create Git repositories | Critical | Lead |
| 2 | Clone & analyze reference project | Critical | All |
| 3 | Create CubeMX base project (DK) | Critical | Dev1 |
| 4 | Configure FreeRTOS | Critical | Dev1 |
| 5 | Create linker scripts | Critical | Dev2 |
| 6 | Define memory map | High | Lead |
| 7 | Set up Makefile build | High | Dev2 |
| 8 | Create .clang-format | High | Lead |

---

## Architecture Quick Reference

```
┌───────────────────────────────────────┐
│  APP  - Product-specific logic        │ ← Managers, handlers, state machines
├───────────────────────────────────────┤
│  SVC  - Reusable software services    │ ← Protocols, utilities, FSM, logging
├───────────────────────────────────────┤
│  FAL  - Thread-safe HW drivers        │ ← fal_uart, fal_spi, fal_npu, etc.
├───────────────────────────────────────┤
│  HAL  - ST's HAL/LL drivers           │ ← CubeMX generated
├───────────────────────────────────────┤
│  Hardware                             │
└───────────────────────────────────────┘
```

**Dependency Rule**: APP → SVC → FAL → HAL (never skip layers!)

---

## Key STM32N6 Considerations

| Feature | Implication |
|---------|-------------|
| **No internal flash** | Must use external flash (XIP) or RAM-only dev mode |
| **4.2MB SRAM** | Generous stacks, large buffers, AI model weights |
| **800MHz Cortex-M55** | Cache coherency critical with DMA |
| **Neural-ART NPU** | Use ST's runtime, wrap with fal_npu |
| **Helium (MVE)** | Enable for DSP/ML performance |

---

## FreeRTOS Task Priority Template

| Priority | Task Type | Examples |
|----------|-----------|----------|
| 7 (highest) | Real-time control | Motor control, safety |
| 6 | AI inference | NPU job management |
| 5 | Media | Camera, display |
| 4 | Communication | WiFi, Ethernet, GNSS |
| 3 | Telemetry | TX/RX message handling |
| 2 | Storage | Flash writes |
| 1 | Monitoring | Watchdog, stack check |
| 0 (idle) | Idle | Tickless sleep |

---

## Module Naming Conventions

| Layer | Prefix | Example |
|-------|--------|---------|
| FAL | `fal_` | `fal_uart.c`, `fal_spi.h` |
| SVC | `svc_` | `svc_log.c`, `svc_ringbuf.h` |
| BSP | `bsp_` | `bsp_led.c`, `bsp_config.h` |
| APP | `app_` or domain | `led_mngr.c`, `msg_handler.c` |

---

## Week 1 Exit Criteria

- [ ] All repos created with README.md
- [ ] Reference project cloned and documented
- [ ] CubeMX project generating code
- [ ] FreeRTOS blinky LED running
- [ ] UART console output working
- [ ] Memory map documented
- [ ] Build system working (make/IDE)
- [ ] Jira populated with Sprint 1 issues
- [ ] Team aligned on architecture

---

## Quick Commands

```bash
# Build (Makefile)
make -j8

# Flash via STM32CubeProgrammer
export DKEL="<path>/MX66UW1G45G_STM32N6570-DK.stldr"
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -el $DKEL -hardRst -w build/Project_sign.bin 0x70100000

# Sign binary
STM32_SigningTool_CLI -bin build/Project.bin -nk -t ssbl -hv 2.3 -o build/Project_sign.bin

# Debug (GDB)
ST-LINK_gdbserver -p 61234 -l 1 -d -s -cp <cubeprog-bin> -m 1 -g
arm-none-eabi-gdb build/Project.elf -ex "target remote :61234" -ex "load"
```

---

## Files to Analyze from Reference Project

Priority files to study in x-cube-n6-ai-h264-usb-uvc:

1. `App/Src/main.c` - Entry point, task creation
2. `App/Inc/FreeRTOSConfig.h` - RTOS configuration
3. `App/Src/app_freertos.c` - Task definitions
4. Linker scripts (`.ld` files)
5. Camera/ISP/NPU integration code
6. Memory map in linker scripts

---

*Print this page for your kickoff meeting!*
