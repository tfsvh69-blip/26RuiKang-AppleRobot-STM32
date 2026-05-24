# Project Guidelines

## Code Style
- Keep custom logic inside CubeMX USER CODE blocks; avoid edits in generated sections (for example Core/Src/gpio.c).
- Prefer RTOS-aware delays (`osDelay`) when the scheduler is running; use `HAL_Delay` only during init.
- Follow existing module patterns (Init/On/Off/Toggle) and header structure; see [CLAUDE.md](CLAUDE.md).

## Architecture
- FreeRTOS tasks are defined in [Core/Src/freertos.c](Core/Src/freertos.c); SC16 RX tasks are in [Core/Src/sc16_tasks.c](Core/Src/sc16_tasks.c).
- SC16IS752 SPI bridge driver lives in [Core/Src/sc16is752.c](Core/Src/sc16is752.c); UART-based lidars use UART4/5/USART6.
- Hardware mappings, UART/SPI/I2C assignments, and module roles are documented in [CLAUDE.md](CLAUDE.md).

## Build and Test
- Build/flash/debug use Keil MDK-ARM; follow the steps in [CLAUDE.md](CLAUDE.md).

## Conventions
- If CubeMX regeneration is needed, update v1.0.ioc and keep USER CODE sections intact; see [CLAUDE.md](CLAUDE.md).
- Known SC16IS752 robustness notes and debugging tips are tracked in [IMPROVEMENTS.md](IMPROVEMENTS.md).
