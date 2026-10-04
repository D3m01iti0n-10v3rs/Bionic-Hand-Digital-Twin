/*
 * flash_stm32f103.h
 *
 *  Created on: Sep 29, 2026
 *      Author: PC
 */

#ifndef INC_FLASH_STM32F103_H_
#define INC_FLASH_STM32F103_H_

#include <string.h>
#include <stdint.h>



#ifndef FLASH_ACR

#define FLASH_ACR 				(*(volatile uint32_t *)0x40022000UL)
#define FLASH_ACR_LATENCY_Pos 	0
#define FLASH_ACR_LATENCY_Msk   (0x7UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_0WS   (0x0UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_1WS   (0x1UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_2WS   (0x2UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_HLFCYA        (1UL << 3)
#define FLASH_ACR_PRFTBE        (1UL << 4)
#define FLASH_ACR_PRFTBS        (1UL << 5)

#define FLASH_KEYR 				(*(volatile uint32_t *)0x40022004UL)
#define FLASH_KEY1              0x45670123UL
#define FLASH_KEY2              0xCDEF89ABUL
#define FLASH_RDPRT_KEY         0x000000A5UL

#define FLASH_OPTKEYR 			(*(volatile uint32_t *)0x40022008UL)


#define FLASH_SR 				(*(volatile uint32_t *)0x4002200CUL)
#define FLASH_SR_BSY            (1UL << 0)
#define FLASH_SR_PGERR          (1UL << 2)
#define FLASH_SR_WRPRTERR       (1UL << 4)
#define FLASH_SR_EOP            (1UL << 5)

#define FLASH_CR 				(*(volatile uint32_t *)0x40022010UL)
#define FLASH_CR_PG             (1UL << 0)
#define FLASH_CR_PER            (1UL << 1)
#define FLASH_CR_MER            (1UL << 2)
#define FLASH_CR_OPTPG          (1UL << 4)
#define FLASH_CR_OPTER          (1UL << 5)
#define FLASH_CR_STRT           (1UL << 6)
#define FLASH_CR_LOCK           (1UL << 7)
#define FLASH_CR_OPTWRE         (1UL << 9)
#define FLASH_CR_ERRIE          (1UL << 10)
#define FLASH_CR_EOPIE          (1UL << 12)

#define FLASH_AR 				(*(volatile uint32_t *)0x40022014UL)


#define FLASH_OBR 				(*(volatile uint32_t *)0x4002201CUL)
#define FLASH_OBR_OPTERR        (1UL << 0)
#define FLASH_OBR_RDPRT         (1UL << 1)
#define FLASH_OBR_WDG_SW        (1UL << 2)
#define FLASH_OBR_nRST_STOP     (1UL << 3)
#define FLASH_OBR_nRST_STDBY    (1UL << 4)
#define FLASH_OBR_DATA0_Pos     10
#define FLASH_OBR_DATA0_Msk     (0xFFUL << FLASH_OBR_DATA0_Pos)
#define FLASH_OBR_DATA1_Pos     18
#define FLASH_OBR_DATA1_Msk     (0xFFUL << FLASH_OBR_DATA1_Pos)

#define FLASH_WRPR 				(*(volatile uint32_t *)0x40022020UL)

#endif



uint32_t Flash_ErasePage(uint32_t addr);

uint8_t Flash_ReadByte(uint32_t addr);
uint16_t Flash_ReadHlfWord(uint32_t addr);
uint32_t Flash_ReadWord(uint32_t addr);

uint32_t Flash_WriteByte(uint32_t addr, uint8_t data);
uint32_t Flash_WriteHlfWord(uint32_t addr, uint16_t data);
uint32_t Flash_WriteWord(uint32_t addr, uint32_t data);

void Flash_ReadByte_Buf(uint32_t addr, uint8_t *buf, uint32_t n_bytes);
void Flash_ReadHlfWord_Buf(uint32_t addr, uint16_t *buf, uint32_t n_hlfwords);
void Flash_ReadWord_Buf(uint32_t addr, uint32_t *buf, uint32_t n_words);

uint32_t Flash_WriteByte_Buf(uint32_t addr, const uint8_t *buf, uint32_t n_bytes);
uint32_t Flash_WriteHlfWord_Buf(uint32_t addr, const uint16_t *buf, uint32_t n_hlfwords);
uint32_t Flash_WriteWord_Buf(uint32_t addr, const uint32_t *buf, uint32_t n_words);

#endif /* INC_FLASH_STM32F103_H_ */
