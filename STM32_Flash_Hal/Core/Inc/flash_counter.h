#ifndef FLASH_COUNTER_H
#define FLASH_COUNTER_H

#include "main.h"

/* -----------------------------------------------------------------------------
 * ตัวนับง่ายๆ เก็บใน internal flash ของ STM32H743ZITX
 *
 *   เปิดสวิตช์ที่ PD1 (ขาเป็น High) -> counter + 1 -> เขียนลง flash
 *   ปิด-เปิดบอร์ด                   -> ค่ายังอยู่
 *
 * ที่เก็บ: Bank2 sector 7 (0x081E0000, 128 KB) — bank เดียวกับที่โค้ดไม่ได้ใช้
 * ---------------------------------------------------------------------------*/

/* เรียก 1 ครั้งใน main() หลัง MX_GPIO_Init() + MX_UART4_Init() */
void     flash_counter_init(void);

/* เรียกรัวๆ ใน while(1) — มันจะเช็คสวิตช์เอง */
void     flash_counter_poll(void);

/* ค่าปัจจุบัน */
uint32_t flash_counter_get(void);

/* รีเซ็ตกลับเป็น 0 (ลบทั้ง sector) */
void     flash_counter_reset(void);

#endif /* FLASH_COUNTER_H */
