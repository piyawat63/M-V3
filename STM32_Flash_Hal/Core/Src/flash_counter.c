#include "flash_counter.h"
#include <stdio.h>

/* ===== ตั้งค่าตรงนี้ ======================================================== */

/* สวิตช์ที่ใช้: PD1  (กด/เปิดสวิตช์ = High)
 * ต้องมี pull-down ไม่งั้นตอนสวิตช์เปิดวงจร ขาจะลอย -> นับมั่ว
 * ดู MX_GPIO_Init() ใน main.c ให้ตั้ง Pull = GPIO_PULLDOWN */
#define SW_PORT     GPIOD
#define SW_PIN      GPIO_PIN_1

/* ที่เก็บใน flash: H743 Bank2 sector 7 */
#define CNT_BANK    FLASH_BANK_2
#define CNT_SECTOR  FLASH_SECTOR_7
#define CNT_ADDR    0x081E0000UL
#define CNT_SIZE    (128UL * 1024UL)
#define CNT_SLOTS   (CNT_SIZE / 32UL)        /* = 4096 ครั้งก่อนต้องลบ */

#define CNT_MAGIC   0xC0FFEE01UL

/* ========================================================================== */

/* 1 record = 32 byte = flash word ของ H7 พอดี (เขียนน้อยกว่านี้ไม่ได้) */
typedef struct __attribute__((aligned(32))) {
    uint32_t magic;
    uint32_t count;
    uint32_t pad[6];
} cnt_rec_t;

static uint32_t counter;      /* ค่าปัจจุบันใน RAM */
static uint32_t next_slot;    /* ช่องว่างถัดไปใน flash */

/* --------------------------------------------------------------------------- */
static const cnt_rec_t *slot(uint32_t i)
{
    return (const cnt_rec_t *)(CNT_ADDR + i * 32UL);
}

/* --------------------------------------------------------------------------- */
void flash_counter_init(void)
{
    /* หา slot ว่างช่องแรก (flash ที่ลบแล้วจะเป็น 0xFF ทั้งหมด) */
    next_slot = 0;
    while (next_slot < CNT_SLOTS && slot(next_slot)->magic == CNT_MAGIC)
        next_slot++;

    /* ค่าล่าสุด = ช่องก่อนหน้าช่องว่าง */
    counter = (next_slot == 0) ? 0 : slot(next_slot - 1)->count;

    printf("\n--- FLASH COUNTER (SW = PD1) ---\n");
    printf("counter = %lu   (slot %lu/%lu)\n",
           (unsigned long)counter,
           (unsigned long)next_slot, (unsigned long)CNT_SLOTS);
    printf("เปิดสวิตช์ PD1 ให้เป็น High เพื่อ +1\n\n");
}

uint32_t flash_counter_get(void)
{
    return counter;
}

/* --------------------------------------------------------------------------- */
static HAL_StatusTypeDef cnt_erase(void)
{
    FLASH_EraseInitTypeDef e = {0};
    uint32_t err;
    HAL_StatusTypeDef st;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);

    e.TypeErase    = FLASH_TYPEERASE_SECTORS;
    e.Banks        = CNT_BANK;
    e.Sector       = CNT_SECTOR;
    e.NbSectors    = 1;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    st = HAL_FLASHEx_Erase(&e, &err);   /* ใช้เวลาหลายร้อย ms */
    HAL_FLASH_Lock();

    next_slot = 0;
    return st;
}

/* เขียนค่า counter ปัจจุบันลง slot ว่างถัดไป
 * (เขียนทับที่เดิมไม่ได้ — H7 มี ECC จะพัง — ต้องเขียนช่องใหม่เสมอ) */
static HAL_StatusTypeDef cnt_save(void)
{
    cnt_rec_t r __attribute__((aligned(32)));
    HAL_StatusTypeDef st;

    if (next_slot >= CNT_SLOTS) {       /* เต็ม -> ลบแล้วเริ่มเขียนที่ช่อง 0 */
        printf("sector full, erasing...\n");
        if (cnt_erase() != HAL_OK) return HAL_ERROR;
    }

    r.magic = CNT_MAGIC;
    r.count = counter;
    for (int i = 0; i < 6; i++) r.pad[i] = 0xFFFFFFFFUL;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);

    /* หมายเหตุ H7: พารามิเตอร์ที่ 3 คือ "address ของ buffer" ไม่ใช่ตัวข้อมูล */
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                           (uint32_t)slot(next_slot),
                           (uint32_t)&r);
    HAL_FLASH_Lock();

    if (st == HAL_OK) next_slot++;
    return st;
}

void flash_counter_reset(void)
{
    HAL_IWDG_Refresh(&hiwdg1);
    if (cnt_erase() == HAL_OK) {
        counter = 0;
        printf("counter reset to 0\n");
    } else {
        printf("erase FAILED\n");
    }
}

/* --------------------------------------------------------------------------- */
void flash_counter_poll(void)
{
    static uint8_t prev = 0;
    uint8_t now = (HAL_GPIO_ReadPin(SW_PORT, SW_PIN) == GPIO_PIN_SET);
    /* ขอบขาขึ้น: Low -> High */
    if (now && !prev) {
        HAL_Delay(30);                                  /* debounce */
        if (HAL_GPIO_ReadPin(SW_PORT, SW_PIN) == GPIO_PIN_SET) {
            counter++;
            HAL_IWDG_Refresh(&hiwdg1);
            if (cnt_save() == HAL_OK) {
                printf("PD1 High -> counter = %lu   (saved to flash)\n",
                       (unsigned long)counter);
            } else {
                counter--;                              /* เขียนไม่ผ่าน คืนค่าเดิม */
                printf("PD1 High -> SAVE FAILED (err=0x%08lX)\n",
                       (unsigned long)HAL_FLASH_GetError());
            }
        }
    }
    prev = now;
}
