PROJECT = smart

ARCHFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS = -Os -std=c99 -g $(ARCHFLAGS) -Wextra -Wall -pedantic -fno-common -ffunction-sections -fdata-sections -MD -DSTM32F407VG -DSTM32F4CCM -DSTM32F4 -Ilibopencm3/include
LDFLAGS = --static -nostartfiles $(ARCHFLAGS)

CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

LDSCRIPT = generated.STM32F407VG.ld

all: libopencm3/lib/libopencm3_stm32f4.a $(PROJECT).o $(PROJECT).bin $(PROJECT).elf

libopencm3/lib/libopencm3_stm32f4.a:
	@make -j8 -C libopencm3

%.o: %.c
	@$(CC) $(CFLAGS) -c $<

$(LDSCRIPT):
	@$(CC) -E $(ARCHFLAGS) -D_ROM=1024K -D_RAM=128K -D_CCM=64K -DSTM32F4CCM -D_CCM_OFF=0x10000000 -DSTM32F4 -D_ROM_OFF=0x08000000 -D_RAM_OFF=0x20000000  -P -E libopencm3/ld/linker.ld.S > $@

%.elf: %.o $(LDSCRIPT)
	@$(CC) -T$(LDSCRIPT) -Wl,-Map=$(*).map $(LDFLAGS) $< -Wl,--gc-sections -Llibopencm3/lib -lopencm3_stm32f4 -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group -o $@

%.bin: %.elf
	@$(OBJCOPY) -Obinary $< $@

flash: $(PROJECT).elf $(PROJECT).bin
	st-flash write $(PROJECT).bin 0x8000000
	openocd -f interface/stlink-v2.cfg \
	        -f target/stm32f4x.cfg \
	        -c "program $(PROJECT).elf verify reset exit"

clean:
	rm -f *.d *.bin *.elf *.map *.o $(LDSCRIPT)

.PHONY: clean flash all
