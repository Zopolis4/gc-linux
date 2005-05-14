obj-$(CONFIG_GAMECUBE_DI) += gcn-di.o

$(obj)/gcn-di.o: $(obj)/drive_20020402.h

#CONFIG_GAMECUBE_DI_BUILD_FIRMWARE=y

ifeq ($(CONFIG_GAMECUBE_DI_BUILD_FIRMWARE),y)

ASMN102 = mn10200-linux-as
LDMN102 = mn10200-linux-ld
OCMN102 = mn10200-linux-objcopy

quiet_cmd_build_difw = BLD FW	$@
cmd_build_difw = \
	$(ASMN102) -o $(obj)/$(<F).o $<; \
	$(LDMN102) --section-start absolute=0 -Ttext=0x40d000 \
		-o $(obj)/$(<F).elf -e 0x40d000 $(obj)/$(<F).o; \
	$(OCMN102) -I elf32-mn10200 -O binary $(obj)/$(<F).elf \
		$(obj)/$(<F).bin; \
	(echo "static "; cat $(obj)/$(<F).bin | scripts/bin2c "$(subst .asm,,$(<F))_firmware") > $@; \
	rm -f $(obj)/$(<F).o $(obj)/$(<F).elf $(obj)/$(<F).bin
	

targets += drive_20020402.h
$(obj)/drive_20020402.h: $(src)/drive_20020402.asm FORCE
	$(call if_changed,build_difw)

endif


