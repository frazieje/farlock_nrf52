TARGETS = custom_nrf52832 pca10040 pca10056

.PHONY: all $(TARGETS)

all: $(TARGETS)

custom_nrf52832:
	@$(MAKE) -C custom_nrf52832 -f Makefile

custom_nrf52832_clean:
	@$(MAKE) -C custom_nrf52832 -f Makefile clean

custom_nrf52832_erase:
	@$(MAKE) -C custom_nrf52832 -f Makefile erase

custom_nrf52832_flash_softdevice:
	@$(MAKE) -C custom_nrf52832 -f Makefile flash_softdevice

custom_nrf52832_flash:
	@$(MAKE) -C custom_nrf52832 -f Makefile flash

pca10040:
	@$(MAKE) -C pca10040 -f Makefile

pca10040_clean:
	@$(MAKE) -C pca10040 -f Makefile clean

pca10040_erase:
	@$(MAKE) -C pca10040 -f Makefile erase

pca10040_flash_softdevice:
	@$(MAKE) -C pca10040 -f Makefile flash_softdevice

pca10040_flash:
	@$(MAKE) -C pca10040 -f Makefile flash

pca10056:
	@$(MAKE) -C pca10056 -f Makefile

pca10056_clean:
	@$(MAKE) -C pca10056 -f Makefile clean

pca10056_erase:
	@$(MAKE) -C pca10056 -f Makefile erase

pca10056_flash_softdevice:
	@$(MAKE) -C pca10056 -f Makefile flash_softdevice

pca10056_flash:
	@$(MAKE) -C pca10056 -f Makefile flash

clean:
	@$(MAKE) -C custom_nrf52832 -f Makefile clean
	@$(MAKE) -C pca10040 -f Makefile clean
	@$(MAKE) -C pca10056 -f Makefile clean
