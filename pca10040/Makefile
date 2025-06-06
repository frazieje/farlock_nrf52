PROJECT_NAME     := iot_lwip_mqtt_subscriber_pca10040_s132
TARGETS          := nrf52832_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := $(NRF_SDK_HOME)
PROJ_DIR = ..

$(OUTPUT_DIRECTORY)/nrf52832_xxaa.out: \
  LINKER_SCRIPT  := farlock_gcc_nrf52_s132.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52.S \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_uart.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(PROJ_DIR)/utf8.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/external/lwip/src/core/def.c \
  $(SDK_ROOT)/external/lwip/src/core/dhcp.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv6/dhcp6.c \
  $(SDK_ROOT)/external/lwip/src/core/dns.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv4/icmp.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv6/icmp6.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv6/inet6.c \
  $(SDK_ROOT)/external/lwip/src/core/inet_chksum.c \
  $(SDK_ROOT)/external/lwip/src/core/init.c \
  $(SDK_ROOT)/external/lwip/src/core/ip.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv4/ip4.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv4/ip4_addr.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv6/ip6.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv6/ip6_addr.c \
  $(SDK_ROOT)/external/lwip/src/core/memp.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv6/mld6.c \
  $(SDK_ROOT)/external/lwip/src/core/ipv6/nd6.c \
  $(SDK_ROOT)/external/lwip/src/core/netif.c \
  $(SDK_ROOT)/external/lwip/src/port/nrf_platform_port.c \
  $(SDK_ROOT)/external/lwip/src/core/pbuf.c \
  $(SDK_ROOT)/external/lwip/src/core/raw.c \
  $(SDK_ROOT)/external/lwip/src/core/sys.c \
  $(SDK_ROOT)/external/lwip/src/core/tcp.c \
  $(SDK_ROOT)/external/lwip/src/core/tcp_in.c \
  $(SDK_ROOT)/external/lwip/src/core/tcp_out.c \
  $(SDK_ROOT)/external/lwip/src/core/timeouts.c \
  $(SDK_ROOT)/external/lwip/src/core/udp.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c \
  $(SDK_ROOT)/components/iot/medium/ble_ncfgs/ble_ncfgs.c \
  $(SDK_ROOT)/components/iot/medium/commissioning/commissioning.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/pwm/app_pwm.c \
  $(SDK_ROOT)/components/libraries/fifo/app_fifo.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/mem_manager/mem_manager.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_nvmc.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/queue/nrf_queue.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_rng.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_ppi.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_uart.c \
  $(SDK_ROOT)/modules/nrfx/hal/nrf_nvmc.c \
  $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_rng.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uart.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_ppi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp.c \
  $(PROJ_DIR)/farlock.c \
  $(SDK_ROOT)/external/mbedtls/library/aes.c \
  $(SDK_ROOT)/external/mbedtls/library/aesni.c \
  $(SDK_ROOT)/external/mbedtls/library/arc4.c \
  $(SDK_ROOT)/external/mbedtls/library/asn1parse.c \
  $(SDK_ROOT)/external/nrf_tls/mbedtls/replacements/asn1write.c \
  $(SDK_ROOT)/external/mbedtls/library/base64.c \
  $(SDK_ROOT)/external/mbedtls/library/bignum.c \
  $(SDK_ROOT)/external/mbedtls/library/blowfish.c \
  $(SDK_ROOT)/external/mbedtls/library/camellia.c \
  $(SDK_ROOT)/external/mbedtls/library/ccm.c \
  $(SDK_ROOT)/external/mbedtls/library/certs.c \
  $(SDK_ROOT)/external/mbedtls/library/cipher.c \
  $(SDK_ROOT)/external/mbedtls/library/cipher_wrap.c \
  $(SDK_ROOT)/external/mbedtls/library/cmac.c \
  $(SDK_ROOT)/external/mbedtls/library/ctr_drbg.c \
  $(SDK_ROOT)/external/mbedtls/library/debug.c \
  $(SDK_ROOT)/external/mbedtls/library/des.c \
  $(SDK_ROOT)/external/mbedtls/library/dhm.c \
  $(SDK_ROOT)/external/mbedtls/library/ecdh.c \
  $(SDK_ROOT)/external/mbedtls/library/ecdsa.c \
  $(SDK_ROOT)/external/mbedtls/library/ecp.c \
  $(SDK_ROOT)/external/mbedtls/library/ecp_curves.c \
  $(SDK_ROOT)/external/mbedtls/library/entropy.c \
  $(SDK_ROOT)/external/mbedtls/library/entropy_poll.c \
  $(SDK_ROOT)/external/mbedtls/library/error.c \
  $(SDK_ROOT)/external/mbedtls/library/gcm.c \
  $(SDK_ROOT)/external/mbedtls/library/havege.c \
  $(SDK_ROOT)/external/mbedtls/library/hmac_drbg.c \
  $(SDK_ROOT)/external/mbedtls/library/md.c \
  $(SDK_ROOT)/external/mbedtls/library/md2.c \
  $(SDK_ROOT)/external/mbedtls/library/md4.c \
  $(SDK_ROOT)/external/mbedtls/library/md5.c \
  $(SDK_ROOT)/external/mbedtls/library/md_wrap.c \
  $(SDK_ROOT)/external/mbedtls/library/memory_buffer_alloc.c \
  $(SDK_ROOT)/external/mbedtls/library/oid.c \
  $(SDK_ROOT)/external/mbedtls/library/padlock.c \
  $(SDK_ROOT)/external/mbedtls/library/pem.c \
  $(SDK_ROOT)/external/mbedtls/library/pk.c \
  $(SDK_ROOT)/external/mbedtls/library/pk_wrap.c \
  $(SDK_ROOT)/external/mbedtls/library/pkcs11.c \
  $(SDK_ROOT)/external/mbedtls/library/pkcs12.c \
  $(SDK_ROOT)/external/mbedtls/library/pkcs5.c \
  $(SDK_ROOT)/external/mbedtls/library/pkparse.c \
  $(SDK_ROOT)/external/mbedtls/library/pkwrite.c \
  $(SDK_ROOT)/external/mbedtls/library/platform.c \
  $(SDK_ROOT)/external/mbedtls/library/ripemd160.c \
  $(SDK_ROOT)/external/mbedtls/library/rsa.c \
  $(SDK_ROOT)/external/mbedtls/library/sha1.c \
  $(SDK_ROOT)/external/mbedtls/library/sha256.c \
  $(SDK_ROOT)/external/mbedtls/library/sha512.c \
  $(SDK_ROOT)/external/mbedtls/library/ssl_cache.c \
  $(SDK_ROOT)/external/mbedtls/library/ssl_ciphersuites.c \
  $(SDK_ROOT)/external/mbedtls/library/ssl_cli.c \
  $(SDK_ROOT)/external/mbedtls/library/ssl_cookie.c \
  $(SDK_ROOT)/external/nrf_tls/mbedtls/replacements/ssl_srv.c \
  $(SDK_ROOT)/external/mbedtls/library/ssl_ticket.c \
  $(SDK_ROOT)/external/mbedtls/library/ssl_tls.c \
  $(SDK_ROOT)/external/mbedtls/library/threading.c \
  $(SDK_ROOT)/external/nrf_tls/mbedtls/tls_interface.c \
  $(SDK_ROOT)/external/mbedtls/library/version.c \
  $(SDK_ROOT)/external/mbedtls/library/version_features.c \
  $(SDK_ROOT)/external/mbedtls/library/x509.c \
  $(SDK_ROOT)/external/mbedtls/library/x509_create.c \
  $(SDK_ROOT)/external/mbedtls/library/x509_crl.c \
  $(SDK_ROOT)/external/mbedtls/library/x509_crt.c \
  $(SDK_ROOT)/external/mbedtls/library/x509_csr.c \
  $(SDK_ROOT)/external/mbedtls/library/xtea.c \
  $(SDK_ROOT)/components/iot/ble_6lowpan/ble_6lowpan.c \
  $(SDK_ROOT)/components/iot/errno/errno.c \
  $(SDK_ROOT)/components/iot/context_manager/iot_context_manager.c \
  $(SDK_ROOT)/components/iot/iot_timer/iot_timer.c \
  $(SDK_ROOT)/components/iot/medium/ipv6_medium_ble.c \
  $(SDK_ROOT)/components/iot/mqtt/mqtt.c \
  $(SDK_ROOT)/components/iot/mqtt/mqtt_decoder.c \
  $(SDK_ROOT)/components/iot/mqtt/mqtt_encoder.c \
  $(SDK_ROOT)/components/iot/mqtt/mqtt_rx.c \
  $(SDK_ROOT)/components/iot/mqtt/mqtt_transport.c \
  $(SDK_ROOT)/components/iot/mqtt/mqtt_transport_lwip.c \
  $(SDK_ROOT)/components/iot/mqtt/mqtt_transport_tls.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/external/utf_converter/utf.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_ipsp/ble_ipsp.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR)/config \
  $(SDK_ROOT)/components/iot/errno \
  $(SDK_ROOT)/components/nfc/t4t_lib \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/nfc/ndef/generic/message \
  $(SDK_ROOT)/components/ble/ble_services/ble_tps \
  $(PROJ_DIR) \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/le_oob_rec_parser \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/nfc/ndef/uri \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_msg \
  $(SDK_ROOT)/components/libraries/cli \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ac_rec_parser \
  $(SDK_ROOT)/external/nrf_tls \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ep_oob_rec \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/common \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/external/lwip/src/include/lwip \
  $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ac_rec \
  $(SDK_ROOT)/components/libraries/usbd/class/hid \
  $(SDK_ROOT)/components/libraries/twi_sensor \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/le_oob_rec \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/iot/medium/commissioning \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_oob_advdata \
  $(SDK_ROOT)/components/nfc/ndef/parser/message \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  $(SDK_ROOT)/components/ble/ble_services/ble_ipsp \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/components/iot/medium \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components/libraries/stack_guard \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs \
  $(SDK_ROOT)/components/iot/ble_6lowpan \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/generic \
  $(SDK_ROOT)/components/nfc/t2t_parser \
  $(SDK_ROOT)/components/libraries/experimental_task_manager \
  $(SDK_ROOT)/external/lwip/src/port \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids \
  $(SDK_ROOT)/components/iot/iot_timer \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/external/utf_converter \
  $(SDK_ROOT)/components/libraries/sdcard \
  $(SDK_ROOT)/components/libraries/svc \
  $(SDK_ROOT)/components/ble/ble_services/ble_lls \
  $(SDK_ROOT)/components/iot/medium/ble_ncfgs \
  $(SDK_ROOT)/components/nfc/ndef/parser/record \
  $(SDK_ROOT)/components/softdevice/s132/headers \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/ble/ble_services/ble_cscs \
  $(SDK_ROOT)/external/lwip/src/include \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd \
  $(PROJ_DIR)/pca10040/config \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/external/nrf_tls/mbedtls/tls/config \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/fifo \
  $(SDK_ROOT)/components/nfc/t4t_parser/tlv \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias_c \
  $(SDK_ROOT)/components/nfc/t4t_parser/apdu \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/nfc/ndef/generic/record \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/components/libraries/usbd/class/audio \
  $(SDK_ROOT)/components/nfc/t4t_parser/hl_detection_procedure \
  $(SDK_ROOT)/components/libraries/ecc \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/nfc/ndef/launchapp \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_ans_c \
  $(SDK_ROOT)/components/libraries/twi_mngr \
  $(SDK_ROOT)/components/iot/common \
  $(SDK_ROOT)/components/iot/context_manager \
  $(SDK_ROOT)/external/mbedtls/include \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c \
  $(SDK_ROOT)/external/lwip/src/port/arch \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/nfc/t4t_parser/cc_file \
  $(SDK_ROOT)/components/ble/ble_services/ble_gls \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/spi_mngr \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/external/lwip/src/include/netif \
  $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/components/libraries/mpu \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/nfc/ndef/text \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_lib \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/iot/mqtt \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/libraries/bootloader/ble_dfu \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/hs_rec \
  $(SDK_ROOT)/components/nfc/t2t_lib \
  $(SDK_ROOT)/components/libraries/gfx \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/components/ble/ble_dtm \

# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DMBEDTLS_CONFIG_FILE=\"nrf_tls_config.h\"
CFLAGS += -DNRF52
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -DNRF_SD_BLE_API_VERSION=6
CFLAGS += -DNRF_TLS_MAX_INSTANCE_COUNT=1
CFLAGS += -DS132
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSWI_DISABLE0
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -w
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52_PAN_74
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=6
ASMFLAGS += -DNRF_TLS_MAX_INSTANCE_COUNT=1
ASMFLAGS += -DS132
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSWI_DISABLE0

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

nrf52832_xxaa: CFLAGS += -D__HEAP_SIZE=0
nrf52832_xxaa: CFLAGS += -D__STACK_SIZE=4608
nrf52832_xxaa: ASMFLAGS += -D__HEAP_SIZE=0
nrf52832_xxaa: ASMFLAGS += -D__STACK_SIZE=4608

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52832_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52832_xxaa
	@echo		flash_softdevice
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex --sectorerase
	nrfjprog -f nrf52 --reset

# Flash softdevice
flash_softdevice:
	@echo Flashing: s132 nrf52 softdevice
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_*_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := $(PROJ_DIR)/pca10040/config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
