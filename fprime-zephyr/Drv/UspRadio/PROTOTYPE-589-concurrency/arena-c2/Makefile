# Arena #589 candidate c2: host simulation of the three UspRadio concurrency
# options against the REAL usp smtc_rac_lib (radio planner, RAC, RAL, RALF,
# sx126x driver) and the REAL RalSessionImpl.cpp / ProfilePolicy.cpp, with
# fakes only at the SX126x SPI HAL and smtc_modem_hal.
#
#   make            build everything
#   make run        rerun every measurement (results/*.txt)
#   make clean
#
# Override USP_V100 / USP_V121 to point at other usp checkouts.
HERE      := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
REPO      := $(abspath $(HERE)/../../../../..)
USPRADIO  := $(REPO)/fprime-zephyr/Drv/UspRadio
USP_V100  ?= $(HOME)/GitHut/pcr-588/lib/zephyr-workspace/modules/lib/usp
USP_V121  ?= $(HOME)/GitHut/arena-589/c2-scratch/usp-v1.2.1
BUILD     ?= $(HOME)/GitHut/arena-589/c2-scratch/build
RESULTS   := $(HERE)results

CXX ?= clang++
CC  ?= clang

define USP_SRCS
$(1)/smtc_rac_lib/radio_planner/src/radio_planner.c \
$(1)/smtc_rac_lib/radio_planner/src/duty_cycle.c \
$(1)/smtc_rac_lib/smtc_rac/smtc_rac.c \
$(1)/smtc_rac_lib/smtc_rac/smtc_rac_lora.c \
$(1)/smtc_rac_lib/smtc_rac/smtc_rac_fsk.c \
$(1)/smtc_rac_lib/smtc_rac/smtc_rac_lrfhss.c \
$(1)/smtc_rac_lib/smtc_rac/smtc_rac_lbt.c \
$(1)/smtc_rac_lib/smtc_rac/smtc_rac_flrc.c \
$(wildcard $(1)/smtc_rac_lib/smtc_rac/smtc_rac_flrc_burst.c) \
$(1)/smtc_rac_lib/smtc_ral/src/ral_sx126x.c \
$(1)/smtc_rac_lib/smtc_ralf/src/ralf_sx126x.c \
$(1)/smtc_rac_lib/radio_drivers/sx126x_driver/src/sx126x.c \
$(1)/smtc_rac_lib/radio_drivers/sx126x_driver/src/sx126x_lr_fhss.c \
$(1)/smtc_rac_lib/radio_drivers/sx126x_driver/src/lr_fhss_mac.c
endef

define USP_INC
-I$(1)/smtc_rac_lib/radio_planner/src -I$(1)/smtc_rac_lib/smtc_rac -I$(1)/smtc_rac_lib/smtc_rac_api \
-I$(1)/smtc_rac_lib/smtc_modem_hal -I$(1)/smtc_rac_lib/smtc_ral/src -I$(1)/smtc_rac_lib/smtc_ralf/src \
-I$(1)/smtc_rac_lib/radio_drivers/sx126x_driver/src
endef

COMMON_DEFS := -DSX126X -DSX1262 -DRP_MARGIN_DELAY=8 -DMODEM_HAL_DBG_TRACE=0 -DCONFIG_LORA_BASICS_MODEM_DRIVERS=1 \
               -DLINK_PROFILES_USE_HOST_TYPES=1 -DRAC_FSK_LOG_ENABLE=0 -DRAC_LORA_LOG_ENABLE=0 -DRAC_LRFHSS_LOG_ENABLE=0
# C sources only: lets hal_fake.cpp wrap smtc_rac_open_radio (claim B).
C_ONLY_DEFS := -Dsmtc_rac_open_radio=sim_real_smtc_rac_open_radio
# smtc_rac.c only: overlap counter wrappers (claim E).
RAC_ONLY_DEFS := -Drp_callback=sim_wrapped_rp_callback -Drp_task_enqueue=sim_wrapped_rp_task_enqueue
SIM_INC     := -I$(HERE)sim/shim -I$(HERE)sim -I$(REPO)
WARN        := -Wall -Wno-unused-parameter -Wno-unused-function -Wno-missing-field-initializers -Wno-unused-variable \
               -Wno-deprecated-declarations -Wno-unused-but-set-variable -Wno-sign-compare -Wno-format
CFLAGS      := -O1 -g $(WARN) -std=gnu11
CXXFLAGS    := -O1 -g $(WARN) -std=gnu++17

SIM_COMMON  := $(HERE)sim/zkernel.cpp $(HERE)sim/hal_fake.cpp $(HERE)sim/sx126x_hal_fake.cpp $(HERE)sim/scenarios.cpp
OPT1_SRCS   := $(HERE)sim/opt1/UspRadioSim1.cpp $(HERE)sim/usp_thread.cpp $(USPRADIO)/RalSessionImpl.cpp $(USPRADIO)/ProfilePolicy.cpp
OPT3_SRCS   := $(HERE)sim/opt1/UspRadioSim1.cpp $(HERE)sim/usp_thread.cpp $(HERE)sim/opt3/RalSessionImpl.cpp $(USPRADIO)/ProfilePolicy.cpp $(HERE)sim/opt3/rac_api_mutex.cpp
OPT2_SRCS   := $(HERE)sim/opt2/UspRadioSim2.cpp $(HERE)sim/opt2/RalSessionSt.cpp $(USPRADIO)/ProfilePolicy.cpp $(HERE)sim/usp_thread_stub.cpp

# name, usp dir, usp label, option label, option srcs, extra flags
define BUILD_SIM
$(BUILD)/$(1): $(SIM_COMMON) $(5) $(call USP_SRCS,$(2)) | $(BUILD)
	@echo "  [build] $(1)"
	@for f in $(call USP_SRCS,$(2)); do \
	  extra=""; case $$$$f in */smtc_rac.c) extra="$(RAC_ONLY_DEFS)";; esac; \
	  $(CC) $(CFLAGS) $(6) $(COMMON_DEFS) $(C_ONLY_DEFS) $$$$extra $(call USP_INC,$(2)) $(SIM_INC) -c $$$$f -o $(BUILD)/$(1).$$$$(basename $$$$f).o || exit 1; done
	@$(CXX) $(CXXFLAGS) $(6) $(COMMON_DEFS) -DSIM_OPTION_NAME='"$(4)"' -DSIM_USP_VERSION='"$(3)"' $(call USP_INC,$(2)) $(SIM_INC) \
	  $(SIM_COMMON) $(5) $(BUILD)/$(1).*.o -o $(BUILD)/$(1) -lpthread
ALL_SIMS += $(BUILD)/$(1)
endef

OPT1_FLAGS := -DCONFIG_USP_MAIN_THREAD=1 -DSIM_DEFAULT_PROTECT_MODE=1
OPT3_FLAGS := -DCONFIG_USP_MAIN_THREAD=1 -DCONFIG_USP_THREADS_MUTEXES=1 -DSIM_DEFAULT_PROTECT_MODE=0 -DSIM_SW_PLATFORM_CXX_FIX=1
OPT2_FLAGS := -DSIM_DEFAULT_PROTECT_MODE=0
TSAN       := -fsanitize=thread
BOUNDS     := -fsanitize=bounds -fno-sanitize-recover=bounds

$(eval $(call BUILD_SIM,opt1_v100,$(USP_V100),v1.0.0,opt1,$(OPT1_SRCS),$(OPT1_FLAGS)))
$(eval $(call BUILD_SIM,opt1_v121,$(USP_V121),v1.2.1,opt1,$(OPT1_SRCS),$(OPT1_FLAGS)))
$(eval $(call BUILD_SIM,opt1_v100_tsan,$(USP_V100),v1.0.0,opt1,$(OPT1_SRCS),$(OPT1_FLAGS) $(TSAN)))
$(eval $(call BUILD_SIM,opt1_v100_bounds,$(USP_V100),v1.0.0,opt1,$(OPT1_SRCS),$(OPT1_FLAGS) $(BOUNDS)))
$(eval $(call BUILD_SIM,opt3_v100,$(USP_V100),v1.0.0,opt3,$(OPT3_SRCS),$(OPT3_FLAGS)))
$(eval $(call BUILD_SIM,opt3_v100_tsan,$(USP_V100),v1.0.0,opt3,$(OPT3_SRCS),$(OPT3_FLAGS) $(TSAN)))
$(eval $(call BUILD_SIM,opt2_v100,$(USP_V100),v1.0.0,opt2,$(OPT2_SRCS),$(OPT2_FLAGS)))
$(eval $(call BUILD_SIM,opt2_v121,$(USP_V121),v1.2.1,opt2,$(OPT2_SRCS),$(OPT2_FLAGS)))
$(eval $(call BUILD_SIM,opt2_v100_tsan,$(USP_V100),v1.0.0,opt2,$(OPT2_SRCS),$(OPT2_FLAGS) $(TSAN)))
$(eval $(call BUILD_SIM,opt2_v100_bounds,$(USP_V100),v1.0.0,opt2,$(OPT2_SRCS),$(OPT2_FLAGS) $(BOUNDS)))

.PHONY: all run clean opt3src linecount
all: opt3src $(ALL_SIMS)

$(BUILD):
	@mkdir -p $(BUILD)

# Option 3 sources: #26's RalSessionImpl.cpp with the prototype's diff applied.
opt3src: $(HERE)sim/opt3/RalSessionImpl.cpp
$(HERE)sim/opt3/RalSessionImpl.cpp: $(USPRADIO)/RalSessionImpl.cpp $(HERE)../upstream-mutex/RalSessionImpl.cpp.diff $(HERE)tools/gen_opt3.sh
	@$(HERE)tools/gen_opt3.sh $(USPRADIO) $(HERE)

run: all
	@$(HERE)tools/run_all.sh $(BUILD) $(RESULTS)

linecount:
	@$(HERE)tools/linecount.sh $(USPRADIO) $(HERE)

clean:
	rm -rf $(BUILD)
