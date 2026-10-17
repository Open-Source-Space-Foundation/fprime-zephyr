# Host simulation build.  VARIANT selects the usp / usp_zephyr sources and the concurrency option:
#   opt1        pinned usp v1.0.0 + fork mutex (bbdd729) + USP thread          (option 1, today)
#   opt3        pinned usp v1.0.0 + fork dropped + USP thread + THREADS_MUTEXES (option 3), RalSessionImpl = real diff applied
#   opt2        pinned usp v1.0.0 + fork dropped, no USP thread                 (option 2)
#   opt0        pinned usp v1.0.0, NO fork, USP thread, no mutex at all (the state before bbdd729; control: HWIL 2026-07-11 failed here)
#   opt1_v121   usp v1.2.1 + usp_zephyr v1.2.1 (+ fork patch)                   (option 1 on v1.2.1: claim F)
#   opt3_v121   usp v1.2.1 + usp_zephyr v1.2.1, THREADS_MUTEXES                 (option 3 on v1.2.1)
#   opt2_v121   usp v1.2.1 + usp_zephyr v1.2.1, no USP thread                   (option 2 on v1.2.1)
HERE      := $(abspath .)
WT        := $(abspath $(HERE)/../../../../..)
SCR       := $(HOME)/GitHut/arena-589/c3-scratch
PCR       := $(HOME)/GitHut/pcr-588/lib/zephyr-workspace/modules/lib
UR        := $(WT)/fprime-zephyr/Drv/UspRadio
PROTO     := $(UR)/PROTOTYPE-589-concurrency

USP_PRIO  ?= 2
VARIANT   ?= opt1
BUILD     := $(HERE)/build/$(VARIANT)$(if $(filter-out 2,$(USP_PRIO)),_prio$(USP_PRIO))

ifneq (,$(findstring v121,$(VARIANT)))
  CFG_DEFS_EXTRA := -DNO_RF_SWITCH
  USP := $(SCR)/usp-v121
  ifneq (,$(findstring opt1,$(VARIANT)))
    USPZ := $(SCR)/uspz121-fork
  else
    USPZ := $(SCR)/usp_zephyr-v121
  endif
else
  USP := $(PCR)/usp
  ifneq (,$(findstring opt1,$(VARIANT)))
    USPZ := $(SCR)/uspz-fork
  else
    USPZ := $(SCR)/uspz-nofork
  endif
endif

CFG_DEFS := $(CFG_DEFS_EXTRA) -DCONFIG_USP=1 -DCONFIG_LORA_BASICS_MODEM_DRIVERS=1 -DCONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER=1 \
  -DCONFIG_LORA_BASICS_MODEM_DRIVERS_EVENT_TRIGGER_GLOBAL_THREAD=1 -DCONFIG_LORA_BASICS_MODEM_DRIVERS_HAL_WAIT_ON_BUSY_TIMEOUT_MSEC=600000 \
  -DCONFIG_USP_LOG_LEVEL=0 -DCONFIG_LORA_BASICS_MODEM_DRIVERS_LOG_LEVEL=0 -DCONFIG_LORA_BASICS_MODEM_DRIVERS_RAL_RALF=1 \
  -DCONFIG_USP_MAIN_THREAD_PRIORITY=$(USP_PRIO) -DCONFIG_USP_MAIN_THREAD_MAX_SLEEP_MS=60000 -DCONFIG_USP_MAIN_THREAD_STACK_SIZE=4096
ifneq (,$(findstring opt2,$(VARIANT)))
else
  CFG_DEFS += -DCONFIG_USP_MAIN_THREAD=1
endif
ifneq (,$(findstring opt3,$(VARIANT)))
  CFG_DEFS += -DCONFIG_USP_THREADS_MUTEXES=1
endif
LIB_DEFS := -DSX126X -DSX126X_TRANSCEIVER -DSX126X_DISABLE_WARNINGS -DSX1262 -DUSE_SMTC_RAC -DRP_MARGIN_DELAY=8 -DENABLE_ADVANCED_LOGGING=0

RACLIB := $(USP)/smtc_rac_lib
INC := -I$(HERE)/zsim -I$(HERE)/shim -I$(HERE)/chip \
  -I$(USPZ)/include -I$(USPZ)/drivers/usp/sx126x -I$(USPZ)/modules/smtc_modem_hal -I$(USPZ)/subsys/usp \
  -I$(RACLIB)/smtc_rac_api -I$(RACLIB)/smtc_rac -I$(RACLIB)/radio_planner/src -I$(RACLIB)/smtc_modem_hal \
  -I$(RACLIB)/smtc_ral/src -I$(RACLIB)/smtc_ralf/src -I$(RACLIB)/radio_drivers/sx126x_driver/src

CC  := clang
CXX := clang++
WARN := -w -MMD -MP
ifneq (,$(findstring asan,$(VARIANT)))
  SAN := -fsanitize=address -fno-omit-frame-pointer
endif
CFLAGS   := $(SAN) -O1 -g -std=gnu11 $(WARN) $(CFG_DEFS) $(LIB_DEFS) $(INC)
CXXFLAGS := $(SAN) -O1 -g -std=gnu++17 $(WARN) $(CFG_DEFS) -DLINK_PROFILES_USE_HOST_TYPES $(INC) -I$(BUILD)/patched -I$(WT) -I$(HERE)/opt2

C_SRCS := $(wildcard $(RACLIB)/radio_planner/src/*.c) $(wildcard $(RACLIB)/smtc_rac/*.c) \
  $(RACLIB)/smtc_ral/src/ral_sx126x.c $(RACLIB)/smtc_ralf/src/ralf_sx126x.c \
  $(RACLIB)/radio_drivers/sx126x_driver/src/sx126x.c $(RACLIB)/radio_drivers/sx126x_driver/src/sx126x_driver_version.c \
  $(RACLIB)/radio_drivers/sx126x_driver/src/sx126x_lr_fhss.c $(RACLIB)/radio_drivers/sx126x_driver/src/lr_fhss_mac.c \
  $(USPZ)/modules/smtc_modem_hal/smtc_modem_hal.c $(USPZ)/drivers/usp/sx126x/sx126x_hal.c $(USPZ)/drivers/usp/sx126x/sx126x_ral_bsp.c \
  $(HERE)/hal/hal_stubs.c
ifeq (,$(findstring opt2,$(VARIANT)))
  C_SRCS += $(USPZ)/subsys/usp/zephyr_usp_thread.c $(USPZ)/subsys/usp/zephyr_usp_initialization.c
else
  C_SRCS += $(USPZ)/subsys/usp/zephyr_usp_initialization.c
endif

C_OBJS := $(patsubst %.c,$(BUILD)/c/%.o,$(notdir $(C_SRCS)))
vpath %.c $(sort $(dir $(C_SRCS)))

$(BUILD)/c/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(if $(filter zephyr_usp_thread.o,$(notdir $@)),-include $(HERE)/hal/thread_loop_cost.h) -c $< -o $@

$(BUILD)/libusp.a: $(C_OBJS)
	@rm -f $@; ar rcs $@ $^

lib: $(BUILD)/libusp.a
.PHONY: lib

# ---- C++ side ----
OPT := $(if $(findstring opt2,$(VARIANT)),2,$(if $(findstring opt3,$(VARIANT)),3,1))
CXX_COMMON := $(BUILD)/zsim.o $(BUILD)/chip.o $(BUILD)/rig.o $(BUILD)/ProfilePolicy.o
$(BUILD)/zsim.o: $(HERE)/zsim/zsim.cpp ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -c $< -o $@
$(BUILD)/chip.o: $(HERE)/chip/sx126x_chip.cpp ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -c $< -o $@
$(BUILD)/rig.o: $(HERE)/scen/rig.cpp $(HERE)/scen/rig.hpp ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -DOPT=$(OPT) -c $< -o $@
$(BUILD)/ProfilePolicy.o: $(UR)/ProfilePolicy.cpp ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -c $< -o $@

ifeq ($(OPT),2)
SESSION_OBJS := $(BUILD)/UspRadioSt.o $(BUILD)/RalSessionSt.o
$(BUILD)/UspRadioSt.o: $(HERE)/opt2/UspRadioSt.cpp $(HERE)/opt2/UspRadioSt.hpp ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -c $< -o $@
$(BUILD)/RalSessionSt.o: $(HERE)/opt2/RalSessionSt.cpp $(HERE)/opt2/RalSessionSt.hpp $(HERE)/opt2/apply_params.inc ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -include $(HERE)/hal/fault_inject.h -c $< -o $@
else
SESSION_OBJS := $(BUILD)/UspRadio.o $(BUILD)/RalSessionImpl.o
USPRADIO_SRC := $(UR)/UspRadio.cpp
ifneq (,$(findstring epoch,$(VARIANT)))
# fix for claim A on options 1/3: an RX-epoch tag on ring slots (fixes/epoch_tag.diff)
USPRADIO_SRC := $(BUILD)/patched/fprime-zephyr/Drv/UspRadio/UspRadio.cpp
$(USPRADIO_SRC): $(UR)/UspRadio.cpp $(UR)/UspRadio.hpp $(HERE)/fixes/epoch_tag.diff
	@mkdir -p $(dir $@)
	cp $(UR)/UspRadio.cpp $(UR)/UspRadio.hpp $(dir $@)
	cd $(BUILD)/patched && patch -p1 < $(HERE)/fixes/epoch_tag.diff
endif
$(BUILD)/UspRadio.o: $(USPRADIO_SRC) Makefile ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -D__ZEPHYR__ -c $(USPRADIO_SRC) -o $@
$(BUILD)/rig.o $(BUILD)/scen_%.o: $(USPRADIO_SRC)
ifeq ($(OPT),3)
# Option 3 = the prototype's real diff applied to #26's RalSessionImpl.cpp (+ the hookState() declaration the diff header asks for)
PATCHED := $(BUILD)/patched/fprime-zephyr/Drv/UspRadio
$(PATCHED)/RalSessionImpl.cpp: $(UR)/RalSessionImpl.cpp $(UR)/RalSessionImpl.hpp $(PROTO)/upstream-mutex/RalSessionImpl.cpp.diff
	@mkdir -p $(PATCHED)
	cp $(UR)/RalSessionImpl.cpp $(UR)/RalSessionImpl.hpp $(PATCHED)/
	cd $(BUILD)/patched && patch -p1 < $(PROTO)/upstream-mutex/RalSessionImpl.cpp.diff
	sed -i.bak 's|    void quiesceRadio();|    void quiesceRadio();\n    rp_task_states_t hookState() const;|' $(PATCHED)/RalSessionImpl.hpp && rm -f $(PATCHED)/RalSessionImpl.hpp.bak
$(BUILD)/RalSessionImpl.o: $(PATCHED)/RalSessionImpl.cpp ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -include $(HERE)/hal/fault_inject.h -c $< -o $@
else
$(BUILD)/RalSessionImpl.o: $(UR)/RalSessionImpl.cpp ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -include $(HERE)/hal/fault_inject.h -c $< -o $@
endif
endif

$(BUILD)/scen_%.o: $(HERE)/scen/%.cpp $(HERE)/scen/rig.hpp ; @mkdir -p $(@D); $(CXX) $(CXXFLAGS) -DOPT=$(OPT) -c $< -o $@
$(BUILD)/%.bin: $(BUILD)/scen_%.o $(CXX_COMMON) $(SESSION_OBJS) $(BUILD)/libusp.a
	$(CXX) $(SAN) -g $(filter-out %.a,$^) -Wl,-force_load,$(BUILD)/libusp.a -o $@ -lpthread

BINS := smoke stale initfail race busyloop latency rearm blocked mutexwait txprof phase
bins: $(foreach b,$(BINS),$(BUILD)/$(b).bin)
.PHONY: bins

-include $(wildcard $(BUILD)/*.d $(BUILD)/c/*.d)
