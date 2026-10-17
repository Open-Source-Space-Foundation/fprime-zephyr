# #589 host harness.  `make -C sim all` builds every variant; ../run.sh runs them.
#
# Real sources compiled here (read-only, never copied):
#   usp v1.0.0 fork (as pinned by PCR): radio_planner.c, smtc_rac.c
#   usp v1.2.1 (upstream tag, cloned to scratch): radio_planner.c, smtc_rac.c
#   fprime-zephyr @ arena/589-c1: UspRadio.cpp, RalSessionImpl.cpp, ProfilePolicy.cpp
#   prototype: single-queue/RalSessionSt.cpp, upstream-mutex/RalSessionImpl.cpp.diff
# Fakes (hardware/OS edge only): zephyr kernel on pthreads, SX1262 behind ral_drv_t,
# usp_zephyr HAL, F´ component base queue.

HERE      := $(abspath .)
WT        := $(abspath ../../../../../..)
UR        := $(WT)/fprime-zephyr/Drv/UspRadio
PROTO     := $(UR)/PROTOTYPE-589-concurrency
USP100    ?= $(HOME)/GitHut/pcr-588/lib/zephyr-workspace/modules/lib/usp/smtc_rac_lib
USP121    ?= $(HOME)/GitHut/arena-589/c1-scratch/usp-v1.2.1/smtc_rac_lib
B         ?= $(HOME)/GitHut/arena-589/c1-scratch/build

CC  := clang
CXX := clang++
SAN ?=
CFLAGS_COMMON := -g -O1 -DSX126X -Ifakes -Isrc $(SAN)
uspinc = -I$(1)/smtc_rac -I$(1)/smtc_rac_api -I$(1)/radio_planner/src -I$(1)/smtc_ral/src \
         -I$(1)/smtc_ralf/src -I$(1)/smtc_modem_hal
CXXINC := -I$(WT) -DLINK_PROFILES_USE_HOST_TYPES_OFF

# variant -> (usp tree, hal defines)
V_opt1     := $(USP100) -DFORK_RAC_MUTEX
V_opt1nomx := $(USP100)
V_opt3     := $(USP100) -DCONFIG_USP_THREADS_MUTEXES -DCONFIG_USP_MAIN_THREAD
V_opt1v121 := $(USP121) -DFORK_RAC_MUTEX
V_opt2     := $(USP100) -DFORK_RAC_MUTEX
V_opt2v121 := $(USP121) -DFORK_RAC_MUTEX
V_opt1fix  := $(USP100) -DFORK_RAC_MUTEX
# opt1fix = option 1 + ../fix-stale-epoch.diff, applied to a copy in scratch
FIX       ?= $(HOME)/GitHut/arena-589/c1-scratch/fixwt

VARIANTS := opt1 opt1nomx opt3 opt1v121 opt2 opt2v121 opt1fix

all: $(addprefix $(B)/sim_,$(VARIANTS))
tsan: $(addprefix $(B)/tsan_sim_,opt1 opt1nomx opt3 opt2)

define VARIANT
$(B)/$(1)/%.o: ; @true
$(B)/$(2)sim_$(1): src/*.c src/*.cpp src/*.h fakes/*.h fakes/*/*.h fakes/*/*/*.h fakes/*/*/*/*/*.h $(B)/RalSessionImpl_opt3.cpp $(B)/RalSessionSt_fixed.cpp
	@mkdir -p $(B)/$(2)$(1)
	$(CC) -std=gnu11 $(CFLAGS_COMMON) $(3) $(call uspinc,$(word 1,$(V_$(1)))) $(wordlist 2,9,$(V_$(1))) \
	  -c $(word 1,$(V_$(1)))/smtc_rac/smtc_rac.c -o $(B)/$(2)$(1)/smtc_rac.o
	$(CC) -std=gnu11 $(CFLAGS_COMMON) $(3) $(call uspinc,$(word 1,$(V_$(1)))) $(wordlist 2,9,$(V_$(1))) \
	  -c $(word 1,$(V_$(1)))/radio_planner/src/radio_planner.c -o $(B)/$(2)$(1)/radio_planner.o
	$(CC) -std=gnu11 $(CFLAGS_COMMON) $(3) $(call uspinc,$(word 1,$(V_$(1)))) $(wordlist 2,9,$(V_$(1))) \
	  -c src/chip_fake.c -o $(B)/$(2)$(1)/chip_fake.o
	$(CC) -std=gnu11 $(CFLAGS_COMMON) $(3) $(call uspinc,$(word 1,$(V_$(1)))) $(wordlist 2,9,$(V_$(1))) \
	  -c src/hal_fake.c -o $(B)/$(2)$(1)/hal_fake.o
	$(CC) -std=gnu11 $(CFLAGS_COMMON) $(3) $(call uspinc,$(word 1,$(V_$(1)))) \
	  -c src/usp_stubs.c -o $(B)/$(2)$(1)/usp_stubs.o
	$(CC) -std=gnu11 $(CFLAGS_COMMON) $(3) -c src/kernel_fake.c -o $(B)/$(2)$(1)/kernel_fake.o
	$(CXX) -std=gnu++17 $(CFLAGS_COMMON) $(3) $(call uspinc,$(word 1,$(V_$(1)))) $(wordlist 2,9,$(V_$(1))) \
	  -DCONFIG_LORA_BASICS_MODEM_DRIVERS -D__ZEPHYR__ -DVARIANT_$(1) \
	  $(if $(filter opt3,$(1)),-I$(B)/opt3patch,) $(if $(filter opt1fix,$(1)),-I$(FIX),) -I$(WT) -I$(PROTO)/single-queue \
	  src/component_base.cpp src/scenarios.cpp $(UR)/ProfilePolicy.cpp \
	  $(if $(filter opt1fix,$(1)),$(FIX)/fprime-zephyr/Drv/UspRadio/UspRadio.cpp,$(UR)/UspRadio.cpp) \
	  $(if $(filter opt3,$(1)),$(B)/RalSessionImpl_opt3.cpp,$(if $(filter opt1fix,$(1)),$(FIX)/fprime-zephyr/Drv/UspRadio/RalSessionImpl.cpp,$(UR)/RalSessionImpl.cpp)) \
	  $(if $(filter opt2%,$(1)),src/ralsession_st_glue.cpp $(B)/RalSessionSt_fixed.cpp,) \
	  $(B)/$(2)$(1)/*.o -o $$@ -lpthread
endef

$(eval $(call VARIANT,opt1,,))
$(eval $(call VARIANT,opt1nomx,,))
$(eval $(call VARIANT,opt3,,))
$(eval $(call VARIANT,opt1v121,,))
$(eval $(call VARIANT,opt2,,))
$(eval $(call VARIANT,opt2v121,,))
$(eval $(call VARIANT,opt1fix,,))

# Recreate the opt1fix tree from the committed diff (idempotent).
$(FIX)/fprime-zephyr/Drv/UspRadio/UspRadio.cpp: ../fix-stale-epoch.diff $(UR)/UspRadio.cpp $(UR)/RalSessionImpl.cpp
	rm -rf $(FIX) && mkdir -p $(FIX)/fprime-zephyr/Drv/UspRadio
	cp $(UR)/UspRadio.cpp $(UR)/UspRadio.hpp $(UR)/RalSession.hpp $(UR)/RalSessionImpl.cpp $(UR)/RalSessionImpl.hpp \
	  $(FIX)/fprime-zephyr/Drv/UspRadio/
	cd $(FIX) && patch -p1 -s < $(HERE)/../fix-stale-epoch.diff
$(B)/sim_opt1fix: $(FIX)/fprime-zephyr/Drv/UspRadio/UspRadio.cpp
$(eval $(call VARIANT,opt1,tsan_,-fsanitize=thread))
$(eval $(call VARIANT,opt1nomx,tsan_,-fsanitize=thread))
$(eval $(call VARIANT,opt3,tsan_,-fsanitize=thread))
$(eval $(call VARIANT,opt2,tsan_,-fsanitize=thread))

# Option 2 sketch, with the two compile fixes it needs (both reported in RESULTS.md):
#   missing #include <zephyr/lorawan_lbm/lorawan_hal_init.h> (smtc_modem_hal_get_event_sem)
#   rp_disable_failsafe() takes (rp, bool); the sketch passes one argument
$(B)/RalSessionSt_fixed.cpp: $(PROTO)/single-queue/RalSessionSt.cpp
	@mkdir -p $(B)
	( echo '#include <zephyr/lorawan_lbm/lorawan_hal_init.h>'; \
	  sed 's/rp_disable_failsafe(smtc_rac_get_rp())/rp_disable_failsafe(smtc_rac_get_rp(), true)/' $< ) > $@

# Option 3 = #26 RalSessionImpl.cpp + the prototype diff, applied in scratch.
$(B)/RalSessionImpl_opt3.cpp: $(UR)/RalSessionImpl.cpp $(PROTO)/upstream-mutex/RalSessionImpl.cpp.diff
	@mkdir -p $(B)/opt3patch/fprime-zephyr/Drv/UspRadio
	cp $(UR)/RalSessionImpl.cpp $(B)/opt3patch/fprime-zephyr/Drv/UspRadio/
	cd $(B)/opt3patch && patch -p1 -s < $(PROTO)/upstream-mutex/RalSessionImpl.cpp.diff
	cp $(UR)/RalSessionImpl.hpp $(B)/opt3patch/fprime-zephyr/Drv/UspRadio/
	sed -i '' 's/^    void quiesceRadio();/    void quiesceRadio();\n    rp_task_states_t hookState() const;  \/\/ per the diff header note/' \
	  $(B)/opt3patch/fprime-zephyr/Drv/UspRadio/RalSessionImpl.hpp
	cp $(B)/opt3patch/fprime-zephyr/Drv/UspRadio/RalSessionImpl.cpp $@

clean:
	rm -rf $(B)

.PHONY: all clean
