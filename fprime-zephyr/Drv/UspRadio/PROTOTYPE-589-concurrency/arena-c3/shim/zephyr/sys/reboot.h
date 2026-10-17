#include <zephyr/kernel.h>
#define SYS_REBOOT_COLD 1
#ifdef __cplusplus
extern "C" {
#endif
void sys_reboot(int type);
#ifdef __cplusplus
}
#endif
