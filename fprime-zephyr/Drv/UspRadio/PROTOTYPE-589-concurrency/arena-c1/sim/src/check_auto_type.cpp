static int lockc; static void lk(){lockc++;} static void ul(){lockc--;}
static int f(int x){return x+1;}
#define SMTC_SW_PLATFORM( call ) ( { lk(); __auto_type __result = ( call ); ul(); __result; } )
int g(int y){ return SMTC_SW_PLATFORM(f(y)); }
