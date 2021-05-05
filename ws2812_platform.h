typedef unsigned char u8;
typedef signed char i8;

typedef unsigned short u16;
typedef signed short i16;

typedef unsigned int u32;
typedef signed int i32;

typedef unsigned int sz;

typedef u32 b32;

#define internal static
#define private_global static
#define public_global
#define persist static

#define noreturn __attribute__((noreturn))

#define BreakPoint() __asm volatile ("bkpt")
#define NoOperation() __asm volatile ("nop")

#define in_section(x) __attribute__((section(x), used))

#define Assert(x) if(!(x)) { while(1) { BreakPoint(); } }
#define InvalidCodePath() Assert(!"InvalidCodePath")

#define ArrayCount(x) (sizeof(x)/sizeof(x[0]))
