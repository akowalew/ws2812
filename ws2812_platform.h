typedef unsigned char u8;
typedef signed char i8;

typedef unsigned short u16;
typedef signed short i16;

typedef unsigned int u32;
typedef signed int i32;

typedef unsigned int sz;

typedef u32 b32;

typedef char c8;

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

internal c8
ToLower(c8 Char)
{
    c8 Result = 0;

    if((Char >= 'A') &&
       (Char <= 'Z'))
    {
        Result = ('a' + (Char - 'A'));
    }
    else
    {
        Result = Char;
    }

    return Result;
}

internal void
SaturateIncrementU8(u8* Value)
{
    if(*Value < 255)
    {
        *Value = *Value + 1;
    }
}

internal void
SaturateDecrementU8(u8* Value)
{
    if(*Value > 0)
    {
        *Value = *Value - 1;
    }
}
