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

internal void
ZeroRange8(u8* Start, u8* End)
{
    u8* Data = Start;
    while(Data != End)
    {
        *(Data++) = 0;
    }
}

internal void
ZeroMemory(void* Start, sz Count)
{
    ZeroRange8((u8*) Start, (u8*) Start + Count);
}

internal void
CopyRange8(u8* DstStart, u8* DstEnd, u8* SrcStart)
{
    u8* Dst = DstStart;
    u8* Src = SrcStart;
    while(Dst != DstEnd)
    {
        *(Dst++) = *(Src++);
    }
}

internal void
CopyMemory8(u8* DstStart, u8* SrcStart, sz Count)
{
    u8* DstEnd = (DstStart + Count);
    CopyRange8(DstStart, DstEnd, SrcStart);
}

internal void
CopyMemory(void* DstStart, void* SrcStart, sz Count)
{
    CopyMemory8(DstStart, SrcStart, Count);
}

typedef struct
{
    c8* Next;
    sz Elapsed;
} stream;

internal void
Printb(stream* Stream, c8* Data, sz Count)
{
    Assert(Stream->Elapsed >= Count);

    CopyMemory(Stream->Next, Data, Count);

    Stream->Next += Count;

    Stream->Elapsed -= Count;
}

internal void
Prints(stream* Stream, const c8* String)
{
    const c8* Str = String;
    c8* Next = Stream->Next;
    while(*Str)
    {
        *(Next++) = *(Str++);
    }

    // TODO: I know, i know, this is BUGGGGYYYY, but as far as we don't overflow stream, this will work
    Assert(Next <= (Stream->Next + Stream->Elapsed))
    Stream->Elapsed -= (Next - Stream->Next);

    Stream->Next = Next;
}

#define Printz(Stream, String) Printb(Stream, String, ArrayCount(String)-1)

internal void
Printx(stream* Stream, unsigned Number)
{
    c8 TmpData[16];

    Assert(Stream->Elapsed >= ArrayCount(TmpData));

    c8* Tmp = TmpData;
    *Tmp = '\0';

    do
    {
        unsigned Quotient = (Number / 16);
        unsigned Remainder = (Number - Quotient * 16);
        *(++Tmp) = (Remainder < 10) ? ('0' + Remainder) : ('A' + Remainder - 10);
        Number = Quotient;
    }
    while(Number);

    c8* Next = Stream->Next;
    *(Next++) = '0';
    *(Next++) = 'x';
    while(*Tmp)
    {
        *(Next++) = *(Tmp--);
    }

    Stream->Elapsed -= (Next - Stream->Next);
    Stream->Next = Next;
}

