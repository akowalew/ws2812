#include "ws2812.h"

#include "fadeintable.c"

#define PIXELS_MAX 128

private_global u8 Buffer[3 * PIXELS_MAX];

private_global u8 Manual_G = 10;
private_global u8 Manual_R = 20;
private_global u8 Manual_B = 30;

internal void
AnimationUpdate_Manual()
{
    ZeroMemory(Buffer, sizeof(Buffer));

    Buffer[3*0 + 0] = Manual_G;
    Buffer[3*0 + 1] = Manual_R;
    Buffer[3*0 + 2] = Manual_B;

    Buffer[3*1 + 0] = 0;
    Buffer[3*1 + 1] = 0;
    Buffer[3*1 + 2] = Manual_B;

    Buffer[3*2 + 0] = 0;
    Buffer[3*2 + 1] = Manual_R;
    Buffer[3*2 + 2] = 0;

    Buffer[3*3 + 0] = Manual_G;
    Buffer[3*3 + 1] = 0;
    Buffer[3*3 + 2] = 0;

    Buffer[3*4 + 0] = Manual_G;
    Buffer[3*4 + 1] = 0;
    Buffer[3*4 + 2] = Manual_B;

    Buffer[3*5 + 0] = 0;
    Buffer[3*5 + 1] = Manual_R;
    Buffer[3*5 + 2] = Manual_B;

    Buffer[3*6 + 0] = Manual_G;
    Buffer[3*6 + 1] = Manual_R;
    Buffer[3*6 + 2] = 0;

    Buffer[3*7 + 0] = 0;
    Buffer[3*7 + 1] = 0;
    Buffer[3*7 + 2] = 0;
}

private_global sz  Fade_GIdx = 0;
private_global sz  Fade_RIdx = 20;
private_global sz  Fade_BIdx = 40;
private_global int Fade_GDelta = 1;
private_global int Fade_RDelta = -1;
private_global int Fade_BDelta = 1;

internal void
AnimationUpdate_Fade()
{
    ZeroMemory(Buffer, sizeof(Buffer));

    Buffer[0] = FadeInTable[Fade_GIdx];
    Buffer[1] = FadeInTable[Fade_RIdx];
    Buffer[2] = FadeInTable[Fade_BIdx];

    if(Fade_GIdx == ArrayCount(FadeInTable)-1)
    {
        Fade_GDelta = -1;
    }
    else if(Fade_GIdx == 0)
    {
        Fade_GDelta = 1;
    }

    if(Fade_RIdx == ArrayCount(FadeInTable)-1)
    {
        Fade_RDelta = -1;
    }
    else if(Fade_RIdx == 0)
    {
        Fade_RDelta = 1;
    }

    if(Fade_BIdx == ArrayCount(FadeInTable)-1)
    {
        Fade_BDelta = -1;
    }
    else if(Fade_BIdx == 0)
    {
        Fade_BDelta = 1;
    }

    Fade_GIdx += Fade_GDelta;
    Fade_RIdx += Fade_RDelta;
    Fade_BIdx += Fade_BDelta;
}

private_global sz PixelsCount = 8;

private_global sz Snake_Counter;
private_global sz Snake_Tail;
private_global sz Snake_Length = 3;

internal void
AnimationUpdate_Snake()
{
    Snake_Counter++;
    if(Snake_Counter == 10)
    {
        Snake_Counter = 0;

        ZeroMemory(Buffer, sizeof(Buffer));

        sz Pos = Snake_Tail;
        for(sz Idx = 0;
            Idx < Snake_Length;
            Idx++)
        {
            Assert(Pos < PixelsCount);
            Buffer[Pos*3 + 0] = 15;
            Buffer[Pos*3 + 1] = 0;
            Buffer[Pos*3 + 2] = 0;

            Pos++;
            if(Pos >= PixelsCount)
            {
                Pos = 0;
            }
        }

        Snake_Tail++;
        if(Snake_Tail >= PixelsCount)
        {
            Snake_Tail = 0;
        }
    }
}

internal void
AnimationUpdate_Ekans()
{
    Snake_Counter++;
    if(Snake_Counter == 10)
    {
        Snake_Counter = 0;

        ZeroMemory(Buffer, sizeof(Buffer));

        sz Pos = Snake_Tail;
        for(sz Idx = 0;
            Idx < Snake_Length;
            Idx++)
        {
            Assert(Pos < PixelsCount);
            Buffer[Pos*3 + 0] = 15;
            Buffer[Pos*3 + 1] = 0;
            Buffer[Pos*3 + 2] = 0;

            Pos++;
            if(Pos >= PixelsCount)
            {
                Pos = 0;
            }
        }

        if(Snake_Tail == 0)
        {
            Snake_Tail = PixelsCount;
        }

        Snake_Tail--;
    }
}

animation_type AnimationType;

internal void
SwitchToNextAnimationType()
{
    animation_type NextAnimationType = (AnimationType + 1);
    if(NextAnimationType >= (animation_type)ANIMATIONS_TYPES_COUNT)
    {
        NextAnimationType = (animation_type) 0;
    }

    AnimationType = NextAnimationType;
}

internal void
Animation_Update()
{
    switch(AnimationType)
    {
        case Animation_Manual: AnimationUpdate_Manual(); break;
        case Animation_Fade: AnimationUpdate_Fade(); break;
        case Animation_Snake: AnimationUpdate_Snake(); break;
        case Animation_Ekans: AnimationUpdate_Ekans(); break;
    }
}

internal void
Terminal_Update(u8 RxByte, stream* TxStream)
{
    c8 RxChar = ToLower(RxByte);
    switch(RxChar)
    {
        case 'q': SaturateIncrementU8(&Manual_G); break;
        case 'a': SaturateDecrementU8(&Manual_G); break;

        case 'w': SaturateIncrementU8(&Manual_R); break;
        case 's': SaturateDecrementU8(&Manual_R); break;

        case 'e': SaturateIncrementU8(&Manual_B); break;
        case 'd': SaturateDecrementU8(&Manual_B); break;

        case ' ': SwitchToNextAnimationType(); break;
    }

    Printz(TxStream, "\033[2J");
    Printz(TxStream, "\033[H");
    Printz(TxStream, "Animation type: ");
    Prints(TxStream, AnimationTypeToString(AnimationType));
    Printz(TxStream, "\r\nGreen: ");
    Printx(TxStream, Buffer[0]);
    Printz(TxStream, "\r\nRed:   ");
    Printx(TxStream, Buffer[1]);
    Printz(TxStream, "\r\nBlue:  ");
    Printx(TxStream, Buffer[2]);
}

private_global u32 BUTTON_Pressed;
private_global u32 BUTTON_Value;

private_global volatile u32 BUTTON_Release_Head;
private_global volatile u32 BUTTON_Press_Head;

internal void
BUTTON_Update(b32 Status)
{
    BUTTON_Value >>= 1;
    if(Status)
    {
        BUTTON_Value |= 0x80;
    }

    if(BUTTON_Value == 0)
    {
        if(!BUTTON_Pressed)
        {
            BUTTON_Pressed = 1;

            BUTTON_Press_Head++;
        }
    }
    else if(BUTTON_Value == 0xFF)
    {
        if(BUTTON_Pressed)
        {
            BUTTON_Pressed = 0;

            BUTTON_Release_Head++;
        }
    }
}

u32 BUTTON_Press_Tail = 0;

internal void
BUTTON_Handle()
{
    if(BUTTON_Press_Head != BUTTON_Press_Tail)
    {
        BUTTON_Press_Tail = BUTTON_Press_Head;

        SwitchToNextAnimationType();
    }
}
