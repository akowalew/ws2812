#include <stdio.h>
#include <math.h>

int main(int Argc, char** Argv)
{
    unsigned Count = 50;

    float K = (32 / log((Count / 64.0f) + 1));

    float Table[Count];

    for(unsigned Idx = 0;
        Idx < Count;
        Idx++)
    {
        Table[Idx] = K * log((Idx / 64.0f) + 1);
    }

    printf("private_global u8 FadeInTable[] =\n");
    printf("{\n");

    for(unsigned Y = 0;
        Y < (Count / 16);
        Y++)
    {
        printf("    ");
        for(unsigned X = 0;
            X < 16;
            X++)
        {
            printf("%3u, ", (unsigned)Table[Y*16 + X]);
        }

        printf("\n");
    }

    printf("};\n");
}
