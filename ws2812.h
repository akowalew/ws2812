#define ANIMATION_TYPES \
    _ANIMATION_TYPE(Manual) \
    _ANIMATION_TYPE(Fade) \
    _ANIMATION_TYPE(Snake) \
    _ANIMATION_TYPE(Ekans) \

typedef enum
{
    #define _ANIMATION_TYPE(X) Animation_ ## X,
        ANIMATION_TYPES
    #undef _ANIMATION_TYPE
} animation_type;

#define _ANIMATION_TYPE(X) + 1
enum
{
    ANIMATIONS_TYPES_COUNT = (0 ANIMATION_TYPES)
};
#undef _ANIMATION_TYPE

internal const char*
AnimationTypeToString(animation_type Type)
{
    const char* Result = 0;

    switch(Type)
    {
        #define _ANIMATION_TYPE(X) case Animation_ ## X: Result = #X; break;
            ANIMATION_TYPES
        #undef _ANIMATION_TYPE
    }

    return Result;
}
