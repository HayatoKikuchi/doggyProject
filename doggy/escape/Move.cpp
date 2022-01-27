#include "Move.h"

Move Move(LegController _leg[], int leg_num)
{
    for (int i = 0; i < leg_num; i++)
    {
        leg[i] = _leg[i];
    }
}

coord Move::getposi(void)
{
    return posi;
}