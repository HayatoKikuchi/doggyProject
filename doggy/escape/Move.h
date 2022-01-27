#ifndef MOVE_H
#define MOVE_H

#include "Tool.h"
#include "LegController.h"

class Move
{
public:
    Move(LegController::posi = {0.0,0.0});

    coord getposi(void);


private:

    LegController leg[];
};

#endif
