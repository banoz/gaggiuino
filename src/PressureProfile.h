#ifndef PRESSURE_PROFILER_H
#define PRESSURE_PROFILER_H

#include "utils.h"

struct Phase
{
    float startPressure;
    float endPressure;
    unsigned int durationMs;

    float getPressure(unsigned long timeInPhase)
    {
        return mapRange(timeInPhase, 0, durationMs, startPressure, endPressure, 1);
    }
};

struct CurrentPhase
{
    short phaseIndex;
    long timeInPhase;
};

struct Phases
{
    short count;
    Phase *phases;

    CurrentPhase getCurrentPhase(long timeInPP)
    {
        short phase = 0;
        long accumulatedTime = 0L;

        while (phase < count - 1 && timeInPP >= accumulatedTime + (phases[phase].durationMs))
        {
            accumulatedTime += phases[phase].durationMs;
            phase += 1;
        }
        return CurrentPhase{phase, timeInPP - accumulatedTime};
    }
};

#endif
