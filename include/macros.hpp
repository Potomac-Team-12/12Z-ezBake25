#pragma once

struct Macro {
    const char* name;
    void (*function)();
};

void doNothing();
void runSelectedMacro(Macro macros[], size_t macroCount);
