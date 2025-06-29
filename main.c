#include "defaults.h"
#include <stdio.h>

int main(void)
{
    ths8200_regs_t regs;
    ths8200_set_defaults(&regs);
    ths8200_print_regs(&regs);
    return 0;
}
