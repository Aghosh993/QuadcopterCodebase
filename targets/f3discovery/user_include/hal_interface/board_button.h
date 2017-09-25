#ifndef BOARD_BUTTON_H_
#define BOARD_BUTTON_H_		1

#include <stdbool.h>
#include "hal_common_includes.h"

typedef enum {
	BTN_USER
} button;

void board_button_init(void);
bool board_button_read(button b);

#endif