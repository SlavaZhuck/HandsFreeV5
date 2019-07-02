/*
 * buttons.h
 *
 *  Created on: 19 апр. 2019 г.
 *      Author: CIT_007
 */

#ifndef APPLICATION_BUTTONS_H_
#define APPLICATION_BUTTONS_H_

#include "project_zero.h"
#define BUTTON_DEBOUNCE_TIME      30 /* ms */

void buttons_init (void);
void Handler_ButtonPress(pzButtonState_t *pState);
#endif /* APPLICATION_BUTTONS_H_ */
