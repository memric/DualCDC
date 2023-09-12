/*
 * TRACE.h
 *
 *  Created on: 28.10.2020
 *      Author: Valeriy Chudnikov
 */

#include <stdio.h>

#ifndef INC_TRACE_H_
#define INC_TRACE_H_

#define TRACE_LEVEL_DISABLE		0
#define TRACE_LEVEL_ERROR		1
#define TRACE_LEVEL_WARNING		2
#define TRACE_LEVEL_INFO		3
#define TRACE_LEVEL_DEBUG		4

#ifndef TRACE_OUT
#define TRACE_OUT				printf
#endif

/*Default TRACE level*/
#ifndef TRACE_LEVEL
#define TRACE_LEVEL				TRACE_LEVEL_DEBUG
#endif

#ifndef TRACE_USE_ESC
#define TRACE_USE_ESC			1
#endif

#ifdef USE_SEGGER_SYSVIEW
#define SEGGER_TRACE			SEGGER_SYSVIEW_PrintfHost
#define SEGGER_TRACE_ERR		SEGGER_SYSVIEW_ErrorfHost
#define SEGGER_TRACE_WRN		SEGGER_SYSVIEW_WarnfHost
#define SEGGER_TRACE_IRQ		SEGGER_SYSVIEW_PrintfHost
#else
#define SEGGER_TRACE(msg...)
#define SEGGER_TRACE_ERR(msg...)
#define SEGGER_TRACE_WRN(msg...)
#define SEGGER_TRACE_IRQ(msg...)
#endif /*USE_SEGGER_SYSVIEW*/

#if TRACE_USE_ESC
#define TRACE_BLACK()			TRACE_OUT("\e[30m")
#define TRACE_RED()				TRACE_OUT("\e[31m")
#define TRACE_GREEN()			TRACE_OUT("\e[32m")
#define TRACE_YELLOW()			TRACE_OUT("\e[33m")
#define TRACE_BLUE()			TRACE_OUT("\e[34m")
#define TRACE_MAGENTA()			TRACE_OUT("\e[35m")
#define TRACE_CYAN()			TRACE_OUT("\e[36m")
#define TRACE_WHITE()			TRACE_OUT("\e[37m")
#define TRACE_BG_BLACK()		TRACE_OUT("\e[40m")
#define TRACE_BG_RED()			TRACE_OUT("\e[41m")
#define TRACE_BG_GREEN()		TRACE_OUT("\e[42m")
#define TRACE_BG_YELLOW()		TRACE_OUT("\e[43m")
#define TRACE_BG_BLUE()			TRACE_OUT("\e[44m")
#define TRACE_BG_MAGENTA()		TRACE_OUT("\e[45m")
#define TRACE_BG_CYAN()			TRACE_OUT("\e[46m")
#define TRACE_BG_WHITE()		TRACE_OUT("\e[47m")
#define TRACE_ERASE_ALL()		TRACE_OUT("\e[2J")
#define TRACE_HOME()			TRACE_OUT("\e[f")
#define TRACE_CURSOR_DIS()		TRACE_OUT("\e[?25l")
#define TRACE_ATTR_OFF()		TRACE_OUT("\e[0m")
#define TRACE_BOLD()			TRACE_OUT("\e[1m")
#define TRACE_BLINK()			TRACE_OUT("\e[5m")
#else
#define TRACE_BLACK()
#define TRACE_RED()
#define TRACE_GREEN()
#define TRACE_YELLOW()
#define TRACE_BLUE()
#define TRACE_MAGENTA()
#define TRACE_CYAN()
#define TRACE_WHITE()
#define TRACE_BG_BLACK()
#define TRACE_BG_RED()
#define TRACE_BG_GREEN()
#define TRACE_BG_YELLOW()
#define TRACE_BG_BLUE()
#define TRACE_BG_MAGENTA()
#define TRACE_BG_CYAN()
#define TRACE_BG_WHITE()
#define TRACE_ERASE_ALL()
#define TRACE_HOME()
#define TRACE_CURSOR_DIS()
#define TRACE_ATTR_OFF()
#define TRACE_BOLD()
#define TRACE_BLINK()
#endif

#if TRACE_LEVEL > TRACE_LEVEL_DISABLE
#define TRACE_ERR(msg...)		SEGGER_TRACE_ERR(msg); TRACE_RED(); TRACE_OUT(msg); TRACE_ATTR_OFF()
#define TRACE_OK(msg...)		SEGGER_TRACE(msg); TRACE_GREEN(); TRACE_OUT(msg); TRACE_ATTR_OFF()
#else
#define TRACE_ERR(msg...)
#define TRACE_OK(msg...)
#endif /*TRACE_LEVEL > TRACE_LEVEL_DISABLE*/
#if TRACE_LEVEL > TRACE_LEVEL_ERROR
#define TRACE_WRN(msg...)		SEGGER_TRACE_WRN(msg); TRACE_YELLOW(); TRACE_OUT(msg); TRACE_ATTR_OFF()
#else
#define TRACE_WRN(msg...)
#endif /*TRACE_LEVEL > TRACE_LEVEL_ERROR*/
#if TRACE_LEVEL > TRACE_LEVEL_WARNING
#define TRACE_INFO(msg...)		SEGGER_TRACE(msg); TRACE_OUT(msg)
#else
#define TRACE_INFO(msg...)
#endif /*TRACE_LEVEL > TRACE_LEVEL_WARNING*/
#if TRACE_LEVEL > TRACE_LEVEL_INFO
#define TRACE_DEBUG(msg...)		SEGGER_TRACE(msg); TRACE_OUT(msg)
#else
#define TRACE_DEBUG(msg...)
#endif /*TRACE_LEVEL > TRACE_LEVEL_INFO*/

#define TRACE					TRACE_INFO
#define TRACE_IRQ				SEGGER_TRACE_IRQ

#endif /* INC_TRACE_H_ */
