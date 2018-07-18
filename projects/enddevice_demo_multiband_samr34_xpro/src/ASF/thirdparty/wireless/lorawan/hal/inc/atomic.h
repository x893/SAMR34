/************************************************************************/
/*      DECIDE ON THE LICENSE TEXT                                      */
/************************************************************************/

#ifndef ATOMIC_H
#define ATOMIC_H

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#ifdef UT
#include "lora_test_main.h"
#endif

/************************************************************************/
/* Function Prototypes                                                  */
/************************************************************************/
/**
 * \brief Enters a critical section.
 *
 * Disables global interrupts. To support nested critical sections, an internal
 * count of the critical section nesting will be kept, so that global interrupts
 * are only re-enabled upon leaving the outermost nested critical section.
 *
 */
void system_enter_critical_section(void);

/**
 * \brief Leaves a critical section.
 *
 * Enables global interrupts. To support nested critical sections, an internal
 * count of the critical section nesting will be kept, so that global interrupts
 * are only re-enabled upon leaving the outermost nested critical section.
 *
 */
void system_leave_critical_section(void);

/************************************************************************/
/* Defines                                                              */
/************************************************************************/
#ifdef UT
#define   ATOMIC_SECTION_ENTER
#define   ATOMIC_SECTION_EXIT  
#else
#define   ATOMIC_SECTION_ENTER  system_enter_critical_section();
#define   ATOMIC_SECTION_EXIT   system_leave_critical_section();
#endif /* UT */
#endif /* ATOMIC_H */

/* eof atomic.h */
