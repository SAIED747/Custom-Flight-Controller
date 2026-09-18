/*
 * cf_assert.c
 *
 *  Created on: Jan 5, 2026
 *      Author: SAIED
 */
#include "main.h"

void assertFail(char *exp, char *file, int line)
{
  //portDISABLE_INTERRUPTS();
  //storeAssertFileData(file, line);
  printf("Assert failed %s:%d\n", file, line);

  //motorsStop();
  //ledShowFaultPattern();

//  if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
//  {
//    // Only reset if debugger is not connected
//    NVIC_SystemReset();
//  }
}
