/*
 * cf_assert.h
 *
 *  Created on: Jan 5, 2026
 *      Author: SAIED
 */

#ifndef INC_CF_ASSERT_H_
#define INC_CF_ASSERT_H_

#define ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )


/**
 * Assert handler function
 */
void assertFail(char *exp, char *file, int line);

#endif /* INC_CF_ASSERT_H_ */
