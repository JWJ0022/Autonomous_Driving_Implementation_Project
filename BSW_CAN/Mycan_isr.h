#ifndef MYCAN_ISR_H_
#define MYCAN_ISR_H_

#include <BSW_CAN/Mycan_init.h>

/* ISR handlers */
void canIsrTxHandler(void);
void canIsrRxHandler(void);


#endif /* MYCAN_ISR_H_ */
