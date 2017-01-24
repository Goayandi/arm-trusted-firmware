#ifndef _MT_L2C_H_
#define _MT_L2C_H_

#define CONFIGED_256		0x1
#define CONFIGED_512K		0x3
#define L2C_SIZE_CFG_OFF	8
#define L2C_SHARE_ENABLE	12

enum options {
	BORROW_L2,
	RETURN_L2,
	BORROW_NONE
};

void config_L2_size(void);

#endif
