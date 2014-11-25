#ifndef _PDA12_H
#define _PDA12_H

#include <asm/ioctl.h>

#define IOCTL_GET_REG0		_IOR('U', 0x00, uint32_t)
#define IOCTL_GET_REG1		_IOR('U', 0x01, uint32_t)
#define IOCTL_GET_REG2		_IOR('U', 0x02, uint32_t)
#define IOCTL_GET_REG3		_IOR('U', 0x03, uint32_t)
#define IOCTL_GET_REG4		_IOR('U', 0x04, uint32_t)
#define IOCTL_GET_REG5		_IOR('U', 0x05, uint32_t)
#define IOCTL_GET_REG6		_IOR('U', 0x06, uint32_t)
#define IOCTL_GET_REG7		_IOR('U', 0x07, uint32_t)

#define IOCTL_SET_REG0		_IOW('U', 0x00, uint32_t)
#define IOCTL_SET_REG1		_IOW('U', 0x01, uint32_t)
#define IOCTL_SET_REG2		_IOW('U', 0x02, uint32_t)
#define IOCTL_SET_REG3		_IOW('U', 0x03, uint32_t)
#define IOCTL_SET_REG4		_IOW('U', 0x04, uint32_t)
#define IOCTL_SET_REG5		_IOW('U', 0x05, uint32_t)
#define IOCTL_SET_REG6		_IOW('U', 0x06, uint32_t)
#define IOCTL_SET_REG7		_IOW('U', 0x07, uint32_t)

#endif /* _PDA12_H */
