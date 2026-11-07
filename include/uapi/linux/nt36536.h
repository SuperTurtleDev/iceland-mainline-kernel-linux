/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_NT36536_H
#define _UAPI_LINUX_NT36536_H

#define NVT_GENL_NAME "nt36536"
#define NVT_GENL_VERSION 1
#define NVT_GENL_GROUP "events"

/* Undecoded frames: raw event-buffer bytes, excluding the SPI address byte. */
#define NVT_CONTROL_FRAME_SIZE 300
/* OEM HOST_CMD transport limit, excluding the SPI address byte. */
#define NVT_CONTROL_CMD_MAX 15

/*
 * Driver-local downlinks carried by NVT_CMD_SEND, never sent over SPI.
 * Bytes 0..3: "NVT\0", byte 4: opcode, remaining bytes: payload below.
 * Integers are little-endian. Unknown local opcodes are rejected.
 */
#define NVT_PEN_LOCAL_PREFIX "NVT\0"
#define NVT_PEN_LOCAL_PREFIX_LEN 4
#define NVT_PEN_LOCAL_PRESSURE 1 /* u16 pressure, 0..16384 */
#define NVT_PEN_LOCAL_BUTTONS 2 /* u8 bitmap: bit 0 STYLUS, bit 1 STYLUS2 */
#define NVT_PEN_LOCAL_SOURCES 3 /* u8 bitmap: 0 = IC, 1 = userspace per field */
#define NVT_PEN_SOURCE_PRESSURE (1U << 0)
#define NVT_PEN_SOURCE_STYLUS (1U << 1)
#define NVT_PEN_SOURCE_STYLUS2 (1U << 2)
#define NVT_PEN_SOURCE_MASK 7U

enum nvt_genl_command {
	NVT_CMD_UNSPEC,
	NVT_CMD_SEND,
	NVT_CMD_EVENT,
};

enum nvt_genl_attribute {
	NVT_ATTR_UNSPEC,
	NVT_ATTR_DATA,
	NVT_ATTR_DEVICE,
	__NVT_ATTR_MAX,
};

#define NVT_ATTR_MAX (__NVT_ATTR_MAX - 1)

#endif /* _UAPI_LINUX_NT36536_H */
