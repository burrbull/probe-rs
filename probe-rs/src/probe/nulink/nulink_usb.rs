/***************************************************************************
 *   Copyright (C) 2016-2017 by Nuvoton                                    *
 *   Zale Yu <cyyu@nuvoton.com>                                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <helper/binarybuffer.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <target/target.h>

#include <target/cortex_m.h>

#include <hidapi.h>

pub enum NulinkError {
    Fail,
}

const NULINK_READ_TIMEOUT: i32 = 1000;

const NULINK_HID_MAX_SIZE: usize = 64;
const NULINK2_HID_MAX_SIZE: usize = 1024;
const V6M_MAX_COMMAND_LENGTH: usize = NULINK_HID_MAX_SIZE - 2;
const V7M_MAX_COMMAND_LENGTH: usize = NULINK_HID_MAX_SIZE - 3;

const NULINK2_USB_PID1: u16 = 0x5200;
const NULINK2_USB_PID2: u16 = 0x5201;

struct NulinkUsbHandle {
    dev_handle: hidapi::HidDevice,
    max_packet_size: u16,
    usbcmdidx: u8,
    cmdidx: u8,
    cmdsize: u8,
    cmdbuf: [u8; NULINK2_HID_MAX_SIZE + 1],
    tempbuf: [u8; NULINK2_HID_MAX_SIZE],
    databuf: [u8; NULINK2_HID_MAX_SIZE],
    max_mem_packet: u32,
    hardware_config: u16, /* bit 0: 1:Nu-Link-Pro, 0:Nu-Link */

    int (*xfer)(void *handle, uint8_t *buf, int size);
    void (*init_buffer)(void *handle, uint32_t size);
}

/* ICE Command */
pub mod commands {
    pub const READ_REG: u8 = 0xB5;
    pub const READ_RAM: u8 = 0xB1;
    pub const WRITE_REG: u8 = 0xB8;
    pub const WRITE_RAM: u8 = 0xB9;
    pub const CHECK_ID: u8 = 0xA3;
    pub const MCU_RESET: u8 = 0xE2;
    pub const CHECK_MCU_STOP: u8 = 0xD8;
    pub const MCU_STEP_RUN: u8 = 0xD1;
    pub const MCU_STOP_RUN: u8 = 0xD2;
    pub const MCU_FREE_RUN: u8 = 0xD3;
    pub const SET_CONFIG: u8 = 0xA2;
}

const ARM_SRAM_BASE: u32 = 0x20000000;

const HARDWARE_CONFIG_NULINKPRO: u16 = 1;
const HARDWARE_CONFIG_NULINK2: u16 = 2;

pub enum NulinkReset {
    Auto = 0,
    HW = 1,
    SysResetReq = 2,
    VectReset = 3,
    FastRescue = 4, /* Rescue and erase the chip, need very fast speed */
}

pub enum NulinkConnect {
    Normal = 0,      /* Support all reset method */
    PreReset = 1,   /* Support all reset method */
    UnderReset = 2, /* Support all reset method */
    None = 3,        /* Support RESET_HW, (RESET_AUTO = RESET_HW) */
    Disconnect = 4,  /* Support RESET_NONE, (RESET_AUTO = RESET_NONE) */
    IcpMode = 5     /* Support NUC505 ICP mode*/
}

// ???????/
pub enum TargetState {
    Unknown,
    Halted,
    Running,
}

impl NulinkUsbHandle {
	pub fn usb_xfer_rw(&self, buf: &mut [u8]) -> Result<(), NulinkError> {
		if hid_write(self.dev_handle, self.cmdbuf, h.max_packet_size + 1).is_err() {
			error!("hid_write");
			return Err(NulinkError::Fail);
		}

		if hid_read_timeout(self.dev_handle, buf[..h.max_packet_size], NULINK_READ_TIMEOUT).is_err() {
			Err(NulinkError::Fail)
		} else {
			Ok(())
		}
	}
}

pub fn nulink1_usb_xfer(handle: &NulinkUsbHandle, buf: &mut [u8]) -> Result<(), NulinkError> {
    handle.usb_xfer_rw(handle.tempbuf)?;

    memcpy(buf, h.tempbuf + 2, V6M_MAX_COMMAND_LENGTH);

    Ok(())
}

pub fn nulink2_usb_xfer(handle: &NulinkUsbHandle, buf: &mut [u8]) -> Result<(), NulinkError> {
    handle.usb_xfer_rw(handle.tempbuf)?;

    memcpy(buf, handle.tempbuf + 3, V7M_MAX_COMMAND_LENGTH);

    Ok(())
}

pub fn nulink1_usb_init_buffer(handle: &NulinkUsbHandle, size: usize) {
    handle.cmdidx = 0;

    memset(handle.cmdbuf, 0, h->max_packet_size + 1);
    memset(handle.tempbuf, 0, h->max_packet_size);
    memset(handle.databuf, 0, h->max_packet_size);

    handle.cmdbuf[0] = 0; /* report number */
    handle.usbcmdidx += 1;
    handle.cmdbuf[1] = handle.usbcmdidx & 0x7F;
    handle.cmdbuf[2] = size;
    handle.cmdidx += 3;
}

pub fn nulink2_usb_init_buffer(handle: &NulinkUsbHandle, size: usize) {
    handle.cmdidx = 0;

    memset(handle.cmdbuf, 0, handle.max_packet_size + 1);
    memset(handle.tempbuf, 0, handle.max_packet_size);
    memset(handle.databuf, 0, handle.max_packet_size);

    handle.cmdbuf[0] = 0; /* report number */
    handle.usbcmdidx += 1;
    handle.cmdbuf[1] = handle.usbcmdidx & 0x7F;
    h_u16_to_le(handle.cmdbuf + 2, size);
    handle.cmdidx += 4;
}

impl NulinkUsbHandle {
	pub fn usb_xfer(&self, buf: &mut [u8]) -> Result<(), NulinkError> {
		self.xfer(buf)
	}

	pub fn usb_init_buffer(&self, size: usize) {
		self.init_buffer(size);
	}

	pub fn usb_version(&self) -> Result<(), NulinkError> {
		debug!("nulink_usb_version");

		self.usb_init_buffer(V6M_MAX_COMMAND_LENGTH);

		memset(self.cmdbuf + self.cmdidx, 0xFF, V6M_MAX_COMMAND_LENGTH);
		self.cmdbuf[h->cmdidx + 4] = 0xA1; /* host_rev_num: 6561 */;
		self.cmdbuf[h->cmdidx + 5] = 0x19;

		self.usb_xfer(self.databuf, self.cmdsize)?;

		info!("Nu-Link firmware_version {}, product_id (0x{:08x})",
				 le_to_h_u32(self.databuf),
				 le_to_h_u32(self.databuf + 4 * 1));

		let is_nulinkpro = !!(le_to_h_u32(self.databuf + 4 * 2) & 1 != 0);
		if is_nulinkpro {
			info!("Adapter is Nu-Link-Pro, target_voltage_mv({}), usb_voltage_mv({})",
					 le_to_h_u16(self.databuf + 4 * 3 + 0),
					 le_to_h_u16(self.databuf + 4 * 3 + 2));

			self.hardware_config |= HARDWARE_CONFIG_NULINKPRO;
		} else {
			info!("Adapter is Nu-Link");
		}

		Ok(())
	}

	pub fn usb_idcode(&self) -> Result<u32, NulinkError> {
		debug!("nulink_usb_idcode");

		self.usb_init_buffer(4 * 1);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::CHECK_ID);
		self.cmdidx += 4;

		self.usb_xfer(self.databuf, 4 * 2)?;

		let idcode = le_to_h_u32(self.databuf + 4 * 1);

		info!("IDCODE: 0x{08x}", *idcode);

		Ok(idcode)
	}

	pub fn usb_write_debug_reg(&self, addr: u32, val: u32) -> Result<(), NulinkError> {
		debug!("nulink_usb_write_debug_reg 0x{:08x} 0x{:08x}", addr, val);

		self.usb_init_buffer(8 + 12 * 1);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::WRITE_RAM);
		self.cmdidx += 4;
		/* Count of registers */
		self.cmdbuf[self.cmdidx] = 1;
		self.cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		self.cmdbuf[self.cmdidx] = 0x00;
		self.cmdidx += 1;
		/* Array of bool value (u8Verify) */
		self.cmdbuf[h->cmdidx] = 0x00;
		self.cmdidx += 1;
		/* ignore */
		self.cmdbuf[h->cmdidx] = 0;
		self.cmdidx += 1;
		/* u32Addr */
		h_u32_to_le(self.cmdbuf + self.cmdidx, addr);
		self.cmdidx += 4;
		/* u32Data */
		h_u32_to_le(self.cmdbuf + self.cmdidx, val);
		self.cmdidx += 4;
		/* u32Mask */
		h_u32_to_le(self.cmdbuf + self.cmdidx, 0x00000000);
		self.cmdidx += 4;

		self.usb_xfer(self.databuf, 4 * 2)
	}

	pub fn usb_state(&self) -> TargetState {
		self.usb_init_buffer(4 * 1);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::CHECK_MCU_STOP);
		self.cmdidx += 4;

		if self.usb_xfer(self.databuf, 4 * 4).is_err() {
			TargetState::Unknown
		} else if (!le_to_h_u32(h->databuf + 4 * 2)) {
			TargetState::Halted
		} else {
			TargetState::Running
		}
	}

	pub fn usb_assert_srst(&self, int srst) -> Result<(), NulinkError> {
		debug!("nulink_usb_assert_srst");

		self.usb_init_buffer(4 * 4);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::MCU_RESET);
		self.cmdidx += 4;
		/* set reset type */
		h_u32_to_le(self.cmdbuf + self.cmdidx, RESET_SYSRESETREQ);
		self.cmdidx += 4;
		/* set connect type */
		h_u32_to_le(self.cmdbuf + self.cmdidx, CONNECT_NORMAL);
		self.cmdidx += 4;
		/* set extMode */
		h_u32_to_le(self.cmdbuf + self.cmdidx, 0);
		self.cmdidx += 4;

		self.usb_xfer(self.databuf, 4 * 4)
	}

	pub fn usb_reset(&self) -> Result<(), NulinkError> {
		debug!("nulink_usb_reset");

		self.usb_init_buffer(4 * 4);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::MCU_RESET);
		self.cmdidx += 4;
		/* set reset type */
		h_u32_to_le(self.cmdbuf + self.cmdidx, RESET_HW);
		self.cmdidx += 4;
		/* set connect type */
		h_u32_to_le(self.cmdbuf + self.cmdidx, CONNECT_NORMAL);
		self.cmdidx += 4;
		/* set extMode */
		h_u32_to_le(self.cmdbuf + self.cmdidx, 0);
		self.cmdidx += 4;

		self.usb_xfer(self.databuf, 4 * 4)
	}

	pub fn usb_run(&self) -> Result<(), NulinkError> {
		debug!("nulink_usb_run");

		self.usb_init_buffer(4 * 1);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::MCU_FREE_RUN);
		self.cmdidx += 4;

		self.usb_xfer(self.databuf, 4 * 4)
	}

	pub fn usb_halt(&self) -> Result<(), NulinkError> {
		debug!("nulink_usb_halt");

		self.usb_init_buffer(4 * 1);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::MCU_STOP_RUN);
		self.cmdidx += 4;

		let res = self.usb_xfer(self.databuf, 4 * 4)

		debug!("Nu-Link stop_pc 0x{:08x}", le_to_h_u32(self.databuf + 4));

		res
	}

	pub fn usb_step(&self) -> Result<(), NulinkError> {
		debug!("nulink_usb_step");

		self.usb_init_buffer(4 * 1);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::MCU_STEP_RUN);
		self.cmdidx += 4;

		let res = self.usb_xfer(self.databuf, 4 * 4)

		debug!("Nu-Link pc 0x{:08x}", le_to_h_u32(self.databuf + 4));

		res
	}

	pub fn usb_read_reg(&self, regsel: u32) -> Result<u32, NulinkError> {
		self.usb_init_buffer(8 + 12 * 1);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::WRITE_REG);
		self.cmdidx += 4;
		/* Count of registers */
		self.cmdbuf[self.cmdidx] = 1;
		self.cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		self.cmdbuf[self.cmdidx] = 0xFF;
		self.cmdidx += 1;
		/* Array of bool value (u8Verify) */
		self.cmdbuf[self.cmdidx] = 0x00;
		self.cmdidx += 1;
		/* ignore */
		self.cmdbuf[self.cmdidx] = 0;
		self.cmdidx += 1;
		/* u32Addr */
		h_u32_to_le(self.cmdbuf + self.cmdidx, regsel);
		self.cmdidx += 4;
		/* u32Data */
		h_u32_to_le(self.cmdbuf + self.cmdidx, 0);
		self.cmdidx += 4;
		/* u32Mask */
		h_u32_to_le(self.cmdbuf + self.cmdidx, 0xFFFFFFFF);
		self.cmdidx += 4;

		self.usb_xfer(self.databuf, 4 * 2)?;

		Ok(le_to_h_u32(self.databuf + 4 * 1))
	}

	pub fn usb_write_reg(&self, regsel: u32, val: u32) -> Result<(), NulinkError> {
		self.usb_init_buffer(8 + 12 * 1);
		/* set command ID */
		h_u32_to_le(self.cmdbuf + self.cmdidx, commands::WRITE_REG);
		self.cmdidx += 4;
		/* Count of registers */
		self.cmdbuf[self.cmdidx] = 1;
		self.cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		self.cmdbuf[self.cmdidx] = 0x00;
		self.cmdidx += 1;
		/* Array of bool value (u8Verify) */
		self.cmdbuf[self.cmdidx] = 0x00;
		self.cmdidx += 1;
		/* ignore */
		self.cmdbuf[self.cmdidx] = 0;
		self.cmdidx += 1;
		/* u32Addr */
		h_u32_to_le(self.cmdbuf + self.cmdidx, regsel);
		self.cmdidx += 4;
		/* u32Data */
		h_u32_to_le(self.cmdbuf + self.cmdidx, val);
		self.cmdidx += 4;
		/* u32Mask */
		h_u32_to_le(self.cmdbuf + self.cmdidx, 0x00000000);
		self.cmdidx += 4;

		self.usb_xfer(self.databuf, 4 * 2)
	}

	pub fn usb_read_mem8(&self, addr: u32, buffer: &mut [u8]) -> Result<(), NulinkError> {
		let mut offset = 0_u32;
		let mut bytes_remaining = 12_u32;

		debug!("nulink_usb_read_mem8: addr 0x{:08x}, len {}", addr, buffer.len() as u16);

		/* check whether data is word aligned */
		if addr % 4 != 0 {
			let aligned_addr = addr / 4;
			let aligned_addr = aligned_addr * 4;
			offset = addr - aligned_addr;
			debug!("nulink_usb_read_mem8: unaligned address addr 0x{:08x}/aligned addr 0x{:08x} offset {}",
					addr, aligned_addr, offset);

			addr = aligned_addr;
		}

		while len != 0 {
			if len < bytes_remaining {
				bytes_remaining = len;
			}

			let count = if len < 4 {
				1
			} else {
				2
			};

			self.usb_init_buffer(8 + 12 * count);
			/* set command ID */
			h_u32_to_le(self.cmdbuf + self.cmdidx, commands::WRITE_RAM);
			self.cmdidx += 4;
			/* Count of registers */
			self.cmdbuf[self.cmdidx] = count;
			self.cmdidx += 1;
			/* Array of bool value (u8ReadOld) */
			self.cmdbuf[self.cmdidx] = 0xFF;
			self.cmdidx += 1;
			/* Array of bool value (u8Verify) */
			self.cmdbuf[self.cmdidx] = 0x00;
			self.cmdidx += 1;
			/* ignore */
			self.cmdbuf[self.cmdidx] = 0;
			self.cmdidx += 1;

			for i in 0..count {
				/* u32Addr */
				h_u32_to_le(self.cmdbuf + self.cmdidx, addr);
				self.cmdidx += 4;
				/* u32Data */
				h_u32_to_le(self.cmdbuf + self.cmdidx, 0);
				self.cmdidx += 4;
				/* u32Mask */
				h_u32_to_le(self.cmdbuf + self.cmdidx, 0xFFFFFFFF);
				self.cmdidx += 4;
				/* proceed to the next one  */
				addr += 4;
			}

			self.usb_xfer(self.databuf, 4 * count * 2)?;

			/* fill in the output buffer */
			for i in 0..count {
				if i == 0 {
					memcpy(buffer, self.databuf + 4 + offset, len);
				} else {
					memcpy(buffer + 2 * i, self.databuf + 4 * (2 * i + 1), len - 2);
				}
			}

			if len >= bytes_remaining {
				len -= bytes_remaining;
			}
		}

		Ok(())
	}

	pub fn usb_write_mem8(&self, addr: u32, buffer: &[u8]) -> Result<(), NulinkError> {
		let mut offset = 0_u32;
		let mut bytes_remaining = 12_u32;

		debug!("nulink_usb_write_mem8: addr 0x{:08x}, len {}", addr, buffer.len() as u16);

		/* check whether data is word aligned */
		if addr % 4 != 0 {
			let aligned_addr = addr / 4;
			let aligned_addr = aligned_addr * 4;
			offset = addr - aligned_addr;
			debug!("nulink_usb_write_mem8: address not aligned. addr(0x{:08x})/aligned_addr(0x{:08x})/offset({})",
					addr, aligned_addr, offset);

			addr = aligned_addr;
		}

		while len != 0 {
			if len < bytes_remaining {
				bytes_remaining = len;
			}

			let count = if len < 4 {
				1
			} else {
				2
			};

			self.usb_init_buffer(8 + 12 * count);
			/* set command ID */
			h_u32_to_le(self.cmdbuf + self.cmdidx, commands::WRITE_RAM);
			self.cmdidx += 4;
			/* Count of registers */
			self.cmdbuf[self.cmdidx] = count;
			self.cmdidx += 1;
			/* Array of bool value (u8ReadOld) */
			self.cmdbuf[self.cmdidx] = 0x00;
			self.cmdidx += 1;
			/* Array of bool value (u8Verify) */
			self.cmdbuf[self.cmdidx] = 0x00;
			self.cmdidx += 1;
			/* ignore */
			self.cmdbuf[self.cmdidx] = 0;
			self.cmdidx += 1;

			for i in 0..count {
				/* u32Addr */
				h_u32_to_le(self.cmdbuf + self.cmdidx, addr);
				self.cmdidx += 4;
				/* u32Data */
				uint32_t u32buffer = buf_get_u32(buffer, 0, len * 8);
				u32buffer = (u32buffer << offset * 8);
				h_u32_to_le(self.cmdbuf + self.cmdidx, u32buffer);
				self.cmdidx += 4;
				/* u32Mask */
				if i == 0 {
					if offset == 0 {
						if len == 1 {
							h_u32_to_le(self.cmdbuf + self.cmdidx, 0xFFFFFF00);
							debug!("nulink_usb_write_mem8: count({}), mask: 0xFFFFFF00", i);
						} else {
							h_u32_to_le(self.cmdbuf + self.cmdidx, 0xFFFF0000);
							debug!("nulink_usb_write_mem8: count({}), mask: 0xFFFF0000", i);
						}
					} else {
						if len == 1 {
							h_u32_to_le(self.cmdbuf + self.cmdidx, 0xFF00FFFF);
							debug!("nulink_usb_write_mem8: count({}), mask: 0xFF00FFFF", i);

						} else {
							h_u32_to_le(self.cmdbuf + self.cmdidx, 0x0000FFFF);
							debug!("nulink_usb_write_mem8: count({}), mask: 0x0000FFFF", i);
						}
					}
				} else {
					if len == 4 {
						h_u32_to_le(self.cmdbuf + self.cmdidx, 0xFFFF0000);
						debug!("nulink_usb_write_mem8: count({}), mask: 0xFFFF0000", i);
					} else {
						h_u32_to_le(self.cmdbuf + self.cmdidx, 0x00000000);
						debug!("nulink_usb_write_mem8: count({}), mask: 0x00000000", i);
					}
				}
				self.cmdidx += 4;

				/* proceed to the next one */
				addr += 4;
				buffer += 4;
			}

			self.usb_xfer(self.databuf, 4 * count * 2)?;

			if len >= bytes_remaining {
				len -= bytes_remaining;
			}
		}

		Ok(())
	}

	pub fn usb_read_mem32(&self, addr: u32, buffer: &mut [u8]) -> Result<(), NulinkError> {
		let mut res = Ok(());
		let mut bytes_remaining = 12;

		/* data must be a multiple of 4 and word aligned */
		if (len % 4 || addr % 4) != 0 {
			error!("Invalid data alignment");
			return ERROR_TARGET_UNALIGNED_ACCESS;
		}

		while len != 0 {
			if len < bytes_remaining {
				bytes_remaining = len;
			}

			let count = bytes_remaining / 4;

			nulink_usb_init_buffer(handle, 8 + 12 * count);
			// set command ID
			self.cmdbuf[self.cmdidx..self.cmdidx+4].copy_from_slice((commands::WRITE_RAM as u32).to_le_bytes());
			self.cmdidx += 4;
			self.cmdbuf[self.cmdidx] = count; // Count of registers
			self.cmdidx += 1;
			self.cmdbuf[self.cmdidx] = 0xFF; // Array of bool value (u8ReadOld)
			self.cmdidx += 1;
			self.cmdbuf[self.cmdidx] = 0x00; // Array of bool value (u8Verify)
			self.cmdidx += 1;
			self.cmdbuf[self.cmdidx] = 0; // ignore
			self.cmdidx += 1;

			for i in 0..count {
				self.cmdbuf[self.cmdidx..self.cmdidx+4].copy_from_slice(addr.to_le_bytes()); // u32Addr
				self.cmdidx += 4;
				self.cmdbuf[self.cmdidx..self.cmdidx+4].copy_from_slice(0.to_le_bytes()); // u32Data
				self.cmdidx += 4;
				self.cmdbuf[self.cmdidx..self.cmdidx+4].copy_from_slice(0xFFFFFFFF.to_le_bytes()); // u32Mask
				self.cmdidx += 4;
				addr += 4; // proceed to the next one
			}

			res = self.usb_xfer(self.databuf, 4 * count * 2);

			// fill in the output buffer
			for i in 0..count {
				memcpy(buffer, self.databuf + 4 * (2 * i + 1), 4);
				buffer += 4;
			}

			if len >= bytes_remaining {
				len -= bytes_remaining;
			} else {
				len = 0;
			}
		}

		res
	}

	pub fn usb_write_mem32(&self, addr: u32, buffer: &[u8]) -> Result<(), NulinkError> {
		let mut res = Ok(());
		let bytes_remaining = 12;

		/* data must be a multiple of 4 and word aligned */
		if (len % 4 || addr % 4) != 0 {
			error!("Invalid data alignment");
			return ERROR_TARGET_UNALIGNED_ACCESS;
		}

		while len != 0 {
			if len < bytes_remaining {
				bytes_remaining = len;
			}

			let count = bytes_remaining / 4;

			self.usb_init_buffer(8 + 12 * count);
			/* set command ID */
			h_u32_to_le(self.cmdbuf + self.cmdidx, commands::WRITE_RAM);
			self.cmdidx += 4;
			/* Count of registers */
			self.cmdbuf[self.cmdidx] = count;
			self.cmdidx += 1;
			/* Array of bool value (u8ReadOld) */
			self.cmdbuf[self.cmdidx] = 0x00;
			self.cmdidx += 1;
			/* Array of bool value (u8Verify) */
			self.cmdbuf[self.cmdidx] = 0x00;
			self.cmdidx += 1;
			/* ignore */
			self.cmdbuf[self.cmdidx] = 0;
			self.cmdidx += 1;

			for i in 0..count {
				/* u32Addr */
				h_u32_to_le(self.cmdbuf + self.cmdidx, addr);
				self.cmdidx += 4;
				/* u32Data */
				let u32buffer = buf_get_u32(buffer, 0, 32);
				h_u32_to_le(self.cmdbuf + self.cmdidx, u32buffer);
				self.cmdidx += 4;
				/* u32Mask */
				h_u32_to_le(self.cmdbuf + self.cmdidx, 0x00000000);
				self.cmdidx += 4;

				/* proceed to the next one */
				addr += 4;
				buffer += 4;
			}

			res = self.usb_xfer(self.databuf, 4 * count * 2);

			if len >= bytes_remaining {
				len -= bytes_remaining;
			} else {
				len = 0;
			}
		}

		res
	}

	pub fn max_block_size(tar_autoincr_block: u32, address: u32) -> u32 {
		let mut max_tar_block = (tar_autoincr_block - ((tar_autoincr_block - 1) & address));

		if max_tar_block == 0 {
			max_tar_block = 4;
		}

		max_tar_block
	}

	pub fn usb_read_mem(&self, addr: u32, uint32_t size,
			uint32_t count, uint8_t *buffer) -> Result<(), NulinkError> {
		/* calculate byte count */
		count *= size;

		while count != 0 {
			let bytes_remaining = Self::max_block_size(self.max_mem_packet, addr) as usize;

			if count < bytes_remaining {
				bytes_remaining = count;
			}

			if bytes_remaining >= 4 {
				size = 4;
			}

			/* the nulink only supports 8/32bit memory read/writes
			 * honour 32bit, all others will be handled as 8bit access */
			if size == 4 {
				/* When in jtag mode the nulink uses the auto-increment functionality.
				 * However it expects us to pass the data correctly, this includes
				 * alignment and any page boundaries. We already do this as part of the
				 * adi_v5 implementation, but the nulink is a hla adapter and so this
				 * needs implementing manually.
				 * currently this only affects jtag mode, they do single
				 * access in SWD mode - but this may change and so we do it for both modes */

				/* we first need to check for any unaligned bytes */
				if addr % 4 != 0 {
					uint32_t head_bytes = 4 - (addr % 4);
					nulink_usb_read_mem8(handle, addr, head_bytes, buffer)?;
					buffer += head_bytes;
					addr += head_bytes;
					count -= head_bytes;
					bytes_remaining -= head_bytes;
				}

				if bytes_remaining % 4 != 0 {
					self.usb_read_mem(addr, 1, bytes_remaining, buffer)?;
				} else {
					self.usb_read_mem32(addr, bytes_remaining, buffer)?;
				}
			} else {
				self.usb_read_mem8(addr, bytes_remaining, buffer)?;
			}

			buffer += bytes_remaining;
			addr += bytes_remaining;
			count -= bytes_remaining;
		}

		Ok(())
	}

	pub fn usb_write_mem(handle: &NulinkUsbHandle, addr: u32, uint32_t size,
			uint32_t count, const uint8_t *buffer) -> Result<(), NulinkError> {
		if (addr < ARM_SRAM_BASE) {
			debug!("nulink_usb_write_mem: address below ARM_SRAM_BASE, not supported.\n");
			return Ok(());
		}

		/* calculate byte count */
		count *= size;

		while count != 0 {
			let bytes_remaining = Self::max_block_size(self.max_mem_packet, addr);

			if count < bytes_remaining {
				bytes_remaining = count;
			}

			if bytes_remaining >= 4 {
				size = 4;
			}

			/* the nulink only supports 8/32bit memory read/writes
			 * honour 32bit, all others will be handled as 8bit access */
			if (size == 4) {
				/* When in jtag mode the nulink uses the auto-increment functionality.
				 * However it expects us to pass the data correctly, this includes
				 * alignment and any page boundaries. We already do this as part of the
				 * adi_v5 implementation, but the nulink is a hla adapter and so this
				 * needs implementing manually.
				 * currently this only affects jtag mode, do single
				 * access in SWD mode - but this may change and so we do it for both modes */

				/* we first need to check for any unaligned bytes */
				if addr % 4 != 0 {
					uint32_t head_bytes = 4 - (addr % 4);
					nulink_usb_write_mem8(handle, addr, head_bytes, buffer)?;
					buffer += head_bytes;
					addr += head_bytes;
					count -= head_bytes;
					bytes_remaining -= head_bytes;
				}

				if bytes_remaining % 4 != 0 {
					nulink_usb_write_mem(handle, addr, 1, bytes_remaining, buffer)?;
				} else {
					nulink_usb_write_mem32(handle, addr, bytes_remaining, buffer)?;
				}

			} else {
				nulink_usb_write_mem8(handle, addr, bytes_remaining, buffer)?;
			}

			buffer += bytes_remaining;
			addr += bytes_remaining;
			count -= bytes_remaining;
		}

		Ok(())
	}
}

static int nulink_usb_override_target(const char *targetname)
{
    debug!("nulink_usb_override_target");

    return !strcmp(targetname, "cortex_m");
}

static int nulink_speed(handle: &NulinkUsbHandle, int khz, bool query)
{
    unsigned long max_ice_clock = khz;

    debug!("nulink_speed: query {}", if query { "yes" } else { "no" });

    if max_ice_clock > 12000 {
        max_ice_clock = 12000;
    } else if (max_ice_clock == 3 * 512) || (max_ice_clock == 1500) {
        max_ice_clock = 1500;
    } else if max_ice_clock >= 1000 {
        max_ice_clock = max_ice_clock / 1000 * 1000;
    } else {
        max_ice_clock = max_ice_clock / 100 * 100;
    }

    debug!("Nu-Link nulink_speed: {}", max_ice_clock);

    if !query {
        nulink_usb_init_buffer(handle, 4 * 6);
        /* set command ID */
        h_u32_to_le(self.cmdbuf + self.cmdidx, commands::SET_CONFIG);
        self.cmdidx += 4;
        /* set max SWD clock */
        h_u32_to_le(self.cmdbuf + self.cmdidx, max_ice_clock);
        self.cmdidx += 4;
        /* chip type: NUC_CHIP_TYPE_GENERAL_V6M */
        h_u32_to_le(self.cmdbuf + self.cmdidx, 0);
        self.cmdidx += 4;
        /* IO voltage */
        h_u32_to_le(self.cmdbuf + self.cmdidx, 5000);
        self.cmdidx += 4;
        /* If supply voltage to target or not */
        h_u32_to_le(self.cmdbuf + self.cmdidx, 0);
        self.cmdidx += 4;
        /* USB_FUNC_E: USB_FUNC_HID_BULK */
        h_u32_to_le(self.cmdbuf + self.cmdidx, 2);
        self.cmdidx += 4;

        nulink_usb_xfer(handle, self.databuf, 4 * 3);

        debug!("nulink_speed: h->hardware_config({})", h.hardware_config);
        if (self.hardware_config & HARDWARE_CONFIG_NULINKPRO)
            info!("Nu-Link target_voltage_mv[0]({:04x}), target_voltage_mv[1]({:04x}), target_voltage_mv[2]({:04x}), if_target_power_supplied({})",
                le_to_h_u16(self.databuf + 4 * 1 + 0),
                le_to_h_u16(self.databuf + 4 * 1 + 2),
                le_to_h_u16(self.databuf + 4 * 2 + 0),
                le_to_h_u16(self.databuf + 4 * 2 + 2) & 1);
    }

    max_ice_clock
}

static int nulink_usb_close(handle: &NulinkUsbHandle)
{
    debug!("nulink_usb_close");

    if (h && h->dev_handle) != 0 {
        hid_close(h->dev_handle);
    }

    free(h);

    hid_exit();

    return ERROR_OK;
}

static int nulink_usb_open(struct hl_interface_param_s *param) -> Result<Box<NulinkUsbHandle>, NulinkError> {
    struct hid_device_info *devs, *cur_dev;
    uint16_t target_vid = 0;
    uint16_t target_pid = 0;
    wchar_t *target_serial = NULL;

    debug!("nulink_usb_open");

    if (param->transport != HL_TRANSPORT_SWD)
        return TARGET_UNKNOWN;

    if (!param->vid[0] && !param->pid[0]) != 0 {
        error!("Missing vid/pid");
        return ERROR_FAIL;
    }

    if hid_init() != 0 {
        error!("unable to open HIDAPI");
        return ERROR_FAIL;
    }

    struct NulinkUsbHandle *h = calloc(1, sizeof(*h));
    if (!h) {
        error!("Out of memory");
        goto error_open;
    }

    if (param->serial) {
        size_t len = mbstowcs(NULL, param->serial, 0);

        target_serial = calloc(len + 1, sizeof(wchar_t));
        if (!target_serial) {
            error!("Out of memory");
            goto error_open;
        }

        if (mbstowcs(target_serial, param->serial, len + 1) == (size_t)(-1)) {
            warn!("unable to convert serial");
            free(target_serial);
            target_serial = NULL;
        }
    }

    devs = hid_enumerate(0, 0);
    cur_dev = devs;
    while (cur_dev) {
        bool found = false;

        for (unsigned int i = 0; param->vid[i] || param->pid[i]; i++) {
            if (param->vid[i] == cur_dev->vendor_id && param->pid[i] == cur_dev->product_id) {
                found = true;
                break;
            }
        }

        if found {
            if (!target_serial) {
                break;
            }
            if (cur_dev->serial_number && wcscmp(target_serial, cur_dev->serial_number) == 0) {
                break;
            }
        }

        cur_dev = cur_dev->next;
    }
    if (cur_dev) {
        target_vid = cur_dev->vendor_id;
        target_pid = cur_dev->product_id;
    }

    hid_free_enumeration(devs);

    if (target_vid == 0 && target_pid == 0) {
        error!("unable to find Nu-Link");
        goto error_open;
    }

    hid_device *dev = hid_open(target_vid, target_pid, target_serial);
    if (!dev) {
        error!("unable to open Nu-Link device 0x%" PRIx16 ":0x%" PRIx16, target_vid, target_pid);
        goto error_open;
    }

    h->dev_handle = dev;
    h->usbcmdidx = 0;

    match target_pid {
        NULINK2_USB_PID1 | NULINK2_USB_PID2 => {
            h->hardware_config = HARDWARE_CONFIG_NULINK2;
            h->max_packet_size = NULINK2_HID_MAX_SIZE;
            h->init_buffer = nulink2_usb_init_buffer;
            h->xfer = nulink2_usb_xfer;
        }
        _ => {
            h->hardware_config = 0;
            h->max_packet_size = NULINK_HID_MAX_SIZE;
            h->init_buffer = nulink1_usb_init_buffer;
            h->xfer = nulink1_usb_xfer;
        }
    }

    /* get the device version */
    h.cmdsize = 4 * 5;
    int err = nulink_usb_version(h);
    if (err != ERROR_OK) {
        debug!("nulink_usb_version failed with cmdSize(4 * 5)");
        h.cmdsize = 4 * 6;
        err = nulink_usb_version(h);
        if (err != ERROR_OK)
            debug!("nulink_usb_version failed with cmdSize(4 * 6)");
    }

    /* SWD clock rate : 1MHz */
    nulink_speed(h, 1000, false);

    /* get cpuid, so we can determine the max page size
     * start with a safe default */
    h->max_mem_packet = (1 << 10);

    debug!("nulink_usb_open: we manually perform nulink_usb_reset");
    nulink_usb_reset(h);

    free(target_serial);
    return Ok(h);

error_open:
    nulink_usb_close(h);
    free(target_serial);

    return ERROR_FAIL;
}

struct hl_layout_api_s nulink_usb_layout_api = {
    .open = nulink_usb_open,
    .close = nulink_usb_close,
    .idcode = nulink_usb_idcode,
    .state = nulink_usb_state,
    .reset = nulink_usb_reset,
    .assert_srst = nulink_usb_assert_srst,
    .run = nulink_usb_run,
    .halt = nulink_usb_halt,
    .step = nulink_usb_step,
    .read_reg = nulink_usb_read_reg,
    .write_reg = nulink_usb_write_reg,
    .read_mem = nulink_usb_read_mem,
    .write_mem = nulink_usb_write_mem,
    .write_debug_reg = nulink_usb_write_debug_reg,
    .override_target = nulink_usb_override_target,
    .speed = nulink_speed,
};
