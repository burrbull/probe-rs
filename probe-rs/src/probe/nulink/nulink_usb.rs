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
};

pub enum NulinkConnect {
    Normal = 0,      /* Support all reset method */
    PreReset = 1,   /* Support all reset method */
    UnderReset = 2, /* Support all reset method */
    None = 3,        /* Support RESET_HW, (RESET_AUTO = RESET_HW) */
    Disconnect = 4,  /* Support RESET_NONE, (RESET_AUTO = RESET_NONE) */
    IcpMode = 5     /* Support NUC505 ICP mode*/
};

pub fn nulink_usb_xfer_rw(handle: &NulinkUsbHandle, buf: &mut [u8]) -> Result<(), NulinkError> {
    if hid_write(h->dev_handle, h->cmdbuf, h.max_packet_size + 1).is_err() {
        error!("hid_write");
        return Err(NulinkError::Fail);
    }

    if hid_read_timeout(h->dev_handle, buf[..h.max_packet_size], NULINK_READ_TIMEOUT).is_err() {
        Err(NulinkError::Fail)
    } else {
        Ok(())
    }
}

pub fn nulink1_usb_xfer(handle: &NulinkUsbHandle, buf: &mut [u8]) -> Result<(), NulinkError> {
    nulink_usb_xfer_rw(h, h->tempbuf)?;

    memcpy(buf, h->tempbuf + 2, V6M_MAX_COMMAND_LENGTH);

    Ok(())
}

pub fn nulink2_usb_xfer(handle: &NulinkUsbHandle, buf: &mut [u8]) -> Result<(), NulinkError> {
    nulink_usb_xfer_rw(h, h->tempbuf)?;

    memcpy(buf, h->tempbuf + 3, V7M_MAX_COMMAND_LENGTH);

    Ok(())
}

pub fn nulink1_usb_init_buffer(handle: &NulinkUsbHandle, size: usize) {
    h->cmdidx = 0;

    memset(h->cmdbuf, 0, h->max_packet_size + 1);
    memset(h->tempbuf, 0, h->max_packet_size);
    memset(h->databuf, 0, h->max_packet_size);

    h->cmdbuf[0] = 0; /* report number */
    h->cmdbuf[1] = ++h->usbcmdidx & 0x7F;
    h->cmdbuf[2] = size;
    h->cmdidx += 3;
}

pub fn nulink2_usb_init_buffer(handle: &NulinkUsbHandle, size: usize) {
    h->cmdidx = 0;

    memset(h->cmdbuf, 0, h->max_packet_size + 1);
    memset(h->tempbuf, 0, h->max_packet_size);
    memset(h->databuf, 0, h->max_packet_size);

    h->cmdbuf[0] = 0; /* report number */
    h->cmdbuf[1] = ++h->usbcmdidx & 0x7F;
    h_u16_to_le(h->cmdbuf + 2, size);
    h->cmdidx += 4;
}

pub fn nulink_usb_xfer(handle: &NulinkUsbHandle, buf: &mut [u8]) -> Result<(), NulinkError> {
    h->xfer(handle, buf)
}

pub fn nulink_usb_init_buffer(handle: &NulinkUsbHandle, size: usize) {
    h->init_buffer(handle, size);
}

pub fn nulink_usb_version(handle: &NulinkUsbHandle) -> Result<(), NulinkError> {
    debug!("nulink_usb_version");

    nulink_usb_init_buffer(handle, V6M_MAX_COMMAND_LENGTH);

    memset(h->cmdbuf + h->cmdidx, 0xFF, V6M_MAX_COMMAND_LENGTH);
    h->cmdbuf[h->cmdidx + 4] = 0xA1; /* host_rev_num: 6561 */;
    h->cmdbuf[h->cmdidx + 5] = 0x19;

    nulink_usb_xfer(handle, h->databuf, h->cmdsize)?;

    info!("Nu-Link firmware_version {}, product_id (0x{:08x})",
             le_to_h_u32(h->databuf),
             le_to_h_u32(h->databuf + 4 * 1));

    const bool is_nulinkpro = !!(le_to_h_u32(h->databuf + 4 * 2) & 1);
    if (is_nulinkpro) {
        info!("Adapter is Nu-Link-Pro, target_voltage_mv({}), usb_voltage_mv({})",
                 le_to_h_u16(h->databuf + 4 * 3 + 0),
                 le_to_h_u16(h->databuf + 4 * 3 + 2));

        h->hardware_config |= HARDWARE_CONFIG_NULINKPRO;
    } else {
        info!("Adapter is Nu-Link");
    }

    Ok(())
}

pub fn nulink_usb_idcode(handle: &NulinkUsbHandle, uint32_t *idcode) -> Result<(), NulinkError> {
    debug!("nulink_usb_idcode");

    nulink_usb_init_buffer(handle, 4 * 1);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::CHECK_ID);
    h->cmdidx += 4;

    nulink_usb_xfer(handle, h->databuf, 4 * 2)?;

    *idcode = le_to_h_u32(h->databuf + 4 * 1);

    info!("IDCODE: 0x{08x}", *idcode);

    Ok(())
}

pub fn nulink_usb_write_debug_reg(handle: &NulinkUsbHandle, addr: u32, val: u32) -> Result<(), NulinkError> {
    debug!("nulink_usb_write_debug_reg 0x{:08x} 0x{:08x}", addr, val);

    nulink_usb_init_buffer(handle, 8 + 12 * 1);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::WRITE_RAM);
    h->cmdidx += 4;
    /* Count of registers */
    h->cmdbuf[h->cmdidx] = 1;
    h->cmdidx += 1;
    /* Array of bool value (u8ReadOld) */
    h->cmdbuf[h->cmdidx] = 0x00;
    h->cmdidx += 1;
    /* Array of bool value (u8Verify) */
    h->cmdbuf[h->cmdidx] = 0x00;
    h->cmdidx += 1;
    /* ignore */
    h->cmdbuf[h->cmdidx] = 0;
    h->cmdidx += 1;
    /* u32Addr */
    h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
    h->cmdidx += 4;
    /* u32Data */
    h_u32_to_le(h->cmdbuf + h->cmdidx, val);
    h->cmdidx += 4;
    /* u32Mask */
    h_u32_to_le(h->cmdbuf + h->cmdidx, 0x00000000);
    h->cmdidx += 4;

    nulink_usb_xfer(handle, h->databuf, 4 * 2)
}

pub enum TargetState {
    Unknown,
    Halted,
    Running,
}

pub fn nulink_usb_state(handle: &NulinkUsbHandle) -> TargetState {
    nulink_usb_init_buffer(handle, 4 * 1);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::CHECK_MCU_STOP);
    h->cmdidx += 4;

    if nulink_usb_xfer(handle, h->databuf, 4 * 4).is_err() {
        return TargetState::Unknown;
    }

    if (!le_to_h_u32(h->databuf + 4 * 2)) {
        TargetState::Halted
    } else {
        TargetState::Running
    }
}

pub fn nulink_usb_assert_srst(handle: &NulinkUsbHandle, int srst) -> Result<(), NulinkError> {
    debug!("nulink_usb_assert_srst");

    nulink_usb_init_buffer(handle, 4 * 4);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::MCU_RESET);
    h->cmdidx += 4;
    /* set reset type */
    h_u32_to_le(h->cmdbuf + h->cmdidx, RESET_SYSRESETREQ);
    h->cmdidx += 4;
    /* set connect type */
    h_u32_to_le(h->cmdbuf + h->cmdidx, CONNECT_NORMAL);
    h->cmdidx += 4;
    /* set extMode */
    h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
    h->cmdidx += 4;

    nulink_usb_xfer(handle, h->databuf, 4 * 4)
}

pub fn nulink_usb_reset(handle: &NulinkUsbHandle) -> Result<(), NulinkError> {
    debug!("nulink_usb_reset");

    nulink_usb_init_buffer(handle, 4 * 4);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::MCU_RESET);
    h->cmdidx += 4;
    /* set reset type */
    h_u32_to_le(h->cmdbuf + h->cmdidx, RESET_HW);
    h->cmdidx += 4;
    /* set connect type */
    h_u32_to_le(h->cmdbuf + h->cmdidx, CONNECT_NORMAL);
    h->cmdidx += 4;
    /* set extMode */
    h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
    h->cmdidx += 4;

    nulink_usb_xfer(handle, h->databuf, 4 * 4)
}

pub fn nulink_usb_run(handle: &NulinkUsbHandle) -> Result<(), NulinkError> {
    debug!("nulink_usb_run");

    nulink_usb_init_buffer(handle, 4 * 1);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::MCU_FREE_RUN);
    h->cmdidx += 4;

    nulink_usb_xfer(handle, h->databuf, 4 * 4)
}

pub fn nulink_usb_halt(handle: &NulinkUsbHandle) -> Result<(), NulinkError> {
    debug!("nulink_usb_halt");

    nulink_usb_init_buffer(handle, 4 * 1);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::MCU_STOP_RUN);
    h->cmdidx += 4;

    let res = nulink_usb_xfer(handle, h->databuf, 4 * 4)

    debug!("Nu-Link stop_pc 0x{:08x}", le_to_h_u32(h->databuf + 4));

    res
}

pub fn nulink_usb_step(handle: &NulinkUsbHandle) -> Result<(), NulinkError> {
    debug!("nulink_usb_step");

    nulink_usb_init_buffer(handle, 4 * 1);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::MCU_STEP_RUN);
    h->cmdidx += 4;

    let res = nulink_usb_xfer(handle, h->databuf, 4 * 4)

    debug!("Nu-Link pc 0x{:08x}", le_to_h_u32(h->databuf + 4));

    res
}

pub fn nulink_usb_read_reg(handle: &NulinkUsbHandle, unsigned int regsel, uint32_t *val) -> Result<(), NulinkError> {
    nulink_usb_init_buffer(handle, 8 + 12 * 1);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::WRITE_REG);
    h->cmdidx += 4;
    /* Count of registers */
    h->cmdbuf[h->cmdidx] = 1;
    h->cmdidx += 1;
    /* Array of bool value (u8ReadOld) */
    h->cmdbuf[h->cmdidx] = 0xFF;
    h->cmdidx += 1;
    /* Array of bool value (u8Verify) */
    h->cmdbuf[h->cmdidx] = 0x00;
    h->cmdidx += 1;
    /* ignore */
    h->cmdbuf[h->cmdidx] = 0;
    h->cmdidx += 1;
    /* u32Addr */
    h_u32_to_le(h->cmdbuf + h->cmdidx, regsel);
    h->cmdidx += 4;
    /* u32Data */
    h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
    h->cmdidx += 4;
    /* u32Mask */
    h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFFFFUL);
    h->cmdidx += 4;

    nulink_usb_xfer(handle, h->databuf, 4 * 2)?;

    *val = le_to_h_u32(h->databuf + 4 * 1);

    Ok(())
}

pub fn nulink_usb_write_reg(handle: &NulinkUsbHandle, unsigned int regsel, uint32_t val) -> Result<(), NulinkError> {
    nulink_usb_init_buffer(handle, 8 + 12 * 1);
    /* set command ID */
    h_u32_to_le(h->cmdbuf + h->cmdidx, commands::WRITE_REG);
    h->cmdidx += 4;
    /* Count of registers */
    h->cmdbuf[h->cmdidx] = 1;
    h->cmdidx += 1;
    /* Array of bool value (u8ReadOld) */
    h->cmdbuf[h->cmdidx] = 0x00;
    h->cmdidx += 1;
    /* Array of bool value (u8Verify) */
    h->cmdbuf[h->cmdidx] = 0x00;
    h->cmdidx += 1;
    /* ignore */
    h->cmdbuf[h->cmdidx] = 0;
    h->cmdidx += 1;
    /* u32Addr */
    h_u32_to_le(h->cmdbuf + h->cmdidx, regsel);
    h->cmdidx += 4;
    /* u32Data */
    h_u32_to_le(h->cmdbuf + h->cmdidx, val);
    h->cmdidx += 4;
    /* u32Mask */
    h_u32_to_le(h->cmdbuf + h->cmdidx, 0x00000000UL);
    h->cmdidx += 4;

    nulink_usb_xfer(handle, h->databuf, 4 * 2)
}

pub fn nulink_usb_read_mem8(handle: &NulinkUsbHandle, addr: u32, buffer: &mut [u8]) -> Result<(), NulinkError> {
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

        nulink_usb_init_buffer(handle, 8 + 12 * count);
        /* set command ID */
        h_u32_to_le(h->cmdbuf + h->cmdidx, commands::WRITE_RAM);
        h->cmdidx += 4;
        /* Count of registers */
        h->cmdbuf[h->cmdidx] = count;
        h->cmdidx += 1;
        /* Array of bool value (u8ReadOld) */
        h->cmdbuf[h->cmdidx] = 0xFF;
        h->cmdidx += 1;
        /* Array of bool value (u8Verify) */
        h->cmdbuf[h->cmdidx] = 0x00;
        h->cmdidx += 1;
        /* ignore */
        h->cmdbuf[h->cmdidx] = 0;
        h->cmdidx += 1;

        for (unsigned int i = 0; i < count; i++) {
            /* u32Addr */
            h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
            h->cmdidx += 4;
            /* u32Data */
            h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
            h->cmdidx += 4;
            /* u32Mask */
            h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFFFF);
            h->cmdidx += 4;
            /* proceed to the next one  */
            addr += 4;
        }

        nulink_usb_xfer(handle, h->databuf, 4 * count * 2)?;

        /* fill in the output buffer */
        for (unsigned int i = 0; i < count; i++) {
            if (i == 0)
                memcpy(buffer, h->databuf + 4 + offset, len);
            else
                memcpy(buffer + 2 * i, h->databuf + 4 * (2 * i + 1), len - 2);
        }

        if len >= bytes_remaining {
            len -= bytes_remaining;
        }
    }

    Ok(())
}

pub fn nulink_usb_write_mem8(handle: &NulinkUsbHandle, addr: u32, buffer: &[u8]) -> Result<(), NulinkError> {
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

        nulink_usb_init_buffer(handle, 8 + 12 * count);
        /* set command ID */
        h_u32_to_le(h->cmdbuf + h->cmdidx, commands::WRITE_RAM);
        h->cmdidx += 4;
        /* Count of registers */
        h->cmdbuf[h->cmdidx] = count;
        h->cmdidx += 1;
        /* Array of bool value (u8ReadOld) */
        h->cmdbuf[h->cmdidx] = 0x00;
        h->cmdidx += 1;
        /* Array of bool value (u8Verify) */
        h->cmdbuf[h->cmdidx] = 0x00;
        h->cmdidx += 1;
        /* ignore */
        h->cmdbuf[h->cmdidx] = 0;
        h->cmdidx += 1;

        for i in 0..count {
            /* u32Addr */
            h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
            h->cmdidx += 4;
            /* u32Data */
            uint32_t u32buffer = buf_get_u32(buffer, 0, len * 8);
            u32buffer = (u32buffer << offset * 8);
            h_u32_to_le(h->cmdbuf + h->cmdidx, u32buffer);
            h->cmdidx += 4;
            /* u32Mask */
            if i == 0 {
                if offset == 0 {
                    if len == 1 {
                        h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFF00);
                        debug!("nulink_usb_write_mem8: count({}), mask: 0xFFFFFF00", i);
                    } else {
                        h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFF0000);
                        debug!("nulink_usb_write_mem8: count({}), mask: 0xFFFF0000", i);
                    }
                } else {
                    if len == 1 {
                        h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFF00FFFF);
                        debug!("nulink_usb_write_mem8: count({}), mask: 0xFF00FFFF", i);

                    } else {
                        h_u32_to_le(h->cmdbuf + h->cmdidx, 0x0000FFFF);
                        debug!("nulink_usb_write_mem8: count({}), mask: 0x0000FFFF", i);
                    }
                }
            } else {
                if len == 4 {
                    h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFF0000);
                    debug!("nulink_usb_write_mem8: count({}), mask: 0xFFFF0000", i);
                } else {
                    h_u32_to_le(h->cmdbuf + h->cmdidx, 0x00000000);
                    debug!("nulink_usb_write_mem8: count({}), mask: 0x00000000", i);
                }
            }
            h->cmdidx += 4;

            /* proceed to the next one */
            addr += 4;
            buffer += 4;
        }

        nulink_usb_xfer(handle, h->databuf, 4 * count * 2)?;

        if len >= bytes_remaining {
            len -= bytes_remaining;
        }
    }

    Ok(())
}

pub fn nulink_usb_read_mem32(handle: &NulinkUsbHandle, uint32_t addr, uint16_t len,
        uint8_t *buffer) -> Result<(), NulinkError> {
    let mut res = Ok(());
    let mut bytes_remaining = 12;

    assert(handle);

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
        h.cmdbuf[h.cmdidx..h.cmdidx+4].copy_from_slice((commands::WRITE_RAM as u32).to_le_bytes());
        h.cmdidx += 4;
        h.cmdbuf[h.cmdidx] = count; // Count of registers
        h.cmdidx += 1;
        h.cmdbuf[h.cmdidx] = 0xFF; // Array of bool value (u8ReadOld)
        h.cmdidx += 1;
        h.cmdbuf[h.cmdidx] = 0x00; // Array of bool value (u8Verify)
        h.cmdidx += 1;
        h->cmdbuf[h->cmdidx] = 0; // ignore
        h->cmdidx += 1;

        for i in 0..count {
            h.cmdbuf[h.cmdidx..h.cmdidx+4].copy_from_slice(addr.to_le_bytes()); // u32Addr
            h.cmdidx += 4;
            h.cmdbuf[h.cmdidx..h.cmdidx+4].copy_from_slice(0.to_le_bytes()); // u32Data
            h.cmdidx += 4;
            h.cmdbuf[h.cmdidx..h.cmdidx+4].copy_from_slice(0xFFFFFFFF.to_le_bytes()); // u32Mask
            h.cmdidx += 4;
            addr += 4; // proceed to the next one
        }

        res = nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

        // fill in the output buffer
        for (unsigned int i = 0; i < count; i++) {
            memcpy(buffer, h->databuf + 4 * (2 * i + 1), 4);
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

pub fn nulink_usb_write_mem32(void *handle, uint32_t addr, uint16_t len,
        const uint8_t *buffer) -> Result<(), NulinkError> {
    let mut res = Ok(());
    uint32_t bytes_remaining = 12;
    struct NulinkUsbHandle *h = handle;

    assert(handle);

    /* data must be a multiple of 4 and word aligned */
    if (len % 4 || addr % 4) {
        error!("Invalid data alignment");
        return ERROR_TARGET_UNALIGNED_ACCESS;
    }

    while (len) {
        if (len < bytes_remaining)
            bytes_remaining = len;

        unsigned int count = bytes_remaining / 4;

        nulink_usb_init_buffer(handle, 8 + 12 * count);
        /* set command ID */
        h_u32_to_le(h->cmdbuf + h->cmdidx, commands::WRITE_RAM);
        h->cmdidx += 4;
        /* Count of registers */
        h->cmdbuf[h->cmdidx] = count;
        h->cmdidx += 1;
        /* Array of bool value (u8ReadOld) */
        h->cmdbuf[h->cmdidx] = 0x00;
        h->cmdidx += 1;
        /* Array of bool value (u8Verify) */
        h->cmdbuf[h->cmdidx] = 0x00;
        h->cmdidx += 1;
        /* ignore */
        h->cmdbuf[h->cmdidx] = 0;
        h->cmdidx += 1;

        for (unsigned int i = 0; i < count; i++) {
            /* u32Addr */
            h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
            h->cmdidx += 4;
            /* u32Data */
            uint32_t u32buffer = buf_get_u32(buffer, 0, 32);
            h_u32_to_le(h->cmdbuf + h->cmdidx, u32buffer);
            h->cmdidx += 4;
            /* u32Mask */
            h_u32_to_le(h->cmdbuf + h->cmdidx, 0x00000000);
            h->cmdidx += 4;

            /* proceed to the next one */
            addr += 4;
            buffer += 4;
        }

        res = nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

        if (len >= bytes_remaining)
            len -= bytes_remaining;
        else
            len = 0;
    }

    res
}

pub fn nulink_max_block_size(tar_autoincr_block: u32, address: u32) -> u32 {
    let mut max_tar_block = (tar_autoincr_block - ((tar_autoincr_block - 1) & address));

    if max_tar_block == 0 {
        max_tar_block = 4;
    }

    max_tar_block
}

pub fn nulink_usb_read_mem(handle: &NulinkUsbHandle, addr: u32, uint32_t size,
        uint32_t count, uint8_t *buffer) -> Result<(), NulinkError> {
    /* calculate byte count */
    count *= size;

    while (count) {
        uint32_t bytes_remaining = nulink_max_block_size(h->max_mem_packet, addr);

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
                nulink_usb_read_mem(handle, addr, 1, bytes_remaining, buffer)?;
            } else {
                nulink_usb_read_mem32(handle, addr, bytes_remaining, buffer)?;
            }
        } else {
            nulink_usb_read_mem8(handle, addr, bytes_remaining, buffer)?;
        }

        buffer += bytes_remaining;
        addr += bytes_remaining;
        count -= bytes_remaining;
    }

    Ok(())
}

pub fn nulink_usb_write_mem(void *handle, uint32_t addr, uint32_t size,
        uint32_t count, const uint8_t *buffer) -> Result<(), NulinkError> {
    int retval = ERROR_OK;
    struct NulinkUsbHandle *h = handle;

    if (addr < ARM_SRAM_BASE) {
        debug!("nulink_usb_write_mem: address below ARM_SRAM_BASE, not supported.\n");
        return Ok(());
    }

    /* calculate byte count */
    count *= size;

    while count != 0 {
        let bytes_remaining = nulink_max_block_size(h->max_mem_packet, addr);

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
        h_u32_to_le(h->cmdbuf + h->cmdidx, commands::SET_CONFIG);
        h->cmdidx += 4;
        /* set max SWD clock */
        h_u32_to_le(h->cmdbuf + h->cmdidx, max_ice_clock);
        h->cmdidx += 4;
        /* chip type: NUC_CHIP_TYPE_GENERAL_V6M */
        h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
        h->cmdidx += 4;
        /* IO voltage */
        h_u32_to_le(h->cmdbuf + h->cmdidx, 5000);
        h->cmdidx += 4;
        /* If supply voltage to target or not */
        h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
        h->cmdidx += 4;
        /* USB_FUNC_E: USB_FUNC_HID_BULK */
        h_u32_to_le(h->cmdbuf + h->cmdidx, 2);
        h->cmdidx += 4;

        nulink_usb_xfer(handle, h->databuf, 4 * 3);

        debug!("nulink_speed: h->hardware_config({})", h.hardware_config);
        if (h->hardware_config & HARDWARE_CONFIG_NULINKPRO)
            info!("Nu-Link target_voltage_mv[0]({:04x}), target_voltage_mv[1]({:04x}), target_voltage_mv[2]({:04x}), if_target_power_supplied({})",
                le_to_h_u16(h->databuf + 4 * 1 + 0),
                le_to_h_u16(h->databuf + 4 * 1 + 2),
                le_to_h_u16(h->databuf + 4 * 2 + 0),
                le_to_h_u16(h->databuf + 4 * 2 + 2) & 1);
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
