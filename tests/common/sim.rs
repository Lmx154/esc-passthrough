//! A simulated AM32 bootloader on up to four single-wire lines, for the host tests.
//!
//! Written from the bootloader's `main.c` (BL = `_plan/ref/AM32-bootloader/bootloader/main.c`,
//! line numbers below) and NOT from this crate's encoders: it parses the incoming bytes itself
//! and builds its answers byte by byte. It implements `esc_passthrough::line::EscLines`, so the
//! client under test talks to it as to a board.
//!
//! What it models (as `main.c` does):
//! - a burst of bytes from one `write` is one command (the ESC ends a command at a gap of more
//!   than 5 bit times, BL:860; a client sends back to back). Bytes of a burst beyond the
//!   258-byte receive buffer, or beyond what the ESC consumed, are lost (BL:1007-1048);
//! - the init match at three offsets (BL:604-627), the 9-byte device-info reply;
//! - set address with the magic addresses 0x20-0x23 and the refusal of the others below 1024
//!   (BL:671-713), set buffer with no answer (BL:715-737), the payload with its CRC check
//!   (BL:583-600), program (BL:636-667), erase (BL:753-769), read, after which the address is
//!   forgotten (BL:771-813), keep-alive answering `C1` (BL:739-751), run (BL:629-634);
//! - the three acknowledges `30`, `C1`, `C2` and `invalid_command`, the count of bad commands,
//!   after which (more than 100) the bootloader tries to jump (BL:1276-1279, jump at BL:423-476);
//! - the flash image, one per ESC.
//!
//! What it does not model: the 20 ms line-low jump (BL:841-853), the 250 us command window
//! (BL:1028), the idle/pull-down checks at start-up, the DroneCAN paths, the erase-then-write of
//! a page in `save_flash_nolib` (a write is a plain overwrite; `write_granule` models the
//! alignment it demands, BL: `Mcu/*/Src/eeprom.c`), reads outside the image (they return 0xFF; the
//! real part reads whatever is mapped there).
//!
//! Time is virtual: a write takes 520 us per byte (10 bits of 52 us), a read that finds nothing
//! takes its timeout, and answers carry the time they become readable.
#![allow(dead_code)]

use esc_passthrough::line::EscLines;
use std::collections::VecDeque;

pub const MCU_FLASH_START: u32 = 0x0800_0000;
const BIT_US: u64 = 52;
const BYTE_US: u64 = 10 * BIT_US;
const RX_BUFFER: usize = 258;
const DEVINFO_OFFSET: u32 = 0x0D00;
/// `BOOTLOADER_VERSION` in `Inc/version.h:5`; the bootloader writes it into byte 2 of a
/// settings-block write (BL:650-658).
pub const BOOTLOADER_VERSION: u8 = 19;

/// What the sim does to the next answer an ESC sends (one fault per answer, in order).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Fault {
    /// Leave this answer alone (to skip to a later one).
    Pass,
    /// Send nothing.
    NoAnswer,
    /// Send it, but readable only this many microseconds later than it should be.
    Late(u32),
    /// Send only the first `k` bytes.
    Truncated(usize),
    /// Flip a bit in the CRC low byte of a read reply (the byte three from the end).
    CorruptCrc,
    /// Replace the last byte (the acknowledge).
    WrongAck(u8),
    /// Replace the whole answer.
    Replace(Vec<u8>),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SimError {
    NoLineSelected,
    NoSuchLine(u8),
    Injected,
}

/// One call on the line, as the client made it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Wire {
    Select(u8),
    Write(Vec<u8>),
    Read { cap: usize, timeout_us: u32 },
    Break(bool),
}

impl Wire {
    /// The call without the bytes written: what must not depend on the data.
    pub fn shape(&self) -> Wire {
        match self {
            Wire::Write(bytes) => Wire::Write(vec![0; bytes.len()]),
            other => other.clone(),
        }
    }
}

/// One simulated ESC: its flash and the state `main.c` keeps.
#[derive(Debug, Clone)]
pub struct EscModel {
    flash: Vec<u8>,
    pin_code: u8,
    size_code: u8,
    protocol_version: u8,
    shift: u32,
    eeprom_add: u32,
    app_add: u32,
    file_name_add: u32,
    write_granule: u32,
    // main.c state
    address: u32,
    continue_address: u32,
    payload: [u8; 256],
    payload_size: u16,
    incoming_payload_no_command: bool,
    invalid_command: u16,
    initialized: bool,
    running: bool,
    // the line
    pending: VecDeque<(u64, u8)>,
    faults: VecDeque<Fault>,
    corrupt_incoming: Option<(usize, usize, u8)>,
    // records
    bursts: Vec<Vec<u8>>,
    answers: Vec<Vec<u8>>,
}

fn crc16(bytes: &[u8]) -> u16 {
    // BL:486-511.
    let mut ret: u16 = 0;
    for &byte in bytes {
        let mut xb = byte;
        for _ in 0..8 {
            if ((xb & 1) as u16 ^ (ret & 1)) != 0 {
                ret >>= 1;
                ret ^= 0xA001;
            } else {
                ret >>= 1;
            }
            xb >>= 1;
        }
    }
    ret
}

fn check_crc(buf: &[u8], length: usize) -> bool {
    let received = buf[length] as u16 | ((buf[length + 1] as u16) << 8);
    received == crc16(&buf[..length])
}

impl EscModel {
    /// A 32 KB part laid out like the F421 target: settings block at 0x7C00, file name 32 bytes
    /// below it, application from 0x1000, no address shift (BL:214-218). The first byte of the
    /// settings block is 1 and the application's vectors are plausible, so run succeeds.
    pub fn f421(pin_code: u8) -> Self {
        let mut esc = Self {
            flash: vec![0xFF; 32 * 1024],
            pin_code,
            size_code: 0x1F,
            protocol_version: 3,
            shift: 0,
            eeprom_add: MCU_FLASH_START + 0x7C00,
            app_add: MCU_FLASH_START + 0x1000,
            file_name_add: MCU_FLASH_START + 0x7C00 - 32,
            write_granule: 1,
            address: 0,
            continue_address: 0,
            payload: [0; 256],
            payload_size: 0,
            incoming_payload_no_command: false,
            invalid_command: 0,
            initialized: false,
            running: false,
            pending: VecDeque::new(),
            faults: VecDeque::new(),
            corrupt_incoming: None,
            bursts: Vec::new(),
            answers: Vec::new(),
        };
        esc.flash[0x7C00] = 1;
        let name = b"SIM_TARGET";
        esc.flash[0x7C00 - 32..0x7C00 - 32 + name.len()].copy_from_slice(name);
        // Vector table of the application: stack pointer in RAM, entry point in flash.
        esc.flash[0x1000..0x1004].copy_from_slice(&0x2000_1000u32.to_le_bytes());
        esc.flash[0x1004..0x1008].copy_from_slice(&0x0800_1101u32.to_le_bytes());
        let devinfo = esc.devinfo_struct();
        let at = DEVINFO_OFFSET as usize;
        esc.flash[at..at + devinfo.len()].copy_from_slice(&devinfo);
        esc
    }

    /// Flash bytes must be written in multiples of this, at addresses that are multiples of
    /// it, or program answers `C1` (`save_flash_nolib`: 1 on F051 (even), 4 on F421, 8 on
    /// G071/G431/L431). The default 1 is the plain overwrite.
    pub fn with_write_granule(mut self, granule: u32) -> Self {
        self.write_granule = granule;
        self
    }

    /// The `devinfo` struct of BL:244-286, packed, 27 bytes.
    fn devinfo_struct(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&0x5925_e3dau32.to_le_bytes());
        out.extend_from_slice(&0x4eb8_63d9u32.to_le_bytes());
        out.extend_from_slice(&self.device_info());
        out.push(27);
        out.push(self.shift as u8);
        let firmware_relative_start = self.app_add - MCU_FLASH_START;
        for value in [
            firmware_relative_start >> self.shift,
            self.file_name_add >> self.shift,
            self.eeprom_add >> self.shift,
            (self.eeprom_add + 48) >> self.shift,
        ] {
            out.extend_from_slice(&(value as u16).to_le_bytes());
        }
        out
    }

    /// `'4','7','1', pin code, flash size code, 6, 6, protocol version, 0x30` (BL:242).
    pub fn device_info(&self) -> [u8; 9] {
        [
            b'4',
            b'7',
            b'1',
            self.pin_code,
            self.size_code,
            0x06,
            0x06,
            self.protocol_version,
            0x30,
        ]
    }

    pub fn flash(&self) -> &[u8] {
        &self.flash
    }
    pub fn flash_mut(&mut self) -> &mut [u8] {
        &mut self.flash
    }
    /// Offset in `flash()` of the settings block.
    pub fn settings_offset(&self) -> usize {
        (self.eeprom_add - MCU_FLASH_START) as usize
    }
    /// Offset in `flash()` of the application.
    pub fn app_offset(&self) -> usize {
        (self.app_add - MCU_FLASH_START) as usize
    }
    /// `invalid_command`, the bootloader's count of bad commands in a row.
    pub fn invalid_command(&self) -> u16 {
        self.invalid_command
    }
    /// True once the bootloader has jumped to the application: the ESC is silent from then on.
    pub fn running(&self) -> bool {
        self.running
    }
    /// The bursts this ESC received, as they came (before any `corrupt_burst`).
    pub fn bursts(&self) -> &[Vec<u8>] {
        &self.bursts
    }
    /// The answers this ESC produced, before faults.
    pub fn answers(&self) -> &[Vec<u8>] {
        &self.answers
    }
    /// The address the ESC holds (0 = none), as an absolute address.
    pub fn address(&self) -> u32 {
        self.address
    }
    /// True while the ESC takes the next frame as a payload (BL:`incoming_payload_no_command`).
    pub fn expecting_payload(&self) -> bool {
        self.incoming_payload_no_command
    }

    fn read_flash(&self, address: u32, n: usize) -> Vec<u8> {
        (0..n as u32)
            .map(|i| {
                address
                    .checked_add(i)
                    .and_then(|a| a.checked_sub(MCU_FLASH_START))
                    .and_then(|offset| self.flash.get(offset as usize).copied())
                    .unwrap_or(0xFF)
            })
            .collect()
    }

    /// `save_flash_nolib`, minus the page erase: false when the part would refuse.
    fn save_flash(&mut self, data: &[u8], address: u32) -> bool {
        let granule = self.write_granule;
        if !address.is_multiple_of(granule) || !(data.len() as u32).is_multiple_of(granule) {
            return false;
        }
        let Some(offset) = address.checked_sub(MCU_FLASH_START) else {
            return false;
        };
        let offset = offset as usize;
        match self.flash.get_mut(offset..offset + data.len()) {
            Some(target) => {
                target.copy_from_slice(data);
                true
            }
            None => false,
        }
    }

    /// `jump()`, BL:423-476: refused (and the count cleared) unless the first byte of the
    /// settings block is 1 and the first two words of the application are in range.
    fn jump(&mut self) {
        if self.flash.get(self.settings_offset()).copied() != Some(1) {
            self.invalid_command = 0;
            return;
        }
        let app = self.app_offset();
        let word = |at: usize| -> u32 {
            self.flash
                .get(at..at + 4)
                .map_or(0, |b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        };
        let (stack, entry) = (word(app), word(app + 4));
        let ram_start = 0x2000_0000u32;
        let ram_end = ram_start + 64 * 1024;
        if stack < ram_start || stack > ram_end {
            self.invalid_command = 0;
            return;
        }
        if entry < self.app_add || entry > self.app_add + 256 * 1024 {
            self.invalid_command = 0;
            return;
        }
        self.running = true;
    }

    /// Queue the bytes of one answer: first byte readable after the turn-around bit time (once
    /// initialised, BL:520) and one byte time per byte; apply the next fault.
    fn send(&mut self, now: u64, spacing_us: u64, bytes: &[u8]) {
        self.answers.push(bytes.to_vec());
        let mut bytes = bytes.to_vec();
        let mut start = now + if self.initialized { BIT_US } else { 0 };
        match self.faults.pop_front().unwrap_or(Fault::Pass) {
            Fault::Pass => {}
            Fault::NoAnswer => return,
            Fault::Late(us) => start += us as u64,
            Fault::Truncated(k) => bytes.truncate(k),
            Fault::CorruptCrc => {
                if bytes.len() >= 3 {
                    let at = bytes.len() - 3;
                    bytes[at] ^= 0x01;
                }
            }
            Fault::WrongAck(code) => {
                if let Some(last) = bytes.last_mut() {
                    *last = code;
                }
            }
            Fault::Replace(other) => bytes = other,
        }
        for (i, byte) in bytes.into_iter().enumerate() {
            self.pending
                .push_back((start + (i as u64 + 1) * spacing_us, byte));
        }
    }

    fn send_ack(&mut self, now: u64, spacing: u64) {
        self.send(now, spacing, &[0x30]);
        self.invalid_command = 0; // BL:547
    }

    fn send_bad_ack(&mut self, now: u64, spacing: u64) {
        self.send(now, spacing, &[0xC1]);
        self.invalid_command += 1;
    }

    fn send_bad_crc_ack(&mut self, now: u64, spacing: u64) {
        self.send(now, spacing, &[0xC2]);
        self.invalid_command += 1;
    }

    fn send_device_info(&mut self, now: u64, spacing: u64) {
        let info = self.device_info();
        self.send(now, spacing, &info);
        self.initialized = true;
    }

    /// One burst from the host: `receiveBuffer()` (BL:1007-1048) then the main loop's jump test
    /// (BL:1278).
    fn receive_burst(&mut self, now: u64, spacing: u64, burst: &[u8]) {
        self.bursts.push(burst.to_vec());
        if self.running || burst.is_empty() {
            return;
        }
        let mut burst = burst.to_vec();
        match self.corrupt_incoming {
            Some((0, index, xor)) => {
                self.corrupt_incoming = None;
                if let Some(byte) = burst.get_mut(index) {
                    *byte ^= xor;
                }
            }
            Some((skip, index, xor)) => self.corrupt_incoming = Some((skip - 1, index, xor)),
            None => {}
        }
        let mut rx = [0u8; RX_BUFFER];
        let mut count = 0usize;
        for (i, &byte) in burst.iter().enumerate().take(RX_BUFFER) {
            if self.incoming_payload_no_command {
                if count == self.payload_size as usize + 2 {
                    break;
                }
                rx[i] = byte;
                count += 1;
            } else {
                rx[i] = byte;
                if i == 257 {
                    self.invalid_command += 20;
                }
            }
        }
        self.decode_input(now, spacing, &mut rx);
        if self.invalid_command > 100 {
            self.jump();
        }
    }

    /// `decodeInput()`, BL:581-817.
    fn decode_input(&mut self, now: u64, spacing: u64, rx: &mut [u8; RX_BUFFER]) {
        if self.incoming_payload_no_command {
            let len = self.payload_size as usize;
            if check_crc(rx, len) {
                self.payload = [0; 256];
                self.payload[..len].copy_from_slice(&rx[..len]);
                self.send_ack(now, spacing);
                self.incoming_payload_no_command = false;
            } else {
                self.send_bad_crc_ack(now, spacing);
            }
            return;
        }

        let cmd = rx[0];

        if rx[16] == 0x7d {
            if rx[8] == 13 && rx[9] == 66 {
                self.send_device_info(now, spacing);
                rx[20] = 0;
            }
            return;
        }
        if rx[20] == 0x7d && rx[12] == 13 && rx[13] == 66 {
            self.send_device_info(now, spacing);
            rx[20] = 0;
            return;
        }
        if rx[40] == 0x7d && rx[32] == 13 && rx[33] == 66 {
            self.send_device_info(now, spacing);
            rx[20] = 0;
            return;
        }

        if cmd == 0x00 && rx[1] == 0 && rx[2] == 0 && rx[3] == 0 {
            // CMD_RUN: no return here; main.c falls through to the bad-command answer below.
            self.invalid_command = 101;
        }

        if cmd == 0x01 {
            if !check_crc(rx, 2) {
                self.send_bad_crc_ack(now, spacing);
                return;
            }
            if self.address < self.app_add {
                self.send_bad_ack(now, spacing);
                return;
            }
            if self.address == self.eeprom_add && self.payload_size > 2 {
                self.payload[2] = BOOTLOADER_VERSION;
            }
            let size = self.payload_size as usize;
            let data = self.payload[..size.min(256)].to_vec();
            if !self.save_flash(&data, self.address) {
                self.send_bad_ack(now, spacing);
            } else {
                self.send_ack(now, spacing);
            }
            return;
        }

        if cmd == 0xFF {
            if !check_crc(rx, 4) {
                self.send_bad_crc_ack(now, spacing);
                return;
            }
            self.invalid_command = 0;
            self.address = (rx[2] as u32) << 8 | rx[3] as u32;
            if self.address == 0x20 {
                self.address = self.eeprom_add;
            } else if self.address == 0x21 {
                self.address = self.eeprom_add - 32;
            } else if self.address == 0x22 {
                self.address = self.continue_address;
            } else if self.address == 0x23 {
                self.address = MCU_FLASH_START + DEVINFO_OFFSET;
            } else if self.address < 1024 {
                self.send_bad_ack(now, spacing);
                return;
            } else {
                self.address = MCU_FLASH_START + (self.address << self.shift);
            }
            self.send_ack(now, spacing);
            return;
        }

        if cmd == 0xFE {
            if !check_crc(rx, 4) {
                self.send_bad_crc_ack(now, spacing);
                return;
            }
            self.payload_size = if rx[2] == 0x01 { 256 } else { rx[3] as u16 };
            self.incoming_payload_no_command = true;
            return;
        }

        if cmd == 0xFD {
            if !check_crc(rx, 2) {
                self.send_bad_crc_ack(now, spacing);
                return;
            }
            self.send(now, spacing, &[0xC1]);
            return;
        }

        if cmd == 0x02 {
            if !check_crc(rx, 2) {
                self.send_bad_crc_ack(now, spacing);
                return;
            }
            if self.address < self.app_add {
                self.send_bad_ack(now, spacing);
                return;
            }
            self.send_ack(now, spacing);
            return;
        }

        if cmd == 0x03 {
            if !check_crc(rx, 2) {
                self.send_bad_crc_ack(now, spacing);
                return;
            }
            if self.address == 0 {
                self.send_bad_ack(now, spacing);
                return;
            }
            let n = if rx[1] == 0 { 256 } else { rx[1] as usize };
            let mut reply = self.read_flash(self.address, n);
            let crc = crc16(&reply);
            reply.push((crc & 0xFF) as u8);
            reply.push((crc >> 8) as u8);
            reply.push(0x30);
            self.send(now, spacing, &reply);
            self.continue_address = self.address + n as u32;
            self.address = 0;
            return;
        }

        self.send(now, spacing, &[0xC1]);
        self.invalid_command += 1;
    }
}

/// Up to four ESCs behind the `EscLines` trait.
#[derive(Debug)]
pub struct Sim {
    escs: Vec<EscModel>,
    selected: Option<usize>,
    now_us: u64,
    spacing_us: u64,
    max_per_read: Option<usize>,
    break_on: bool,
    line_errors: VecDeque<LineErrorAt>,
    log: Vec<Wire>,
}

/// Which call the next injected line error hits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineErrorAt {
    NextWrite,
    NextRead,
    /// Skip this many reads, then fail the next one.
    ReadAfter(usize),
}

impl Sim {
    pub fn new(escs: Vec<EscModel>) -> Self {
        assert!(escs.len() <= 4);
        Self {
            escs,
            selected: None,
            now_us: 0,
            spacing_us: BYTE_US,
            max_per_read: None,
            break_on: false,
            line_errors: VecDeque::new(),
            log: Vec::new(),
        }
    }

    /// One F421-like ESC on line 0, pin code 0x14.
    pub fn single() -> Self {
        Self::new(vec![EscModel::f421(0x14)])
    }

    /// Four F421-like ESCs with pin codes 0x10 + line.
    pub fn four() -> Self {
        Self::new((0..4).map(|i| EscModel::f421(0x10 + i)).collect())
    }

    /// All bytes of an answer become readable together (a line that buffers).
    pub fn burst(mut self) -> Self {
        self.spacing_us = 0;
        self
    }

    /// Every read returns at most this many bytes, however many are ready.
    pub fn with_max_per_read(mut self, n: usize) -> Self {
        self.max_per_read = Some(n);
        self
    }

    pub fn esc(&self, line: usize) -> &EscModel {
        &self.escs[line]
    }
    pub fn esc_mut(&mut self, line: usize) -> &mut EscModel {
        &mut self.escs[line]
    }
    pub fn now_us(&self) -> u64 {
        self.now_us
    }
    /// Let virtual time pass (late answers arrive).
    pub fn advance(&mut self, us: u64) {
        self.now_us += us;
    }
    pub fn break_on(&self) -> bool {
        self.break_on
    }
    pub fn log(&self) -> &[Wire] {
        &self.log
    }
    pub fn clear_log(&mut self) {
        self.log.clear();
    }
    /// The bursts written while `line` was active, per line.
    pub fn written(&self, line: usize) -> &[Vec<u8>] {
        self.escs[line].bursts()
    }
    /// The faults for the next answers of `line`.
    pub fn faults(&mut self, line: usize, faults: impl IntoIterator<Item = Fault>) {
        self.escs[line].faults.extend(faults);
    }
    pub fn fault(&mut self, line: usize, fault: Fault) {
        self.faults(line, [fault]);
    }
    /// XOR `xor` into byte `index` of a burst `line` receives (a damaged request): the one after
    /// `skip` more bursts have gone by (0 = the next one).
    pub fn corrupt_burst(&mut self, line: usize, skip: usize, index: usize, xor: u8) {
        self.escs[line].corrupt_incoming = Some((skip, index, xor));
    }
    pub fn inject_line_error(&mut self, at: LineErrorAt) {
        self.line_errors.push_back(at);
    }

    fn take_line_error(&mut self, write: bool) -> bool {
        match self.line_errors.front().copied() {
            Some(LineErrorAt::NextWrite) if write => {
                self.line_errors.pop_front();
                true
            }
            Some(LineErrorAt::NextRead) if !write => {
                self.line_errors.pop_front();
                true
            }
            Some(LineErrorAt::ReadAfter(0)) if !write => {
                self.line_errors.pop_front();
                true
            }
            Some(LineErrorAt::ReadAfter(n)) if !write => {
                self.line_errors[0] = LineErrorAt::ReadAfter(n - 1);
                false
            }
            _ => false,
        }
    }

    fn active(&self) -> Result<usize, SimError> {
        self.selected.ok_or(SimError::NoLineSelected)
    }
}

impl EscLines for Sim {
    type Error = SimError;

    fn select(&mut self, line: u8) -> Result<(), SimError> {
        self.log.push(Wire::Select(line));
        if (line as usize) >= self.escs.len() {
            return Err(SimError::NoSuchLine(line));
        }
        self.selected = Some(line as usize);
        Ok(())
    }

    async fn write(&mut self, bytes: &[u8]) -> Result<(), SimError> {
        self.log.push(Wire::Write(bytes.to_vec()));
        if self.take_line_error(true) {
            return Err(SimError::Injected);
        }
        let line = self.active()?;
        self.now_us += BYTE_US * bytes.len() as u64;
        if self.break_on {
            return Ok(());
        }
        let (now, spacing) = (self.now_us, self.spacing_us);
        self.escs[line].receive_burst(now, spacing, bytes);
        Ok(())
    }

    async fn read(&mut self, buf: &mut [u8], timeout_us: u32) -> Result<usize, SimError> {
        self.log.push(Wire::Read {
            cap: buf.len(),
            timeout_us,
        });
        if self.take_line_error(false) {
            return Err(SimError::Injected);
        }
        let line = self.active()?;
        let deadline = self.now_us + timeout_us as u64;
        let limit = self.max_per_read.unwrap_or(usize::MAX).min(buf.len());
        let esc = &mut self.escs[line];
        loop {
            let ready = esc
                .pending
                .iter()
                .take_while(|(at, _)| *at <= self.now_us)
                .count();
            if ready > 0 && limit > 0 {
                let n = ready.min(limit);
                for slot in buf.iter_mut().take(n) {
                    if let Some((_, byte)) = esc.pending.pop_front() {
                        *slot = byte;
                    }
                }
                return Ok(n);
            }
            match esc.pending.front() {
                Some((at, _)) if *at <= deadline && limit > 0 => self.now_us = self.now_us.max(*at),
                _ => {
                    self.now_us = deadline;
                    return Ok(0);
                }
            }
        }
    }

    fn set_break(&mut self, on: bool) -> Result<(), SimError> {
        self.log.push(Wire::Break(on));
        self.break_on = on;
        Ok(())
    }
}
