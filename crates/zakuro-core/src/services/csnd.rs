//! CSND sound-channel HLE used by 3GX plugins.
//!
//! Models the 32-channel CSND PCM8/PCM16/IMA-ADPCM mixer and the csnd:SND
//! shared-memory command-list interface. PSG/noise generation and capture
//! DMA remain unsupported; capture registers and allocation commands are
//! retained so clients can initialize and query the service safely.

use std::collections::BTreeMap;

use crate::kernel::ipc::{CommandBuffer, Header};
use crate::kernel::object::{KObject, ObjectId, SharedMemory};
use crate::kernel::sync;
use crate::memory::{Memory, MemoryRegion, PhysicalBlock};
use crate::System;

const CHANNELS: usize = 32;
const CAPTURES: usize = 2;
const COMMAND_SIZE: u32 = 0x20;
const CHANNEL_STATE_SIZE: u32 = 0x0C;
const CAPTURE_STATE_SIZE: u32 = 8;
const CLIENT_CHANNELS: u32 = 0xFFFF_FF00;
const LIST_END: u16 = 0xFFFF;
const INVALID_STATE: u32 = 0xC8A0_B7FF;
const ALREADY_INITIALIZED: u32 = 0xE0A0_183B;
const INVALID_ADDRESS: u32 = 0xE0E3_FFF6;

fn start_channel(ch: &mut Channel) {
    ch.block_address = ch.start;
    ch.block_size = ch.size;
    ch.position = 0;
    ch.high_nibble = false;
    ch.holding = false;
    ch.counter = 0;
    ch.previous = 0;
    ch.current = 0;
    ch.predictor = ch.adpcm_state as u16 as i16;
    ch.step_index = ((ch.adpcm_state >> 16) & 0x7F).min(88) as u8;
}

fn stop_channel(ch: &mut Channel) {
    ch.adpcm_state = ch.predictor as u16 as u32 | ((ch.step_index as u32) << 16);
    ch.cnt &= !0x8000;
}

#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
struct Channel {
    cnt: u16,
    timer: u16,
    volume_left: u16,
    volume_right: u16,
    capture_volume_left: u16,
    capture_volume_right: u16,
    start: u32,
    size: u32,
    loop_address: u32,
    adpcm_state: u32,
    loop_adpcm_state: u32,
    block_address: u32,
    block_size: u32,
    position: u32,
    high_nibble: bool,
    holding: bool,
    counter: u32,
    previous: i16,
    current: i16,
    predictor: i16,
    step_index: u8,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct Client {
    shared_block: Option<PhysicalBlock>,
    shared_object: Option<ObjectId>,
    mutex_object: Option<ObjectId>,
    shared_handle: u32,
    mutex_handle: u32,
    master_offset: u32,
    channel_offset: u32,
    capture_offset: u32,
    direct_offset: u32,
    channel_mask: u32,
    capture_mask: u8,
}

impl Default for Client {
    fn default() -> Self {
        Self {
            shared_block: None, shared_object: None, mutex_object: None,
            shared_handle: 0, mutex_handle: 0,
            master_offset: 0, channel_offset: 0, capture_offset: 0, direct_offset: 0,
            channel_mask: 0, capture_mask: 0,
        }
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct CsndState {
    channels: [Channel; CHANNELS],
    clients: BTreeMap<u32, Client>,
    next_session: u32,
    capture_in_use: [bool; CAPTURES],
    master_volume: u16,
}

impl Default for CsndState {
    fn default() -> Self {
        Self {
            channels: [Channel::default(); CHANNELS],
            clients: BTreeMap::new(),
            next_session: 1,
            capture_in_use: [false; CAPTURES],
            master_volume: 0x8000,
        }
    }
}

impl CsndState {
    pub fn open_session(&mut self) -> u32 {
        let id = self.next_session.max(1);
        self.next_session = id.wrapping_add(1).max(1);
        self.clients.entry(id).or_default();
        id
    }

    fn client(&mut self, session: u32) -> &mut Client {
        self.clients.entry(session).or_default()
    }

    fn start(&mut self, index: usize) {
        start_channel(&mut self.channels[index]);
    }

    fn stop(&mut self, index: usize) {
        stop_channel(&mut self.channels[index]);
    }

    fn sample(&mut self, memory: &mut Memory, index: usize) -> i16 {
        const STEPS: [i32; 89] = [
            7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767
        ];
        const INDEX: [i32; 8] = [-1,-1,-1,-1,2,4,6,8];
        let ch = &mut self.channels[index];
        let address = ch.block_address.wrapping_add(ch.position);
        let format = (ch.cnt >> 12) & 3;
        match format {
            0 => {
                let mut b = [0];
                let _ = memory.read_physical(address, &mut b);
                ch.position = ch.position.wrapping_add(1);
                (b[0] as i8 as i16) << 8
            }
            1 => {
                let mut b = [0; 2];
                let _ = memory.read_physical(address, &mut b);
                ch.position = ch.position.wrapping_add(2);
                i16::from_le_bytes(b)
            }
            2 => {
                let mut b = [0];
                let _ = memory.read_physical(address, &mut b);
                let nibble = if ch.high_nibble { b[0] >> 4 } else { b[0] & 0xF };
                if ch.high_nibble { ch.position = ch.position.wrapping_add(1); }
                ch.high_nibble = !ch.high_nibble;
                let step = STEPS[ch.step_index as usize];
                let mut diff = step >> 3;
                if nibble & 1 != 0 { diff += step >> 2; }
                if nibble & 2 != 0 { diff += step >> 1; }
                if nibble & 4 != 0 { diff += step; }
                let next = if nibble & 8 != 0 { ch.predictor as i32 - diff } else { ch.predictor as i32 + diff };
                ch.predictor = next.clamp(-0x7FFF, 0x7FFF) as i16;
                ch.step_index = (ch.step_index as i32 + INDEX[(nibble & 7) as usize]).clamp(0, 88) as u8;
                ch.predictor
            }
            _ => 0, // PSG/noise is not implemented by Azahar's software mixer either.
        }
    }

    fn step(&mut self, memory: &mut Memory, index: usize) {
        let (loop_mode, position, block_size) = {
            let ch = &self.channels[index];
            ((ch.cnt >> 10) & 3, ch.position, ch.block_size)
        };
        if loop_mode != 0 && position >= block_size {
            if loop_mode == 2 {
                let hold = self.channels[index].cnt & 0x80 != 0;
                stop_channel(&mut self.channels[index]);
                self.channels[index].holding = hold;
                return;
            }
            {
                let ch = &mut self.channels[index];
                ch.block_address = ch.loop_address;
                if loop_mode == 1 { ch.block_size = ch.size; }
                ch.position = 0;
                ch.high_nibble = false;
                if ch.loop_adpcm_state & 0x8000_0000 != 0 {
                    ch.predictor = ch.loop_adpcm_state as u16 as i16;
                    ch.step_index = ((ch.loop_adpcm_state >> 16) & 0x7F).min(88) as u8;
                }
            }
            if self.channels[index].block_size == 0 {
                stop_channel(&mut self.channels[index]);
                return;
            }
        }
        let sample = self.sample(memory, index);
        let ch = &mut self.channels[index];
        ch.previous = ch.current;
        ch.current = sample;
    }

    pub fn render_frame(&mut self, memory: &mut Memory, output: &mut Vec<[i16; 2]>) {
        let mut mixed = [[0i16; 2]; 160];
        for frame in &mut mixed {
            let mut left = 0i64;
            let mut right = 0i64;
            for index in 0..CHANNELS {
                if self.channels[index].cnt & 0x8000 == 0 {
                    let ch = &self.channels[index];
                    if ch.holding {
                        left += (ch.current as i64 * ch.volume_left as i64) >> 15;
                        right += (ch.current as i64 * ch.volume_right as i64) >> 15;
                    }
                    continue;
                }
                if self.channels[index].cnt & 0x4000 != 0 {
                    let period = (0x1_0000u32 - self.channels[index].timer as u32).max(0x42);
                    self.channels[index].counter += 2048;
                    loop {
                        if self.channels[index].counter < period || self.channels[index].cnt & 0x8000 == 0 {
                            break;
                        }
                        self.channels[index].counter -= period;
                        self.step(memory, index);
                    }
                }
                let ch = &self.channels[index];
                if ch.cnt & 0x8000 == 0 {
                    if ch.holding {
                        left += (ch.current as i64 * ch.volume_left as i64) >> 15;
                        right += (ch.current as i64 * ch.volume_right as i64) >> 15;
                    }
                    continue;
                }
                let sample = if ch.cnt & 0x40 != 0 {
                    let period = (0x1_0000u32 - ch.timer as u32).max(0x42) as i64;
                    let delta = ch.current as i64 - ch.previous as i64;
                    ch.previous as i64 + delta * ch.counter as i64 / period
                } else { ch.current as i64 };
                left += (sample * ch.volume_left as i64) >> 15;
                right += (sample * ch.volume_right as i64) >> 15;
            }
            frame[0] = ((left * self.master_volume as i64) >> 15).clamp(-32768, 32767) as i16;
            frame[1] = ((right * self.master_volume as i64) >> 15).clamp(-32768, 32767) as i16;
        }
        if output.len() < mixed.len() {
            output.resize(mixed.len(), [0, 0]);
        }
        let start = output.len() - mixed.len();
        for (dest, source) in output[start..].iter_mut().zip(mixed) {
            dest[0] = dest[0].saturating_add(source[0]);
            dest[1] = dest[1].saturating_add(source[1]);
        }
    }fn execute_command(system: &mut System, client: &Client, command: u16, p: [u32; 6]) {
    let index = (p[0] & 0x1F) as usize;
    if command < 0x100 && client.channel_mask & (1 << index) == 0 { return; }
    if (0x100..0x200).contains(&command) && index >= CAPTURES { return; }
    let busy = system.services.csnd.channels[index].cnt & 0x8000 != 0;
    match command {
        0x000 => {
            let ch = &mut system.services.csnd.channels[index];
            if p[1] != 0 { ch.cnt |= 0x8000; } else { ch.cnt &= !0x8000; ch.holding = false; }
        }
        0x001 => {
            if p[1] != 0 {
                let ch = &mut system.services.csnd.channels[index];
                if ch.cnt & 0x8000 == 0 { start_channel(ch); }
                ch.cnt |= 0xC000;
            } else {
                system.services.csnd.channels[index].cnt &= !0x4000;
            }
        }
        0x002 if !busy => {
            let ch = &mut system.services.csnd.channels[index];
            ch.cnt = (ch.cnt & !0x3000) | (((p[1] & 0xFF) as u16) << 12);
        }
        0x003 => {
            let ch = &mut system.services.csnd.channels[index];
            ch.loop_address = p[1]; ch.size = p[2] & 0x07FF_FFFF;
        }
        0x004 if !busy => {
            let ch = &mut system.services.csnd.channels[index];
            ch.cnt = (ch.cnt & !0x0C00) | (((p[1] & 3) as u16) << 10);
        }
        0x005 => {
            let ch = &mut system.services.csnd.channels[index];
            if p[1] != 0 { ch.cnt |= 0x80; } else { ch.cnt &= !0x80; }
        }
        0x006 if !busy => {
            let ch = &mut system.services.csnd.channels[index];
            if p[1] != 0 { ch.cnt |= 0x40; } else { ch.cnt &= !0x40; }
        }
        0x008 => system.services.csnd.channels[index].timer = (0x1_0000u32.wrapping_sub(p[1])) as u16,
        0x009 => {
            let ch = &mut system.services.csnd.channels[index];
            ch.volume_left = (p[1] as u16).min(0x8000);
            ch.volume_right = ((p[1] >> 16) as u16).min(0x8000);
            ch.capture_volume_left = p[2] as u16;
            ch.capture_volume_right = (p[2] >> 16) as u16;
        }
        0x00A => {
            let ch = &mut system.services.csnd.channels[index];
            ch.start = p[1]; ch.size = p[2] & 0x07FF_FFFF; ch.adpcm_state = 0;
        }
        0x00B => {
            let ch = &mut system.services.csnd.channels[index];
            ch.adpcm_state = (p[1] & 0xFFFF) | ((p[2] & 0xFF) << 16); ch.loop_adpcm_state = 0;
        }
        0x00C => system.services.csnd.channels[index].loop_adpcm_state = (p[1] & 0xFFFF) | ((p[2] & 0xFF) << 16) | 0x8000_0000,
        0x00D => {
            let ch = &mut system.services.csnd.channels[index];
            if p[1] == 0 { ch.loop_adpcm_state &= !0x8000_0000; } else { ch.loop_adpcm_state |= 0x8000_0000; }
        }
        0x00E => {
            let flags = p[0];
            let ch = &mut system.services.csnd.channels[index];
            ch.cnt = 0;
            ch.timer = (0x1_0000u32.wrapping_sub(flags >> 16)) as u16;
            ch.capture_volume_left = p[2] as u16;
            ch.capture_volume_right = (p[2] >> 16) as u16;
            ch.loop_address = p[4]; ch.size = p[5] & 0x07FF_FFFF; ch.start = p[3];
            ch.adpcm_state = 0; ch.loop_adpcm_state = 0;
            ch.cnt = (((flags >> 12) & 3) as u16) << 12
                | (((flags >> 10) & 3) as u16) << 10
                | if flags & (1 << 6) != 0 { 0x40 } else { 0 };
            ch.volume_left = (p[1] as u16).min(0x8000);
            ch.volume_right = ((p[1] >> 16) as u16).min(0x8000);
            if flags & (1 << 14) != 0 {
                start_channel(ch);
                ch.cnt |= 0xC000;
            }
        }
        0x200 => {}
        0x300 => write_client_state(system, client),
        _ => {}
    }
}

