//! the emulated console, CPU, memory, HLE kernel, HLE services and GPU, and
//! the loop that drives them.

pub mod camera;
pub mod cheats;
pub mod enhancements;
pub mod cro;
pub mod download_play;
pub mod hints;
pub mod idle;
pub mod spin;
pub mod kernel;
pub mod loader;
pub mod memory;
pub mod microphone;
pub mod mods;
pub mod plugin_3gx;
pub mod replay;
pub mod room;
pub mod recompiled;
pub mod services;
pub mod state;

use std::collections::{BTreeMap, BTreeSet};

use zakuro_common::memory_map::*;
use zakuro_common::ConsoleModel;
use zakuro_cpu::{Cpu, Exit};
use zakuro_fs::Title;
use zakuro_gpu::{Gpu, GpuMemory, Renderer, SoftwareRenderer};

use kernel::thread::{ThreadId, WaitResult, WaitSyscall, THREAD_EXIT_MAGIC};
use kernel::Kernel;
use memory::{Memory, MemoryState, Permission};
use services::hid::InputState;
use services::ServiceState;