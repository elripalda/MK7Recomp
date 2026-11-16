//! 3GX plugin memory mapper and startup trampoline.
//!
//! The 412-byte ARM bootloader payload is derived from Azahar's
//! plugin_3gx_bootloader.h (GPL-2.0-or-later; original payload MIT-licensed
//! by The Pixellizer Group). The mapping and patching code below is a Rust
//! adaptation of the documented Citra/Azahar 3GX loader behavior.

use std::path::{Path, PathBuf};

use zakuro_common::memory_map::{PAGE_SIZE, FCRAM_PADDR};
use zakuro_fs::Title;

use crate::memory::{MemoryRegion, MemoryState, Permission, PhysicalBlock};
use crate::plugin_3gx::{Plugin3gx, Compatibility, PLUGIN_HEADER_ADDRESS, PLUGIN_HEAP_ADDRESS};
use crate::System;

const BOOTLOADER_ADDRESS: u32 = PLUGIN_HEADER_ADDRESS - 0x1000;
const BOOTLOADER_SIZE: u32 = 0x1000;
const PLUGIN_HEADER_SIZE: u32 = 0x100;
const FRAMEBUFFER_SIZE: u32 = 0xA9000;
const BOOTLOADER: [u8; 412] = [
0xff, 0x1f, 0x2d, 0xe9, 0x00, 0x00, 0x0f, 0xe1, 0x01, 0x00, 0x2d, 0xe9, 0xf0, 0x00, 0x8f, 0xe2,
 0x04, 0x10, 0x90, 0xe5, 0x00, 0x00, 0x90, 0xe5, 0xd4, 0x20, 0x8f, 0xe2, 0x0f, 0xe0, 0xa0, 0xe1,
 0xf4, 0xf0, 0x8f, 0xe2, 0xe0, 0x10, 0x8f, 0xe2, 0x00, 0x10, 0x91, 0xe5, 0x01, 0x00, 0x50, 0xe1,
 0xd8, 0x00, 0x8f, 0xe2, 0x00, 0x00, 0x90, 0xe5, 0x01, 0x10, 0xa0, 0x03, 0x00, 0x10, 0xa0, 0x13,
 0x00, 0x10, 0x80, 0xe5, 0x03, 0x00, 0x00, 0x1f, 0xc8, 0x00, 0x8f, 0xe2, 0x00, 0x00, 0xd0, 0xe5,
 0x01, 0x00, 0x50, 0xe3, 0x09, 0x00, 0x00, 0x0a, 0x78, 0x40, 0x9f, 0xe5, 0x78, 0x50, 0x9f, 0xe5,
 0x40, 0x60, 0xa0, 0xe3, 0x00, 0x50, 0x84, 0xe5, 0x70, 0x00, 0x9f, 0xe5, 0x00, 0x10, 0xa0, 0xe3,
 0x0a, 0x00, 0x00, 0xef, 0x01, 0x60, 0x56, 0xe2, 0xf9, 0xff, 0xff, 0x1a, 0x00, 0x60, 0x84, 0xe5,
 0x5c, 0x00, 0x9f, 0xe5, 0x01, 0x10, 0xa0, 0xe3, 0xb3, 0x00, 0x00, 0xef, 0x54, 0x00, 0x8f, 0xe2,
 0x58, 0x10, 0x8f, 0xe2, 0x00, 0x10, 0x91, 0xe5, 0x00, 0x20, 0x90, 0xe5, 0x00, 0x20, 0x81, 0xe5,
 0x04, 0x20, 0x90, 0xe5, 0x04, 0x20, 0x81, 0xe5, 0x94, 0x00, 0x00, 0xef, 0x34, 0x00, 0x8f, 0xe2,
 0x04, 0x00, 0x2d, 0xe5, 0x58, 0x50, 0x8f, 0xe2, 0x00, 0x50, 0x95, 0xe5, 0x35, 0xff, 0x2f, 0xe1,
 0x04, 0xd0, 0x8d, 0xe2, 0x01, 0x00, 0xbd, 0xe8, 0x00, 0xf0, 0x29, 0xe1, 0xff, 0x1f, 0xbd, 0xe8,
 0x18, 0xe0, 0x8f, 0xe2, 0x00, 0xf0, 0x9e, 0xe5, 0x04, 0x22, 0x20, 0x90, 0x33, 0x99, 0xff, 0x01,
 0x40, 0x4b, 0xff, 0x00, 0x01, 0x80, 0xff, 0xff, 0x00, 0x00, 0xad, 0xde, 0x01, 0x00, 0xad, 0xde,
 0x02, 0x00, 0xad, 0xde, 0x03, 0x00, 0xad, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0xad, 0xde, 0x05, 0x00, 0xad, 0xde, 0x06, 0x00, 0xad, 0xde,
 0x07, 0x00, 0xad, 0xde, 0x08, 0x00, 0xad, 0xde, 0x09, 0x00, 0xad, 0xde, 0x0a, 0x00, 0xad, 0xde,
 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3,
 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3,
 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3,
 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3,
 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3,
 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3,
 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3,
 0x00, 0xf0, 0x20, 0xe3, 0x00, 0xf0, 0x20, 0xe3, 0x1e, 0xff, 0x2f, 0xe1,
];

#[derive(Debug, thiserror::Error)]
pub enum LoadError {
    #[error("could not read plugin: {0}")]
    Io(#[from] std::io::Error),
    #[error("invalid 3GX plugin: {0}")]
    Parse(#[from] crate::plugin_3gx::ParseError),
    #[error("plugin is not marked compatible with emulators")]
    Incompatible,
    #[error("plugin does not target this title")]
    WrongTitle,
    #[error("plugin has no executable checksum function")]
    MissingChecksumFunction,
    #[error("plugin memory region is already mapped")]
    AddressConflict,
    #[error("not enough contiguous FCRAM for plugin")]
    OutOfMemory,
    #[error("could not map plugin memory")]
    MapFailed,
}

/// Looks for title-specific plugins first, then a default plugin. Returns the
/// bootloader entry point when a plugin was successfully installed.
pub fn load_for_title(system: &mut System, title: &Title, game_entry: u32) -> Option<u32> {
    if !system.services.plgldr.enabled || system.services.plgldr.plugin_loaded {
        return None;
    }
    let data_dir = system.config.data_dir.as_ref()?;
    let root = data_dir.join("plugins");
    let title_dir = root.join(format!("{:016X}", title.program_id()));
    let mut candidates: Vec<PathBuf> = std::fs::read_dir(&title_dir)
        .ok()
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| path.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("3gx")))
        .collect();
    candidates.sort();
    for path in candidates {
        match load_path(system, title, game_entry, &path, false) {
            Ok(()) => return Some(BOOTLOADER_ADDRESS),
            Err(error) => log::warn!("3GX plugin {} was not loaded: {error}", path.display()),
        }
    }
    let default = root.join("default.3gx");
    if default.is_file() {
        match load_path(system, title, game_entry, &default, true) {
            Ok(()) => return Some(BOOTLOADER_ADDRESS),
            Err(error) => log::warn!("default 3GX plugin {} was not loaded: {error}", default.display()),
        }
    }
    None
}

fn load_path(system: &mut System, title: &Title, game_entry: u32, path: &Path, is_default: bool) -> Result<(), LoadError> {
    let bytes = std::fs::read(path)?;
    let plugin = Plugin3gx::parse(&bytes)?;
    if !plugin.flags.compatibility.supports_emulator() {
        return Err(LoadError::Incompatible);
    }
    if !plugin.supports_title(title.program_id()) {
        return Err(LoadError::WrongTitle);
    }
    let load_func = plugin.executable_check.as_deref().ok_or(LoadError::MissingChecksumFunction)?;
    if load_func.len() > 32 {
        return Err(LoadError::MissingChecksumFunction);
    }

    let block_size = plugin.flags.memory_region_size();
    let exe_size = (PLUGIN_HEADER_SIZE as u64
        + plugin.code.len() as u64
        + plugin.rodata.len() as u64
        + plugin.data.len() as u64
        + plugin.bss_size as u64
        + PAGE_SIZE as u64) & !(PAGE_SIZE as u64 - 1);
    let boot_offset = u32::try_from(exe_size).map_err(|_| LoadError::OutOfMemory)?;
    let heap_offset = boot_offset.checked_add(BOOTLOADER_SIZE).ok_or(LoadError::OutOfMemory)?;
    if heap_offset >= block_size {
        return Err(LoadError::OutOfMemory);
    }
    let heap_size = block_size - heap_offset;

    if system.memory.mapping_at(PLUGIN_HEAP_ADDRESS).is_some()
        || system.memory.mapping_at(BOOTLOADER_ADDRESS).is_some()
        || system.memory.mapping_at(PLUGIN_HEADER_ADDRESS).is_some()
    {
        return Err(LoadError::AddressConflict);
    }

    // Default Luma/Citra behavior allocates from SYSTEM. Strategy 1 (mode 3)
    // uses the application's region, matching Azahar's NewProd path.
    let region = if system.services.plgldr.memory_strategy == 1 {
        MemoryRegion::Application
    } else {
        MemoryRegion::System
    };
    let block = system.memory.phys.allocate(region, block_size).ok_or(LoadError::OutOfMemory)?;
    if !system.memory.zero_physical(block.addr, block.size) {
        system.memory.phys.free(block);
        return Err(LoadError::MapFailed);
    }

    let private = plugin.flags.use_private_memory;
    let state = if private { MemoryState::Private } else { MemoryState::Shared };
    let rwx = Permission::READ | Permission::WRITE | Permission::EXECUTE;

    system.memory.map(PLUGIN_HEADER_ADDRESS, block.addr, exe_size as u32, rwx, state);
    system.memory.map(BOOTLOADER_ADDRESS, block.addr + boot_offset, BOOTLOADER_SIZE, rwx, MemoryState::Private);
    system.memory.map(PLUGIN_HEAP_ADDRESS, block.addr + heap_offset, heap_size, rwx, state);

    let mut header = [0u8; PLUGIN_HEADER_SIZE as usize];
    put_u32(&mut header, 0x00, u32::from_le_bytes(*b"3GX$"));
    put_u32(&mut header, 0x04, plugin.version);
    put_u32(&mut header, 0x08, PLUGIN_HEAP_ADDRESS);
    put_u32(&mut header, 0x0C, heap_size);
    put_u32(&mut header, 0x10, exe_size as u32);
    put_u32(&mut header, 0x14, u32::from(is_default));
    put_u32(&mut header, 0x18, PLUGIN_HEADER_ADDRESS - 4);
    put_u32(&mut header, 0x1C, PLUGIN_HEADER_ADDRESS - 8);
    for (index, value) in system.services.plgldr.config.iter().enumerate() {
        put_u32(&mut header, 0x80 + index * 4, *value);
    }
    system.memory.write_bytes(PLUGIN_HEADER_ADDRESS, &header);
    system.memory.write_bytes(PLUGIN_HEADER_ADDRESS + PLUGIN_HEADER_SIZE, &plugin.code);
    system.memory.write_bytes(PLUGIN_HEADER_ADDRESS + PLUGIN_HEADER_SIZE + plugin.code.len() as u32, &plugin.rodata);
    system.memory.write_bytes(PLUGIN_HEADER_ADDRESS + PLUGIN_HEADER_SIZE + plugin.code.len() as u32 + plugin.rodata.len() as u32, &plugin.data);

    let mut original = [0u8; 8];
    system.memory.read_bytes(game_entry, &mut original);
    let mut boot = BOOTLOADER;
    let checksum_size = (plugin.code.len() + plugin.rodata.len() + plugin.data.len()) as u32;
    patch_bootloader(
        &mut boot,
        u32::from_le_bytes(original[0..4].try_into().unwrap()),
        u32::from_le_bytes(original[4..8].try_into().unwrap()),
        game_entry,
        &plugin.executable_check_args,
        PLUGIN_HEADER_ADDRESS + PLUGIN_HEADER_SIZE,
        PLUGIN_HEADER_ADDRESS + PLUGIN_HEADER_SIZE + checksum_size,
        plugin.executable_checksum,
        PLUGIN_HEADER_ADDRESS - 0xC,
        PLUGIN_HEADER_ADDRESS + PLUGIN_HEADER_SIZE,
        u32::from(system.services.plgldr.no_flash),
        load_func,
    );
    if !system.memory.write_physical(block.addr + boot_offset, &boot) {
        system.memory.unmap(PLUGIN_HEADER_ADDRESS, exe_size as u32);
        system.memory.unmap(BOOTLOADER_ADDRESS, BOOTLOADER_SIZE);
        system.memory.unmap(PLUGIN_HEAP_ADDRESS, heap_size);
        system.memory.phys.free(block);
        return Err(LoadError::MapFailed);
    }

    // Replace the game's entry with an absolute branch to the ARM trampoline.
    // The trampoline restores the two original instructions before calling
    // the plugin and finally resumes the game at its original entry.
    system.memory.write32_privileged(game_entry, 0xE51F_F004);
    system.memory.write32_privileged(game_entry + 4, BOOTLOADER_ADDRESS);

    system.services.plgldr.plugin_loaded = true;
    system.services.plgldr.plugin_path = path.to_string_lossy().into_owned();
    system.services.plgldr.is_default_path = is_default;
    system.services.plgldr.plugin_process_id = system.kernel.process_id;
    system.services.plgldr.memory_region = if region == MemoryRegion::Application { 1 } else { 2 };
    system.services.plgldr.memory_block = Some(block);
    system.services.plgldr.plugin_entry = game_entry;
    system.services.plgldr.exe_size = exe_size as u32;
    system.services.plgldr.heap_size = heap_size;
    log::info!("loaded 3GX plugin '{}' by {} ({} bytes executable, {} bytes heap)", plugin.title, plugin.author, exe_size, heap_size);
    let _ = FRAMEBUFFER_SIZE; // Reserved for the GSP framebuffer handoff.
    let _ = FCRAM_PADDR;
    Ok(())
}

fn patch_bootloader(
    boot: &mut [u8; 412],
    game_instr0: u32,
    game_instr1: u32,
    game_entry: u32,
    load_args: &[u32; 4],
    plugin_start: u32,
    plugin_end: u32,
    checksum: u32,
    launch_status: u32,
    plugin_entry: u32,
    no_flash: u32,
    load_func: &[u32],
) {
    let mut index = 0usize;
    while index < boot.len() / 4 {
        let at = index * 4;
        let word = u32::from_le_bytes(boot[at..at + 4].try_into().unwrap());
        let replacement = match word {
            0xDEAD_0000 => Some(game_instr0),
            0xDEAD_0001 => Some(game_instr1),
            0xDEAD_0002 => Some(game_entry),
            0xDEAD_0004 => Some(plugin_start),
            0xDEAD_0005 => Some(plugin_end),
            0xDEAD_0006 => Some(checksum),
            0xDEAD_0007 => Some(launch_status),
            0xDEAD_0008 => Some(plugin_entry),
            0xDEAD_0009 => Some(no_flash),
            _ => None,
        };
        if let Some(value) = replacement {
            boot[at..at + 4].copy_from_slice(&value.to_le_bytes());
        } else if word == 0xDEAD_0003 {
            for (offset, value) in load_args.iter().enumerate() {
                let pos = at + (offset + 1) * 4;
                boot[pos..pos + 4].copy_from_slice(&value.to_le_bytes());
            }
            index += load_args.len();
        } else if word == 0xDEAD_000A {
            for (offset, value) in load_func.iter().enumerate() {
                let pos = at + offset * 4;
                boot[pos..pos + 4].copy_from_slice(&value.to_le_bytes());
            }
            index += load_func.len().saturating_sub(1);
        }
        index += 1;
    }
}

fn put_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

/// Releases plugin mappings and its contiguous physical block when the title exits.
pub fn unload(system: &mut System) {
    let state = &mut system.services.plgldr;
    if !state.plugin_loaded {
        return;
    }
    if let Some(block) = state.memory_block.take() {
        system.memory.unmap(PLUGIN_HEADER_ADDRESS, state.exe_size);
        system.memory.unmap(BOOTLOADER_ADDRESS, BOOTLOADER_SIZE);
        system.memory.unmap(PLUGIN_HEAP_ADDRESS, state.heap_size);
        system.memory.phys.free(block);
    }
    state.plugin_loaded = false;
    state.plugin_process_id = u32::MAX;
    state.memory_region = 0;
    state.exe_size = 0;
    state.heap_size = 0;
    state.plugin_entry = 0;
    log::info!("unloaded 3GX plugin");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bootloader_patches_all_runtime_markers() {
        let mut boot = BOOTLOADER;
        patch_bootloader(&mut boot, 1, 2, 3, &[4, 5, 6, 7], 8, 9, 10, 11, 12, 1, &[13, 14]);
        let words: Vec<u32> = boot.chunks_exact(4).map(|chunk| u32::from_le_bytes(chunk.try_into().unwrap())).collect();
        assert!(words.contains(&1));
        assert!(words.contains(&2));
        assert!(words.contains(&3));
        assert!(words.contains(&4));
        assert!(words.contains(&5));
        assert!(words.contains(&6));
        assert!(words.contains(&7));
        assert!(words.contains(&8));
        assert!(words.contains(&9));
        assert!(words.contains(&10));
        assert!(words.contains(&11));
        assert!(words.contains(&12));
        assert!(words.contains(&13));
        assert!(words.contains(&14));
    }
}
