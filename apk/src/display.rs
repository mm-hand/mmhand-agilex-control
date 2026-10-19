use std::ffi::c_void;

pub const SIZE: usize = 256;
const CONNECTED: &[u8] = include_bytes!("../assets/connected.rgba");
const DISCONNECTED: &[u8] = include_bytes!("../assets/disconnected.rgba");

#[link(name = "GLESv3")]
unsafe extern "C" {
    fn glBindTexture(target: u32, texture: u32);
    fn glTexSubImage2D(target: u32, level: i32, x: i32, y: i32, w: i32, h: i32, fmt: u32, kind: u32, pixels: *const c_void);
}

pub struct Images { connected: Vec<u8>, disconnected: Vec<u8> }

impl Images {
    pub fn new() -> Self {
        Self { connected: expand(CONNECTED, [3, 42, 18, 255]), disconnected: expand(DISCONNECTED, [45, 3, 8, 255]) }
    }
    pub fn upload(&self, texture: u32, connected: bool) {
        let pixels = if connected { &self.connected } else { &self.disconnected };
        unsafe {
            glBindTexture(0x0DE1, texture);
            glTexSubImage2D(0x0DE1, 0, 0, 0, SIZE as i32, SIZE as i32, 0x1908, 0x1401, pixels.as_ptr().cast());
            glBindTexture(0x0DE1, 0);
        }
    }
}

fn expand(mask: &[u8], bg: [u8; 4]) -> Vec<u8> {
    mask.iter().filter(|b| **b != b'\r' && **b != b'\n').flat_map(|b| {
        if *b == b'#' { [235, 240, 235, 255] } else { bg }
    }).collect()
}
