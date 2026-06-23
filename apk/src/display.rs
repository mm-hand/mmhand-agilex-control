use std::ffi::c_void;

pub const SIZE: usize = 256;
const CONNECTED: &[u8] = include_bytes!("../assets/connected.rgba");
const DISCONNECTED: &[u8] = include_bytes!("../assets/disconnected.rgba");

const GL_TEXTURE_2D: u32 = 0x0DE1;
const GL_RGBA: u32 = 0x1908;
const GL_UNSIGNED_BYTE: u32 = 0x1401;

#[link(name = "GLESv3")]
unsafe extern "C" {
    fn glBindTexture(target: u32, texture: u32);
    fn glTexSubImage2D(
        target: u32,
        level: i32,
        x: i32,
        y: i32,
        width: i32,
        height: i32,
        format: u32,
        kind: u32,
        pixels: *const c_void,
    );
}

pub struct Images {
    connected: Vec<u8>,
    disconnected: Vec<u8>,
}

impl Images {
    pub fn new() -> Self {
        Self {
            connected: expand(CONNECTED, [3, 42, 18, 255]),
            disconnected: expand(DISCONNECTED, [45, 3, 8, 255]),
        }
    }

    pub fn upload(&self, texture: u32, connected: bool) {
        let pixels = if connected {
            &self.connected
        } else {
            &self.disconnected
        };
        unsafe {
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexSubImage2D(
                GL_TEXTURE_2D,
                0,
                0,
                0,
                SIZE as i32,
                SIZE as i32,
                GL_RGBA,
                GL_UNSIGNED_BYTE,
                pixels.as_ptr().cast(),
            );
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }
}

fn expand(mask: &[u8], background: [u8; 4]) -> Vec<u8> {
    let mut pixels = Vec::with_capacity(SIZE * SIZE * 4);
    for byte in mask
        .iter()
        .filter(|byte| **byte != b'\r' && **byte != b'\n')
    {
        pixels.extend_from_slice(if *byte == b'#' {
            &[235, 240, 235, 255]
        } else {
            &background
        });
    }
    assert_eq!(pixels.len(), SIZE * SIZE * 4);
    pixels
}
