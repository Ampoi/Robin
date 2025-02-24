use wasm_bindgen::prelude::*;

#[wasm_bindgen]
extern "C" {
  #[wasm_bindgen(js_namespace = console)]
  fn log(s: &str);
}

#[wasm_bindgen]
pub fn convert(image_base64: &str, width: u32, height: u32, pixel_modulo: u32) -> Vec<u8> {
  let total_pixels = width * height;
  let mut image_buffer: Vec<u8> = Vec::with_capacity((total_pixels * 4) as usize);
  let image_bytes: Vec<u8> = image_base64.chars().map(|c| c as u8).collect();

  let width_modulo = width * pixel_modulo;

  for y in 0..height {
    let y_offset = (y * pixel_modulo) * width_modulo;
    for x in 0..width {
      let i = ((y_offset + (x * pixel_modulo)) * 3) as usize;
      image_buffer.extend_from_slice(&[
        image_bytes[i],
        image_bytes[i + 1],
        image_bytes[i + 2],
        255
      ]);
    }
  }

  image_buffer
}