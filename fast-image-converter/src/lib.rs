use wasm_bindgen::prelude::*;

#[wasm_bindgen]
extern "C" {
  #[wasm_bindgen(js_namespace = console)]
  fn log(s: &str);
}

#[wasm_bindgen]
pub fn convert(image_base64: &str, width: u16, height: u16) -> Vec<u8> {
  let total_pixels = (width as u32) * (height as u32);
  let mut image_buffer: Vec<u8> = Vec::with_capacity((total_pixels * 4) as usize);
  let chars: Vec<char> = image_base64.chars().collect();
  for i in (0..total_pixels * 3).step_by(3) {
    image_buffer.push(chars[i       as usize] as u8);
    image_buffer.push(chars[(i + 1) as usize] as u8);
    image_buffer.push(chars[(i + 2) as usize] as u8);
    image_buffer.push(255 as u8);
  }
  
  image_buffer
}