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
  let chars: Vec<char> = image_base64.chars().collect();
  for y in 0..height{
    for x in 0..width{
      let i = ((y*pixel_modulo) * (width*pixel_modulo) + (x*pixel_modulo)) * 3;
      image_buffer.push(chars[i       as usize] as u8);
      image_buffer.push(chars[(i + 1) as usize] as u8);
      image_buffer.push(chars[(i + 2) as usize] as u8);
      image_buffer.push(255 as u8);
    }
  }
  
  image_buffer
}