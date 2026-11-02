// Lo que hacía cv_bridge en C++: sensor_msgs/Image <-> cv::Mat (en Rust no hay cv_bridge)
use std::borrow::Cow;

use anyhow::{bail, ensure};
use opencv::core::{Mat, Vector};
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::prelude::*;
use r2r::sensor_msgs::msg::{CompressedImage, Image};
use r2r::std_msgs::msg::Header;

// Imagen de ROS -> Mat BGR de 3 canales (como cv_bridge::toCvShare(msg, "bgr8"))
pub fn image_to_bgr(msg: &Image) -> anyhow::Result<Mat> {
    // (canales, conversión a BGR; None = ya es BGR)
    let (channels, conversion) = match msg.encoding.as_str() {
        "bgr8" => (3, None),
        "rgb8" => (3, Some(imgproc::COLOR_RGB2BGR)),
        "mono8" => (1, Some(imgproc::COLOR_GRAY2BGR)),
        "bgra8" => (4, Some(imgproc::COLOR_BGRA2BGR)),
        "rgba8" => (4, Some(imgproc::COLOR_RGBA2BGR)),
        other => bail!("codificación '{other}' no soportada (bgr8, rgb8, mono8, bgra8, rgba8)"),
    };
    let (h, w, step) = (msg.height as usize, msg.width as usize, msg.step as usize);
    let row = w * channels;
    ensure!(h > 0 && w > 0, "imagen vacía");
    ensure!(step >= row && msg.data.len() >= step * h, "data no cuadra con width/height/step");

    // Filas seguidas, sin el relleno que puede haber al final de cada una (step > row)
    let packed: Cow<[u8]> = if step == row {
        Cow::Borrowed(&msg.data[..row * h])
    } else {
        Cow::Owned(msg.data.chunks(step).take(h).flat_map(|r| &r[..row]).copied().collect())
    };
    // Mat de 1 canal (h x row) que apunta a esos bytes, vista como `channels` canales (h x w)
    let flat = Mat::new_rows_cols_with_data(h as i32, row as i32, &packed)?;
    let view = flat.reshape(channels as i32, h as i32)?;
    let mut bgr = Mat::default();
    match conversion {
        Some(code) => imgproc::cvt_color_def(&view, &mut bgr, code)?,
        None => view.copy_to(&mut bgr)?, // copia: `packed` deja de existir al salir
    }
    Ok(bgr)
}

pub fn decode_compressed(msg: &CompressedImage) -> anyhow::Result<Mat> {
    let bgr = imgcodecs::imdecode(&Vector::<u8>::from_slice(&msg.data), imgcodecs::IMREAD_COLOR)?;
    ensure!(!bgr.empty(), "imagen comprimida no decodificable ({})", msg.format);
    Ok(bgr)
}

// Mat -> imagen de ROS (como cv_bridge::CvImage(header, encoding, mat).toImageMsg())
pub fn mat_to_image(mat: &Mat, encoding: &str, header: &Header) -> anyhow::Result<Image> {
    let owned;
    let mat = if mat.is_continuous() {
        mat
    } else {
        owned = mat.try_clone()?; // clone() deja los datos seguidos
        &owned
    };
    Ok(Image {
        header: header.clone(),
        height: mat.rows() as u32,
        width: mat.cols() as u32,
        encoding: encoding.to_string(),
        is_bigendian: 0,
        step: (mat.cols() as usize * mat.elem_size()?) as u32,
        data: mat.data_bytes()?.to_vec(),
    })
}
