// Pre y postproceso de YOLOv8: letterbox, decodificar la salida, NMS, volver a píxeles y top-K
use std::collections::BTreeMap;

use opencv::core::{Mat, Rect, Rect2f, Scalar, Size, CV_8UC3};
use opencv::imgproc;
use opencv::prelude::*;

// Entrada del modelo: imagen escalada sin deformar y centrada en S x S (relleno 114)
#[derive(Debug, Clone)]
pub struct Letterbox {
    pub blob: Vec<f32>, // NCHW [1,3,S,S], RGB, valores en [0,1]
    pub input_size: i32, // S
    pub scale: f32,
    pub pad_x: f32,
    pub pad_y: f32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Detection2D {
    pub class_id: i32,
    pub score: f32,
    pub box_px: Rect2f,
}

// Intersección de dos rectángulos (como `a & b` de OpenCV): vacía si no se tocan
pub fn intersect(a: &Rect2f, b: &Rect2f) -> Rect2f {
    let x1 = a.x.max(b.x);
    let y1 = a.y.max(b.y);
    let w = (a.x + a.width).min(b.x + b.width) - x1;
    let h = (a.y + a.height).min(b.y + b.height) - y1;
    if w <= 0.0 || h <= 0.0 {
        return Rect2f::default();
    }
    Rect2f::new(x1, y1, w, h)
}

// Intersección / unión
pub fn iou(a: &Rect2f, b: &Rect2f) -> f32 {
    let inter = intersect(a, b).area();
    let union = a.area() + b.area() - inter;
    if union <= 0.0 {
        0.0
    } else {
        inter / union
    }
}

pub fn letterbox(bgr: &Mat, input_size: i32) -> opencv::Result<Letterbox> {
    let s = input_size as f32;
    let scale = (s / bgr.cols() as f32).min(s / bgr.rows() as f32);
    let new_w = (bgr.cols() as f32 * scale).round() as i32;
    let new_h = (bgr.rows() as f32 * scale).round() as i32;

    // Igual que el LetterBox de Ultralytics: centrado, el píxel sobrante va abajo/derecha
    let dw = (input_size - new_w) as f32 / 2.0;
    let dh = (input_size - new_h) as f32 / 2.0;
    let left = (dw - 0.1).round() as i32;
    let top = (dh - 0.1).round() as i32;

    let mut padded = Mat::new_rows_cols_with_default(input_size, input_size, CV_8UC3, Scalar::all(114.0))?;
    {
        // roi presta `padded` de forma mutable hasta el final de este bloque
        let mut roi = padded.roi_mut(Rect::new(left, top, new_w, new_h))?;
        if new_w != bgr.cols() || new_h != bgr.rows() {
            let mut resized = Mat::default();
            imgproc::resize(bgr, &mut resized, Size::new(new_w, new_h), 0.0, 0.0, imgproc::INTER_LINEAR)?;
            resized.copy_to(&mut roi)?;
        } else {
            bgr.copy_to(&mut roi)?;
        }
    }

    // HWC BGR u8 -> CHW RGB f32 en [0, 1] (lo que hacía cv::dnn::blobFromImage)
    let pixels = padded.data_bytes()?;
    let plane = (input_size * input_size) as usize;
    let mut blob = vec![0.0f32; 3 * plane];
    for i in 0..plane {
        let (b, g, r) = (pixels[3 * i], pixels[3 * i + 1], pixels[3 * i + 2]);
        blob[i] = r as f32 / 255.0;
        blob[plane + i] = g as f32 / 255.0;
        blob[2 * plane + i] = b as f32 / 255.0;
    }
    Ok(Letterbox { blob, input_size, scale, pad_x: left as f32, pad_y: top as f32 })
}

// Salida YOLOv8 [1, rows = 4 + nc, anchors] en orden fila-mayor (cx, cy, w, h, scores...).
// Las cajas quedan en el espacio del letterbox.
pub fn decode(data: &[f32], rows: usize, anchors: usize, conf_threshold: f32) -> Vec<Detection2D> {
    let mut dets = Vec::new();
    let nc = rows - 4;
    for a in 0..anchors {
        let mut best = 0;
        let mut best_score = data[4 * anchors + a];
        for c in 1..nc {
            let sc = data[(4 + c) * anchors + a];
            if sc > best_score {
                best_score = sc;
                best = c as i32;
            }
        }
        if best_score < conf_threshold {
            continue;
        }
        let (cx, cy) = (data[a], data[anchors + a]);
        let (w, h) = (data[2 * anchors + a], data[3 * anchors + a]);
        dets.push(Detection2D {
            class_id: best,
            score: best_score,
            box_px: Rect2f::new(cx - w / 2.0, cy - h / 2.0, w, h),
        });
    }
    dets
}

// NMS por clase; devuelve las supervivientes ordenadas por score.
// Mismo algoritmo que cv::dnn::NMSBoxes: de mayor a menor score, se queda una caja si no
// solapa más de iou_threshold con ninguna de las ya elegidas.
pub fn nms(dets: &[Detection2D], iou_threshold: f32) -> Vec<Detection2D> {
    let mut by_class: BTreeMap<i32, Vec<Detection2D>> = BTreeMap::new();
    for d in dets {
        by_class.entry(d.class_id).or_default().push(*d);
    }
    let mut kept = Vec::new();
    for (_cls, candidates) in by_class {
        let mut chosen: Vec<Detection2D> = Vec::new();
        for d in top_k(candidates, usize::MAX) {
            if chosen.iter().all(|k| iou(&k.box_px, &d.box_px) <= iou_threshold) {
                chosen.push(d);
            }
        }
        kept.extend(chosen);
    }
    top_k(kept, usize::MAX)
}

// Del espacio del letterbox a píxeles de la imagen original, recortadas a la imagen
pub fn unletterbox(dets: Vec<Detection2D>, lb: &Letterbox, image: Size) -> Vec<Detection2D> {
    let bounds = Rect2f::new(0.0, 0.0, image.width as f32, image.height as f32);
    dets.into_iter()
        .filter_map(|mut d| {
            let r = Rect2f::new(
                (d.box_px.x - lb.pad_x) / lb.scale,
                (d.box_px.y - lb.pad_y) / lb.scale,
                d.box_px.width / lb.scale,
                d.box_px.height / lb.scale,
            );
            d.box_px = intersect(&r, &bounds);
            // las que caen enteras en el relleno no existen en la imagen
            (d.box_px.area() > 0.0).then_some(d)
        })
        .collect()
}

// Las k de mayor score, ordenadas de mayor a menor (sort_by es estable, como std::stable_sort)
pub fn top_k(mut dets: Vec<Detection2D>, k: usize) -> Vec<Detection2D> {
    dets.sort_by(|a, b| b.score.total_cmp(&a.score));
    dets.truncate(k);
    dets
}
