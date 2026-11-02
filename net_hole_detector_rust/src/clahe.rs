// Ecualización adaptativa sobre el canal L de Lab (mejora el contraste bajo el agua
// sin alterar el color). Las imágenes de un canal se ecualizan directamente.
use opencv::core::{Mat, Ptr, Size, Vector};
use opencv::imgproc::{self, CLAHE};
use opencv::prelude::*;

#[derive(Debug, Clone, Copy)]
pub struct ClaheParams {
    pub clip_limit: f64, // contraste (2.0 suave, 4.0 agresivo)
    pub grid_size: i32,  // rejilla de tiles
}

impl Default for ClaheParams {
    fn default() -> Self {
        Self { clip_limit: 2.5, grid_size: 8 }
    }
}

pub struct Clahe {
    clahe: Ptr<CLAHE>,
}

impl Clahe {
    pub fn new(params: &ClaheParams) -> opencv::Result<Self> {
        let g = params.grid_size.max(1);
        Ok(Self { clahe: imgproc::create_clahe(params.clip_limit, Size::new(g, g))? })
    }

    // &mut self: apply de OpenCV modifica estado interno del objeto CLAHE
    pub fn apply(&mut self, bgr: &Mat) -> opencv::Result<Mat> {
        let mut out = Mat::default();
        if bgr.channels() == 1 {
            self.clahe.apply(bgr, &mut out)?;
            return Ok(out);
        }
        // Solo el canal L de Lab: más contraste sin tocar el color
        let mut lab = Mat::default();
        imgproc::cvt_color_def(bgr, &mut lab, imgproc::COLOR_BGR2Lab)?;
        let mut ch = Vector::<Mat>::new();
        opencv::core::split(&lab, &mut ch)?;
        let mut l_eq = Mat::default();
        self.clahe.apply(&ch.get(0)?, &mut l_eq)?;
        ch.set(0, l_eq)?;
        opencv::core::merge(&ch, &mut lab)?;
        imgproc::cvt_color_def(&lab, &mut out, imgproc::COLOR_Lab2BGR)?;
        Ok(out)
    }
}
