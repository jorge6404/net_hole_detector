// Calibración de la cámara y modelo pinhole 2D <-> 3D (frame óptico de la cámara).
// Plan B: YAML de camera_calibration. Plan A: CameraInfo en vivo (set_matrices).
// Imagen rectificada -> P, imagen raw -> K. Sin P válida se usa K.
use opencv::core::{Point3d, Size, Size2d};
use serde_yaml::Value;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Intrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

#[derive(Debug, Default)]
pub struct CameraGeometry {
    k: [f64; 9],
    p: [f64; 12],
    has_p: bool,
    rectified: bool,
    calibrated: bool,
    size: Size,
    active: Intrinsics,
}

// Option: Some(valor) si sale bien, None si falta algo (en vez de devolver bool + parámetro de salida)
fn read_matrix<const N: usize>(node: &Value) -> Option<[f64; N]> {
    // Cada `?` devuelve None en ese momento si lo de su izquierda es None
    let data = node.get("data")?.as_sequence()?;
    if data.len() != N {
        return None;
    }
    let mut out = [0.0; N]; // mut permite modificar la variable; sin mut es solo lectura
    for (slot, v) in out.iter_mut().zip(data) {
        *slot = v.as_f64()?;
    }
    Some(out)
}

impl CameraGeometry {
    // false (y sin cambios) si el fichero no existe o no tiene el formato esperado
    pub fn load_yaml(&mut self, path: &str) -> bool {
        let Ok(text) = std::fs::read_to_string(path) else {
            return false;
        };
        let Ok(root) = serde_yaml::from_str::<Value>(&text) else {
            return false;
        };
        let Some(k) = read_matrix::<9>(&root["camera_matrix"]) else {
            return false;
        };
        // opcional: sin P se usa K
        let p = read_matrix::<12>(&root["projection_matrix"]).unwrap_or([0.0; 12]);
        let dim = |key: &str| root[key].as_i64().unwrap_or(0) as i32;
        self.set_matrices(k, p, Size::new(dim("image_width"), dim("image_height")));
        true
    }

    // P todo ceros = no disponible
    pub fn set_matrices(&mut self, k: [f64; 9], p: [f64; 12], size: Size) {
        self.k = k;
        self.p = p;
        self.has_p = p.iter().any(|&v| v != 0.0);
        self.size = size;
        self.calibrated = true;
        self.apply_selection();
    }

    // Se recuerda: las cargas posteriores vuelven a aplicarlo
    pub fn select(&mut self, is_rectified: bool) {
        self.rectified = is_rectified;
        self.apply_selection();
    }

    pub fn calibrated(&self) -> bool {
        self.calibrated
    }

    pub fn intrinsics(&self) -> &Intrinsics {
        &self.active
    }

    pub fn calibration_size(&self) -> Size {
        self.size
    }

    // Z = fx * escala
    pub fn z_from_scale(&self, m_per_px: f64) -> f64 {
        self.active.fx * m_per_px
    }

    // pinhole inverso
    pub fn project(&self, u: f64, v: f64, z: f64) -> Point3d {
        let a = &self.active;
        Point3d::new((u - a.cx) * z / a.fx, (v - a.cy) * z / a.fy, z)
    }

    // tamaño real
    pub fn size_m(&self, w_px: f64, h_px: f64, z: f64) -> Size2d {
        Size2d::new(w_px * z / self.active.fx, h_px * z / self.active.fy)
    }

    fn apply_selection(&mut self) {
        if !self.calibrated {
            return;
        }
        self.active = if self.rectified && self.has_p {
            // P es 3x4: nos quedamos con la parte 3x3 izquierda
            Intrinsics { fx: self.p[0], fy: self.p[5], cx: self.p[2], cy: self.p[6] }
        } else {
            Intrinsics { fx: self.k[0], fy: self.k[4], cx: self.k[2], cy: self.k[5] }
        };
    }
}
