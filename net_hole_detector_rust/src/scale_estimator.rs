// Escala de la imagen (metros/píxel) a partir de las celdas de la red: la mediana de
// la raíz del área de los blobs válidos frente al lado real de la celda.
use opencv::core::{self, Mat, Point, Point2f, Rect2f, Scalar, Size, Vector, BORDER_DEFAULT, CV_8U, CV_8UC1};
use opencv::imgproc;
use opencv::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScaleEstimatorParams {
    pub blur_kernel: i32,       // impar
    pub morph_kernel: i32,      // impar
    pub thresh_block_size: i32, // impar
    pub thresh_c: f64,
    pub min_area_px: f64,
    pub max_percent_area: f64, // fracción del área de la imagen
    pub min_aspect: f64,
    pub max_aspect: f64,
}

impl Default for ScaleEstimatorParams {
    fn default() -> Self {
        Self {
            blur_kernel: 7,
            morph_kernel: 5,
            thresh_block_size: 41,
            thresh_c: 5.0,
            min_area_px: 25.0,
            max_percent_area: 0.1,
            min_aspect: 0.5,
            max_aspect: 2.0,
        }
    }
}

// Qué imágenes generar (cuestan CPU; solo si alguien las mira)
#[derive(Debug, Clone, Copy, Default)]
pub struct EstimateOutputs {
    pub mask_all: bool,
    pub mask_yolo: bool,
    pub overlay: bool,
}

#[derive(Debug, Default)]
pub struct NetEstimate {
    pub scale_m_per_px: Option<f64>, // None si no se ve la red
    pub median_area_px: f64,
    pub n_blobs: usize,
    pub mask_all: Option<Mat>,  // blobs válidos a 255 sobre fondo 0
    pub mask_yolo: Option<Mat>, // solo los blobs con el centroide dentro de alguna caja
    pub overlay: Option<Mat>,   // imagen con los blobs dibujados
}

// Impar y al menos `min` (OpenCV no acepta kernels pares ni bloques de umbral < 3)
fn make_odd(k: i32, min: i32) -> i32 {
    min.max(if k % 2 == 0 { k + 1 } else { k })
}

// Recibe el Vec por valor (se lo queda) porque select_nth_unstable lo reordena
fn median(mut v: Vec<f64>) -> f64 {
    let n = v.len();
    let (lower, &mut hi, _) = v.select_nth_unstable_by(n / 2, f64::total_cmp);
    if n % 2 == 1 {
        return hi;
    }
    let lo = lower.iter().copied().fold(f64::MIN, f64::max);
    0.5 * (lo + hi)
}

pub struct ScaleEstimator {
    params: ScaleEstimatorParams,
}

impl ScaleEstimator {
    pub fn new(params: ScaleEstimatorParams) -> Self {
        let mut p = params;
        p.blur_kernel = make_odd(p.blur_kernel, 1);
        p.morph_kernel = make_odd(p.morph_kernel, 1);
        p.thresh_block_size = make_odd(p.thresh_block_size, 3);
        Self { params: p }
    }

    // Parámetros ya saneados (kernels impares, bloque del umbral >= 3)
    pub fn params(&self) -> &ScaleEstimatorParams {
        &self.params
    }

    pub fn estimate(
        &self, image: &Mat, mesh_cell_m: f64, boxes: &[Rect2f], out: EstimateOutputs,
    ) -> opencv::Result<NetEstimate> {
        let p = &self.params;
        let mut result = NetEstimate::default();

        // 1. Con fondo azul, el canal azul es el de más contraste entre hilo (oscuro) y agua
        let blue = if image.channels() == 1 {
            image.try_clone()?
        } else {
            let mut b = Mat::default();
            core::extract_channel(image, &mut b, 0)?;
            b
        };

        // 2. Umbral adaptativo + limpieza de ruido
        let (mut blurred, mut thresh, mut opening) = (Mat::default(), Mat::default(), Mat::default());
        imgproc::gaussian_blur_def(&blue, &mut blurred, Size::new(p.blur_kernel, p.blur_kernel), 0.0)?;
        imgproc::adaptive_threshold(
            &blurred, &mut thresh, 255.0, imgproc::ADAPTIVE_THRESH_GAUSSIAN_C, imgproc::THRESH_BINARY,
            p.thresh_block_size, p.thresh_c,
        )?;
        let kernel = Mat::ones(p.morph_kernel, p.morph_kernel, CV_8U)?.to_mat()?;
        imgproc::morphology_ex(
            &thresh, &mut opening, imgproc::MORPH_OPEN, &kernel, Point::new(-1, -1), 1,
            BORDER_DEFAULT, imgproc::morphology_default_border_value()?,
        )?;

        // 3. Contornos filtrados por área y forma
        let mut contours = Vector::<Vector<Point>>::new();
        imgproc::find_contours_def(&opening, &mut contours, imgproc::RETR_LIST, imgproc::CHAIN_APPROX_SIMPLE)?;

        let max_area = blue.cols() as f64 * blue.rows() as f64 * p.max_percent_area;
        let mut sqrt_areas = Vec::new();
        let mut valid = Vector::<Vector<Point>>::new();
        let mut in_boxes = Vector::<Vector<Point>>::new();
        for cnt in &contours {
            let area = imgproc::contour_area_def(&cnt)?;
            if area < p.min_area_px || area > max_area {
                continue;
            }
            let r = imgproc::bounding_rect(&cnt)?;
            let aspect = r.width as f64 / r.height as f64;
            if aspect < p.min_aspect || aspect > p.max_aspect {
                continue;
            }
            // Lado equivalente L = sqrt(área)
            sqrt_areas.push(area.sqrt());

            if !boxes.is_empty() {
                let m = imgproc::moments_def(&cnt)?;
                if m.m00 != 0.0 {
                    let c = Point2f::new((m.m10 / m.m00) as f32, (m.m01 / m.m00) as f32);
                    let inside = |b: &Rect2f| c.x > b.x && c.x < b.x + b.width && c.y > b.y && c.y < b.y + b.height;
                    if boxes.iter().any(inside) {
                        in_boxes.push(cnt.clone());
                    }
                }
            }
            valid.push(cnt);
        }

        if sqrt_areas.is_empty() {
            return Ok(result);
        }

        // 4. Mediana: los agujeros rotos que pasen el filtro no mueven la escala
        let median_l_px = median(sqrt_areas);
        result.scale_m_per_px = Some(mesh_cell_m / median_l_px);
        result.median_area_px = median_l_px * median_l_px;
        result.n_blobs = valid.len();

        // 5. Imágenes de depuración
        let filled = |contours: &Vector<Vector<Point>>| -> opencv::Result<Mat> {
            let mut mask = Mat::zeros_size(blue.size()?, CV_8UC1)?.to_mat()?;
            // grosor FILLED: contornos rellenos
            imgproc::draw_contours(
                &mut mask, contours, -1, Scalar::all(255.0), imgproc::FILLED, imgproc::LINE_8,
                &core::no_array(), i32::MAX, Point::default(),
            )?;
            Ok(mask)
        };
        if out.mask_all {
            result.mask_all = Some(filled(&valid)?);
        }
        if out.mask_yolo {
            result.mask_yolo = Some(filled(&in_boxes)?);
        }
        if out.overlay {
            let mut overlay = image.try_clone()?;
            imgproc::draw_contours(
                &mut overlay, &valid, -1, Scalar::new(0.0, 255.0, 0.0, 0.0), 3, imgproc::LINE_8,
                &core::no_array(), i32::MAX, Point::default(),
            )?;
            result.overlay = Some(overlay);
        }
        Ok(result)
    }
}
