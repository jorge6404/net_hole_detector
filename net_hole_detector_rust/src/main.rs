// Detector de agujeros en la red: imagen -> CLAHE -> blobs (escala, Z) + YOLO -> detecciones 3D
// en el frame de la cámara. Toda la lógica está en la librería (src/lib.rs);
// este nodo solo lee parámetros, convierte mensajes y publica.
mod cv_bridge;

use std::collections::HashMap;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail};
use futures::stream::BoxStream;
use futures::{FutureExt, StreamExt};
use opencv::core::{Mat, Point, Rect, Rect2f, Scalar, Size};
use opencv::imgproc;
use opencv::prelude::*;
use r2r::net_hole_detector_msgs_rust::msg::{BoundingBox, BoundingBoxArray, Detection3D, Detection3DArray, NetStats};
use r2r::sensor_msgs::msg::{CameraInfo, CompressedImage, Image};
use r2r::std_msgs::msg::Header;
use r2r::{Clock, Node, Parameter, ParameterValue, Publisher, QosProfile};

use net_hole_detector_rust::camera_geometry::CameraGeometry;
use net_hole_detector_rust::clahe::{Clahe, ClaheParams};
use net_hole_detector_rust::inference_backend::InferenceBackend;
use net_hole_detector_rust::onnxruntime_backend::{expand_user, OrtBackend, OrtOptions};
use net_hole_detector_rust::scale_estimator::{EstimateOutputs, ScaleEstimator, ScaleEstimatorParams};
use net_hole_detector_rust::yolo_postprocess::{decode, letterbox, nms, top_k, unletterbox, Detection2D};

const PACKAGE: &str = "net_hole_detector_rust";

// ---- Parámetros ----
// r2r no tiene declare_parameter: los parámetros del YAML ya están en node.params. Si falta
// uno se añade con su valor por defecto (así `ros2 param list/get` lo ve), como hace rclcpp.
fn declare(node: &Node, name: &str, default: ParameterValue) -> ParameterValue {
    let mut params = node.params.lock().unwrap();
    params.entry(name.to_string()).or_insert_with(|| Parameter::new(default)).value.clone()
}

fn declare_string(node: &Node, name: &str, default: &str) -> anyhow::Result<String> {
    match declare(node, name, ParameterValue::String(default.into())) {
        ParameterValue::String(s) => Ok(s),
        other => bail!("el parámetro {name} debe ser un texto, no {other:?}"),
    }
}

fn declare_bool(node: &Node, name: &str, default: bool) -> anyhow::Result<bool> {
    match declare(node, name, ParameterValue::Bool(default)) {
        ParameterValue::Bool(b) => Ok(b),
        other => bail!("el parámetro {name} debe ser true/false, no {other:?}"),
    }
}

fn declare_int(node: &Node, name: &str, default: i64) -> anyhow::Result<i64> {
    match declare(node, name, ParameterValue::Integer(default)) {
        ParameterValue::Integer(i) => Ok(i),
        other => bail!("el parámetro {name} debe ser un entero, no {other:?}"),
    }
}

// Parámetro numérico que acepta 1 o 1.0 en el YAML
fn declare_number(node: &Node, name: &str, default: f64) -> anyhow::Result<f64> {
    match declare(node, name, ParameterValue::Double(default)) {
        ParameterValue::Double(d) => Ok(d),
        ParameterValue::Integer(i) => Ok(i as f64),
        other => bail!("el parámetro {name} debe ser un número, no {other:?}"),
    }
}

// share/<paquete> en AMENT_PREFIX_PATH (lo que hace ament_index_cpp); con `cargo run` sin
// instalar, la carpeta del crate
fn package_share_directory() -> String {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    prefixes
        .split(':')
        .map(|prefix| format!("{prefix}/share/{PACKAGE}"))
        .find(|share| Path::new(share).join("package.xml").exists())
        .unwrap_or_else(|| env!("CARGO_MANIFEST_DIR").to_string())
}

// ---- Logs con límite de frecuencia (RCLCPP_*_THROTTLE) ----
#[derive(Default)]
struct Throttle {
    last: HashMap<&'static str, Instant>,
}

impl Throttle {
    // true como mucho una vez cada 2 s por clave
    fn ready(&mut self, key: &'static str) -> bool {
        let now = Instant::now();
        match self.last.get(key) {
            Some(t) if now.duration_since(*t) < Duration::from_secs(2) => false,
            _ => {
                self.last.insert(key, now);
                true
            }
        }
    }
}

// ---- Nodo ----
struct Config {
    camera_frame: String,
    conf_threshold: f64,
    iou_threshold: f64,
    max_detections: usize,
    period_s: f64,
    max_z_age_s: f64,
    mesh_cell_m: f64,
    clahe_enabled: bool,
}

struct Publishers {
    boxes: Publisher<BoundingBoxArray>,
    dets: Publisher<Detection3DArray>,
    stats: Publisher<NetStats>,
    processed: Publisher<Image>,
    blob_bin: Publisher<Image>,
    blob_overlay: Publisher<Image>,
    debug: Publisher<Image>,
    fused: Publisher<Image>,
}

struct NetHoleDetector {
    logger: String,
    clock: Arc<Mutex<Clock>>,
    cfg: Config,
    pubs: Publishers,
    throttle: Throttle,

    // Procesado
    clahe: Clahe,
    estimator: ScaleEstimator,
    backend: Box<dyn InferenceBackend>,
    geo: CameraGeometry,

    // Estado (tiempos en segundos del reloj ROS)
    got_camera_info: bool,
    warned_size: bool,
    latest_z: Option<f64>,
    latest_z_time: f64,
    last_infer: Option<f64>,
}

// Las imágenes de entrada llegan por uno de los dos topics según image_transport
enum InputImage {
    Raw(Image),
    Compressed(CompressedImage),
}

impl NetHoleDetector {
    // Devuelve el nodo listo y el stream de imágenes por el que escucha
    fn new(node: &mut Node) -> anyhow::Result<(Self, BoxStream<'static, InputImage>)> {
        let logger = node.logger().to_string();
        let share = package_share_directory();
        let resolve = |path: String, fallback: &str| -> String {
            let p = expand_user(if path.is_empty() { fallback } else { &path });
            if Path::new(&p).is_absolute() {
                p
            } else {
                format!("{share}/{p}")
            }
        };

        // 1. Parámetros (config/net_hole_detector.yaml)
        let transport = declare_string(node, "image_transport", "compressed")?;
        let is_rectified = declare_bool(node, "is_rectified", false)?;
        let calibration_file = resolve(declare_string(node, "calibration_file", "")?, "config/right_laser.yaml");
        let model_path = resolve(declare_string(node, "model.path", "")?, "models/yolov8_hole_detector_5_best.onnx");
        let input_size = declare_int(node, "model.input_size", 640)? as i32;
        let ort = OrtOptions {
            engine: declare_string(node, "inference.engine", "tensorrt")?,
            trt_fp16: declare_bool(node, "inference.trt_fp16", true)?,
            trt_cache_dir: declare_string(node, "inference.trt_cache_dir", "~/.ros/net_hole_detector/trt_cache")?,
            device_id: declare_int(node, "inference.device_id", 0)? as i32,
            cpu_threads: declare_int(node, "inference.cpu_threads", 0)? as i32,
        };
        let cfg = Config {
            camera_frame: declare_string(node, "camera_frame", "")?,
            conf_threshold: declare_number(node, "yolo.confidence_threshold", 0.4)?,
            iou_threshold: declare_number(node, "yolo.iou_threshold", 0.45)?,
            max_detections: declare_int(node, "yolo.max_detections", 5)?.max(0) as usize,
            period_s: declare_number(node, "yolo.period_s", 0.5)?,
            max_z_age_s: declare_number(node, "max_z_age_s", 0.5)?,
            mesh_cell_m: declare_number(node, "net.mesh_cell_size_m", 0.015)?,
            clahe_enabled: declare_bool(node, "clahe.enabled", true)?,
        };
        let d = ClaheParams::default();
        let cp = ClaheParams {
            clip_limit: declare_number(node, "clahe.clip_limit", d.clip_limit)?,
            grid_size: declare_int(node, "clahe.grid_size", d.grid_size as i64)? as i32,
        };
        let d = ScaleEstimatorParams::default();
        let sp = ScaleEstimatorParams {
            blur_kernel: declare_int(node, "blobs.blur_kernel", d.blur_kernel as i64)? as i32,
            morph_kernel: declare_int(node, "blobs.morph_kernel", d.morph_kernel as i64)? as i32,
            thresh_block_size: declare_int(node, "blobs.thresh_block_size", d.thresh_block_size as i64)? as i32,
            thresh_c: declare_number(node, "blobs.thresh_c", d.thresh_c)?,
            min_area_px: declare_number(node, "blobs.min_area_px", d.min_area_px)?,
            max_percent_area: declare_number(node, "blobs.max_percent_area", d.max_percent_area)?,
            min_aspect: declare_number(node, "blobs.min_aspect", d.min_aspect)?,
            max_aspect: declare_number(node, "blobs.max_aspect", d.max_aspect)?,
        };

        // 2. Procesado
        let clahe = Clahe::new(&cp)?;
        let estimator = ScaleEstimator::new(sp);

        // 3. Calibración de respaldo (camera_info la sobrescribe)
        let mut geo = CameraGeometry::default();
        geo.select(is_rectified);
        if geo.load_yaml(&calibration_file) {
            r2r::log_info!(&logger, "Calibración de respaldo: {} (fx={:.1})", calibration_file, geo.intrinsics().fx);
        } else {
            r2r::log_warn!(&logger, "No se pudo leer la calibración {}; esperando camera_info", calibration_file);
        }

        // 4. Modelo (falla al arrancar si no hay ningún motor posible)
        let backend = OrtBackend::new(
            &model_path,
            &ort,
            |m| r2r::log_info!(&logger, "{}", m),
            |m| r2r::log_warn!(&logger, "{}", m),
        )
        .inspect_err(|e| r2r::log_fatal!(&logger, "No se pudo cargar el modelo: {:#}", e))?;
        if backend.input_size() != input_size {
            r2r::log_warn!(
                &logger,
                "model.input_size={} pero el ONNX espera {}; se usa {}",
                input_size,
                backend.input_size(),
                backend.input_size()
            );
        }
        r2r::log_info!(
            &logger,
            "Modelo {} (entrada {}, motor {})",
            model_path,
            backend.input_size(),
            backend.provider()
        );

        // 5. Publicadores (relativos al namespace) y suscripciones
        let qos = QosProfile::default();
        let pubs = Publishers {
            boxes: node.create_publisher("bounding_boxes", qos.clone())?,
            dets: node.create_publisher("detections_3d", qos.clone())?,
            stats: node.create_publisher("net_stats", qos.clone())?,
            processed: node.create_publisher("image_processed", qos.clone())?,
            blob_bin: node.create_publisher("blobs/binary", qos.clone())?,
            blob_overlay: node.create_publisher("blobs/overlay", qos.clone())?,
            debug: node.create_publisher("debug_image", qos.clone())?,
            fused: node.create_publisher("net_mask_yolo_fused", qos.clone())?,
        };

        // Profundidad 1: si la inferencia es lenta se descartan frames. Con compressed se escucha
        // image/compressed; el launch remapea `image` e `image/compressed` (r2r no deja resolver
        // el nombre remapeado de `image` para añadirle /compressed, como hacía el C++).
        let sensor_qos = QosProfile::sensor_data().keep_last(1);
        let images: BoxStream<'static, InputImage> = match transport.as_str() {
            "raw" => node.subscribe::<Image>("image", sensor_qos)?.map(InputImage::Raw).boxed(),
            "compressed" => {
                node.subscribe::<CompressedImage>("image/compressed", sensor_qos)?.map(InputImage::Compressed).boxed()
            }
            other => bail!("image_transport debe ser raw o compressed, no '{other}'"),
        };
        let topic = if transport == "raw" { "image" } else { "image/compressed" };
        r2r::log_info!(&logger, "Escuchando {} (relativo a {})", topic, node.namespace()?);

        let detector = Self {
            logger,
            clock: node.get_ros_clock(),
            cfg,
            pubs,
            throttle: Throttle::default(),
            clahe,
            estimator,
            backend: Box::new(backend),
            geo,
            got_camera_info: false,
            warned_size: false,
            latest_z: None,
            latest_z_time: 0.0,
            last_infer: None,
        };
        Ok((detector, images))
    }

    fn now(&self) -> f64 {
        self.clock.lock().unwrap().get_now().map(|d| d.as_secs_f64()).unwrap_or(0.0)
    }

    fn on_camera_info(&mut self, msg: &CameraInfo) {
        if msg.k.first().copied().unwrap_or(0.0) == 0.0 {
            return; // CameraInfo sin calibrar
        }
        let Ok(k) = <[f64; 9]>::try_from(msg.k.as_slice()) else {
            return;
        };
        let p = <[f64; 12]>::try_from(msg.p.as_slice()).unwrap_or([0.0; 12]);
        self.geo.set_matrices(k, p, Size::new(msg.width as i32, msg.height as i32));
        if !self.got_camera_info {
            r2r::log_info!(&self.logger, "Calibración en vivo recibida (fx={:.1})", self.geo.intrinsics().fx);
            self.got_camera_info = true;
        }
    }

    fn on_input(&mut self, input: InputImage) {
        let decoded = match &input {
            InputImage::Raw(msg) => cv_bridge::image_to_bgr(msg).map(|m| (m, msg.header.clone())),
            InputImage::Compressed(msg) => cv_bridge::decode_compressed(msg).map(|m| (m, msg.header.clone())),
        };
        match decoded {
            Ok((bgr, header)) => {
                if let Err(e) = self.on_image(&bgr, header) {
                    if self.throttle.ready("procesado") {
                        r2r::log_error!(&self.logger, "Error procesando la imagen: {:#}", e);
                    }
                }
            }
            Err(e) => {
                if self.throttle.ready("conversion") {
                    r2r::log_error!(&self.logger, "Imagen no convertible: {:#}", e);
                }
            }
        }
    }

    fn on_image(&mut self, bgr: &Mat, mut header: Header) -> anyhow::Result<()> {
        if !self.cfg.camera_frame.is_empty() {
            header.frame_id = self.cfg.camera_frame.clone();
        }
        let img = if self.cfg.clahe_enabled { self.clahe.apply(bgr)? } else { bgr.try_clone()? };

        let size = img.size()?;
        if !self.got_camera_info && self.geo.calibrated() && self.geo.calibration_size() != size && !self.warned_size {
            let cal = self.geo.calibration_size();
            r2r::log_warn!(
                &self.logger,
                "La calibración ({}x{}) no coincide con la imagen ({}x{}); Z será incorrecta sin camera_info",
                cal.width,
                cal.height,
                size.width,
                size.height
            );
            self.warned_size = true;
        }
        publish_image(&self.pubs.processed, Some(&img), "bgr8", &header)?;

        // Escala de la red y Z con cada imagen
        let outputs = EstimateOutputs {
            mask_all: has_subscribers(&self.pubs.blob_bin),
            overlay: has_subscribers(&self.pubs.blob_overlay),
            ..Default::default()
        };
        let net = self.estimator.estimate(&img, self.cfg.mesh_cell_m, &[], outputs)?;
        if let Some(scale) = net.scale_m_per_px {
            if self.geo.calibrated() {
                self.latest_z = Some(self.geo.z_from_scale(scale));
                self.latest_z_time = self.now();
            }
            self.pubs.stats.publish(&NetStats {
                header: header.clone(),
                scale_m_per_px: scale as f32,
                median_hole_area_px: net.median_area_px as f32,
                median_hole_area_m2: (net.median_area_px * scale * scale) as f32,
            })?;
            publish_image(&self.pubs.blob_bin, net.mask_all.as_ref(), "mono8", &header)?;
            publish_image(&self.pubs.blob_overlay, net.overlay.as_ref(), "bgr8", &header)?;
        }

        // YOLO cada period_s
        let t = self.now();
        if let Some(last) = self.last_infer {
            if t - last < self.cfg.period_s {
                return Ok(());
            }
        }
        self.last_infer = Some(t);
        if let Err(e) = self.detect(&img, &header, t) {
            if self.throttle.ready("inferencia") {
                r2r::log_error!(&self.logger, "Error en la inferencia: {:#}", e);
            }
        }
        Ok(())
    }

    fn detect(&mut self, img: &Mat, header: &Header, t: f64) -> anyhow::Result<()> {
        let lb = letterbox(img, self.backend.input_size())?;
        let (out, shape) = self.backend.infer(&lb.blob)?;
        if shape.len() != 3 || shape[1] < 5 {
            bail!("salida del modelo con forma inesperada {shape:?}");
        }
        let decoded = decode(&out, shape[1] as usize, shape[2] as usize, self.cfg.conf_threshold as f32);
        let dets = top_k(
            unletterbox(nms(&decoded, self.cfg.iou_threshold as f32), &lb, img.size()?),
            self.cfg.max_detections,
        );

        // Cajas normalizadas [0, 1], como el bbox_detector de ROS 1
        let (cols, rows) = (img.cols() as f64, img.rows() as f64);
        let boxes = BoundingBoxArray {
            header: header.clone(),
            boxes: dets
                .iter()
                .map(|d| {
                    let b = &d.box_px;
                    BoundingBox {
                        class_id: d.class_id.to_string(),
                        score: d.score as f64,
                        x: (b.x as f64 + b.width as f64 / 2.0) / cols,
                        y: (b.y as f64 + b.height as f64 / 2.0) / rows,
                        w: b.width as f64 / cols,
                        h: b.height as f64 / rows,
                    }
                })
                .collect(),
        };
        self.pubs.boxes.publish(&boxes)?;
        if dets.is_empty() {
            return Ok(());
        }

        // 3D con la última Z (si es reciente)
        let z = if !self.geo.calibrated() {
            if self.throttle.ready("sin_calibracion") {
                r2r::log_warn!(&self.logger, "Sin calibración de cámara: no se puede proyectar a 3D");
            }
            None
        } else if self.latest_z.is_none() {
            if self.throttle.ready("sin_z") {
                r2r::log_warn!(&self.logger, "Agujero detectado pero sin Z (no se ve la red)");
            }
            None
        } else if t - self.latest_z_time > self.cfg.max_z_age_s {
            if self.throttle.ready("z_antigua") {
                r2r::log_warn!(
                    &self.logger,
                    "Z demasiado antigua ({:.2} s); no se publica en 3D",
                    t - self.latest_z_time
                );
            }
            None
        } else {
            self.latest_z
        };

        if let Some(z) = z {
            let detections = dets
                .iter()
                .map(|d| {
                    let b = &d.box_px;
                    let (u, v) = (b.x + b.width / 2.0, b.y + b.height / 2.0);
                    let p = self.geo.project(u as f64, v as f64, z);
                    let size = self.geo.size_m(b.width as f64, b.height as f64, z);
                    Detection3D {
                        class_id: d.class_id.to_string(),
                        score: d.score as f64,
                        x: p.x,
                        y: p.y,
                        z: p.z,
                        width: size.width,
                        height: size.height,
                    }
                })
                .collect();
            self.pubs.dets.publish(&Detection3DArray { header: header.clone(), detections })?;
        }

        if has_subscribers(&self.pubs.debug) {
            publish_image(&self.pubs.debug, Some(&self.draw_debug(img, &dets, z)?), "bgr8", header)?;
        }
        if has_subscribers(&self.pubs.fused) {
            let rects: Vec<Rect2f> = dets.iter().map(|d| d.box_px).collect();
            let only_yolo = EstimateOutputs { mask_yolo: true, ..Default::default() };
            let fused = self.estimator.estimate(img, self.cfg.mesh_cell_m, &rects, only_yolo)?;
            publish_image(&self.pubs.fused, fused.mask_yolo.as_ref(), "mono8", header)?;
        }
        Ok(())
    }

    fn draw_debug(&self, img: &Mat, dets: &[Detection2D], z: Option<f64>) -> opencv::Result<Mat> {
        let green = Scalar::new(0.0, 255.0, 0.0, 0.0);
        let mut out = img.try_clone()?;
        for d in dets {
            let b = &d.box_px;
            let r = Rect::new(b.x.round() as i32, b.y.round() as i32, b.width.round() as i32, b.height.round() as i32);
            imgproc::rectangle(&mut out, r, green, 2, imgproc::LINE_8, 0)?;
            let Some(z) = z else {
                continue;
            };
            let label = format!("Z:{:.2}m W:{:.2}m", z, self.geo.size_m(b.width as f64, 0.0, z).width);
            let mut baseline = 0;
            let ts = imgproc::get_text_size(&label, imgproc::FONT_HERSHEY_SIMPLEX, 0.5, 1, &mut baseline)?;
            imgproc::rectangle_points(
                &mut out,
                Point::new(r.x, r.y - 20),
                Point::new(r.x + ts.width, r.y),
                green,
                imgproc::FILLED,
                imgproc::LINE_8,
                0,
            )?;
            imgproc::put_text(
                &mut out,
                &label,
                Point::new(r.x, r.y - 5),
                imgproc::FONT_HERSHEY_SIMPLEX,
                0.5,
                Scalar::all(0.0),
                1,
                imgproc::LINE_8,
                false,
            )?;
        }
        Ok(out)
    }
}

// Las imágenes solo se generan y publican si alguien está suscrito
fn has_subscribers(publisher: &Publisher<Image>) -> bool {
    publisher.get_inter_process_subscription_count().unwrap_or(0) > 0
}

fn publish_image(publisher: &Publisher<Image>, img: Option<&Mat>, encoding: &str, header: &Header) -> anyhow::Result<()> {
    let Some(img) = img else {
        return Ok(());
    };
    if img.empty() || !has_subscribers(publisher) {
        return Ok(());
    }
    publisher.publish(&cv_bridge::mat_to_image(img, encoding, header)?)?;
    Ok(())
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let ctx = r2r::Context::create()?;
    let mut node = Node::create(ctx, "net_hole_detector", "")?;

    // Servicios de parámetros (ros2 param ...) y use_sim_time: hay que ejecutar su future
    let (param_handler, _param_events) = node.make_parameter_handler()?;
    tokio::spawn(param_handler);

    let (mut detector, mut images) = NetHoleDetector::new(&mut node)?;
    let mut camera_info = node.subscribe::<CameraInfo>("camera_info", QosProfile::sensor_data().keep_last(1))?;

    // spin_once en un hilo aparte: mueve los mensajes de ROS a los streams de las suscripciones
    let logger = node.logger().to_string();
    let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let spin_running = running.clone();
    let spinner = tokio::task::spawn_blocking(move || {
        while spin_running.load(std::sync::atomic::Ordering::Relaxed) {
            node.spin_once(Duration::from_millis(10));
        }
    });

    // Se crea una sola vez: si se creara en cada vuelta, un Ctrl+C que llegue mientras se
    // procesa una imagen se perdería
    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);
    loop {
        tokio::select! {
            Some(info) = camera_info.next() => detector.on_camera_info(&info),
            Some(mut img) = images.next() => {
                // Nos quedamos con la más reciente de las que hayan llegado mientras procesábamos
                while let Some(Some(newer)) = images.next().now_or_never() {
                    img = newer;
                }
                detector.on_input(img);
            }
            _ = &mut ctrl_c => break,
            else => break,
        }
    }

    r2r::log_info!(&logger, "Cerrando");
    running.store(false, std::sync::atomic::Ordering::Relaxed);
    spinner.await.map_err(|e| anyhow!("el hilo de spin terminó mal: {e}"))?;
    Ok(())
}
