// Sesión de ONNX Runtime con el primer motor de la cadena que funcione: tensorrt -> cuda -> cpu
use std::collections::BTreeMap;
use std::path::Path;
use std::sync::OnceLock;

use anyhow::{anyhow, bail, Context as _};
use ort::execution_providers::{
    CPUExecutionProvider, CUDAExecutionProvider, ExecutionProvider, ExecutionProviderDispatch,
    TensorRTExecutionProvider,
};
use ort::session::builder::GraphOptimizationLevel;
use ort::session::Session;
use ort::value::{Tensor, ValueType};
use regex::Regex;

use crate::inference_backend::InferenceBackend;

#[derive(Debug, Clone)]
pub struct OrtOptions {
    pub engine: String, // tensorrt | cuda | cpu
    pub trt_fp16: bool,
    pub trt_cache_dir: String,
    pub device_id: i32,
    pub cpu_threads: i32, // 0 = los que elija ONNX Runtime
}

impl Default for OrtOptions {
    fn default() -> Self {
        Self {
            engine: "tensorrt".into(),
            trt_fp16: true,
            trt_cache_dir: "~/.ros/net_hole_detector/trt_cache".into(),
            device_id: 0,
            cpu_threads: 0,
        }
    }
}

// Orden de la cadena y nombre de cada motor en ONNX Runtime
const CHAIN: [(&str, &str); 3] = [
    ("tensorrt", "TensorrtExecutionProvider"),
    ("cuda", "CUDAExecutionProvider"),
    ("cpu", "CPUExecutionProvider"),
];

// Motores a probar, en orden, a partir del pedido. ort_available son los nombres de
// ONNX Runtime (TensorrtExecutionProvider, ...). Error si el motor no existe.
pub fn provider_chain(engine: &str, ort_available: &[&str]) -> anyhow::Result<Vec<String>> {
    let start = CHAIN
        .iter()
        .position(|(name, _)| *name == engine)
        .ok_or_else(|| anyhow!("motor de inferencia desconocido '{engine}' (tensorrt | cuda | cpu)"))?;
    Ok(CHAIN[start..]
        .iter()
        .filter(|(_, ort_name)| ort_available.contains(ort_name))
        .map(|(name, _)| name.to_string())
        .collect())
}

// "~/x" -> "$HOME/x"
pub fn expand_user(path: &str) -> String {
    if path == "~" || path.starts_with("~/") {
        let home = std::env::var("HOME").unwrap_or_default();
        return format!("{home}{}", &path[1..]);
    }
    path.to_string()
}

// libonnxruntime.so que se carga al arrancar: ORT_DYLIB_PATH, si no $ONNXRUNTIME_ROOT/lib
// (como el CMake de la versión C++), si no /opt/onnxruntime/lib
pub fn ort_library_path() -> String {
    if let Ok(p) = std::env::var("ORT_DYLIB_PATH") {
        return p;
    }
    let root = std::env::var("ONNXRUNTIME_ROOT").unwrap_or_else(|_| "/opt/onnxruntime".into());
    format!("{root}/lib/libonnxruntime.so")
}

// ONNX Runtime se inicializa una sola vez por proceso (OnceLock guarda el resultado)
fn init_runtime() -> anyhow::Result<()> {
    static INIT: OnceLock<Result<(), String>> = OnceLock::new();
    INIT.get_or_init(|| {
        let lib = ort_library_path();
        if !Path::new(&lib).exists() {
            // ort hace panic si no encuentra la librería: mejor un error con explicación
            return Err(format!("no existe {lib} (usa ORT_DYLIB_PATH u ONNXRUNTIME_ROOT)"));
        }
        ort::init_from(lib).with_name("net_hole_detector").commit().map(|_| ()).map_err(|e| e.to_string())
    })
    .clone()
    .map_err(|e| anyhow!(e))
}

// Nombres de ONNX Runtime de los motores que trae la librería cargada
fn ort_available() -> Vec<&'static str> {
    let mut out = Vec::new();
    if TensorRTExecutionProvider::default().is_available().unwrap_or(false) {
        out.push(CHAIN[0].1);
    }
    if CUDAExecutionProvider::default().is_available().unwrap_or(false) {
        out.push(CHAIN[1].1);
    }
    if CPUExecutionProvider::default().is_available().unwrap_or(true) {
        out.push(CHAIN[2].1);
    }
    out
}

fn tensorrt(opts: &OrtOptions) -> anyhow::Result<ExecutionProviderDispatch> {
    let cache = expand_user(&opts.trt_cache_dir);
    std::fs::create_dir_all(&cache).with_context(|| format!("no se puede crear la caché {cache}"))?;
    Ok(TensorRTExecutionProvider::default()
        .with_device_id(opts.device_id)
        .with_fp16(opts.trt_fp16)
        .with_engine_cache(true)
        .with_engine_cache_path(cache)
        .build()
        .error_on_failure())
}

fn cuda(opts: &OrtOptions) -> ExecutionProviderDispatch {
    CUDAExecutionProvider::default().with_device_id(opts.device_id).build().error_on_failure()
}

pub struct OrtBackend {
    session: Session,
    provider: String,
    input_name: String,
    output_name: String,
    input_size: i32,
    names: Vec<String>,
}

impl OrtBackend {
    // Error si el modelo no existe o ningún motor arranca
    pub fn new(
        model_path: &str, opts: &OrtOptions, info: impl Fn(&str), warn: impl Fn(&str),
    ) -> anyhow::Result<Self> {
        if !Path::new(model_path).exists() {
            bail!("no existe el modelo ONNX: {model_path}");
        }
        init_runtime()?;

        for p in provider_chain(&opts.engine, &ort_available())? {
            if p == "tensorrt" {
                info(&format!(
                    "TensorRT: si el engine no está en la caché se compila ahora (minutos); caché en {}",
                    expand_user(&opts.trt_cache_dir)
                ));
            }
            // Un closure que devuelve Result: el `?` de dentro sale del closure, no de new()
            let attempt = || -> anyhow::Result<Self> {
                let mut builder = Session::builder()?.with_optimization_level(GraphOptimizationLevel::Level3)?;
                if opts.cpu_threads > 0 {
                    builder = builder.with_intra_threads(opts.cpu_threads as usize)?;
                }
                let eps = match p.as_str() {
                    // los nodos que TensorRT no soporte van a CUDA
                    "tensorrt" => vec![tensorrt(opts)?, cuda(opts)],
                    "cuda" => vec![cuda(opts)],
                    _ => vec![],
                };
                let session = builder.with_execution_providers(eps)?.commit_from_file(model_path)?;
                let mut backend = Self::from_session(session, &p)?;
                // Calentamiento: con TensorRT aquí se construye el engine, y si falla se prueba el siguiente
                let s = backend.input_size as usize;
                backend.infer(&vec![0.0; 3 * s * s])?;
                Ok(backend)
            };
            match attempt() {
                Ok(backend) => {
                    info(&format!("Motor de inferencia: {p}"));
                    return Ok(backend);
                }
                Err(e) => warn(&format!("{p} no disponible: {e:#}; probando el siguiente")),
            }
        }
        bail!("ningún motor de inferencia pudo cargar {model_path}")
    }

    fn from_session(session: Session, provider: &str) -> anyhow::Result<Self> {
        let input_name = session.inputs[0].name.clone();
        let output_name = session.outputs[0].name.clone();
        let dims = |t: &ValueType| match t {
            ValueType::Tensor { dimensions, .. } => dimensions.clone(),
            _ => Vec::new(),
        };
        let in_shape = dims(&session.inputs[0].input_type);
        let out_shape = dims(&session.outputs[0].output_type);
        let mut input_size = if in_shape.len() == 4 && in_shape[2] > 0 { in_shape[2] as i32 } else { 640 };

        // Metadata de Ultralytics: names = "{0: 'hole'}", imgsz = "[640, 640]"
        let meta = session.metadata()?;
        if let Some(imgsz) = meta.custom("imgsz")? {
            if let Some(m) = Regex::new(r"(\d+)")?.captures(&imgsz) {
                input_size = m[1].parse()?;
            }
        }
        let mut names = Vec::new();
        if let Some(s) = meta.custom("names")? {
            let item = Regex::new(r#"(\d+)\s*:\s*['"]([^'"]*)['"]"#)?;
            let by_id: BTreeMap<i32, String> =
                item.captures_iter(&s).filter_map(|c| Some((c[1].parse().ok()?, c[2].to_string()))).collect();
            names = by_id.into_values().collect();
        }
        if names.is_empty() {
            let nc = if out_shape.len() == 3 && out_shape[1] > 4 { out_shape[1] - 4 } else { 1 };
            names = (0..nc).map(|i| i.to_string()).collect();
        }
        drop(meta); // la metadata toma prestada la sesión: hay que soltarla antes de moverla
        Ok(Self { session, provider: provider.to_string(), input_name, output_name, input_size, names })
    }
}

impl InferenceBackend for OrtBackend {
    fn infer(&mut self, blob: &[f32]) -> anyhow::Result<(Vec<f32>, Vec<i64>)> {
        let s = self.input_size as usize;
        if blob.len() != 3 * s * s {
            bail!("el blob tiene {} valores y el modelo espera 3x{s}x{s}", blob.len());
        }
        let input = Tensor::from_array(([1usize, 3, s, s], blob.to_vec()))?;
        let outputs = self.session.run(ort::inputs![self.input_name.as_str() => input]?)?;
        let (shape, data) = outputs[self.output_name.as_str()].try_extract_raw_tensor::<f32>()?;
        Ok((data.to_vec(), shape.to_vec()))
    }

    fn provider(&self) -> &str {
        &self.provider
    }

    fn input_size(&self) -> i32 {
        self.input_size
    }

    fn class_names(&self) -> &[String] {
        &self.names
    }
}
