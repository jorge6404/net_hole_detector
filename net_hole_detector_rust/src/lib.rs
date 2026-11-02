// Librería sin ROS: CLAHE, escala de la red, geometría, YOLO e inferencia.
// El nodo (main.rs) solo lee parámetros, convierte mensajes y publica.
pub mod camera_geometry;
pub mod clahe;
pub mod inference_backend;
pub mod onnxruntime_backend;
pub mod scale_estimator;
pub mod yolo_postprocess;
