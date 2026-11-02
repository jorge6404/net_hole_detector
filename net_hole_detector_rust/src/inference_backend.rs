// Motor de inferencia de un modelo de detección con una entrada y una salida.
// Un trait es como una clase abstracta de C++: define qué métodos debe tener un motor.
pub trait InferenceBackend {
    // blob NCHW f32 [1, 3, S, S] con S = input_size(); devuelve la salida plana y su forma
    fn infer(&mut self, blob: &[f32]) -> anyhow::Result<(Vec<f32>, Vec<i64>)>;
    fn provider(&self) -> &str; // "tensorrt" | "cuda" | "cpu"
    fn input_size(&self) -> i32;
    fn class_names(&self) -> &[String];
}
