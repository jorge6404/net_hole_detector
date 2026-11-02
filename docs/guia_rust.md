# Rust para entender `net_hole_detector_rust`

Esta guía explica lo que hace falta saber de Rust para leer y modificar los nodos de este repo. Todos los ejemplos salen del código del paquete, así que puedes abrir el archivo y ver cada cosa en su contexto. Si ya sabes C++, la [chuleta del final](#20-chuleta-c--rust) traduce cada construcción.

Orden recomendado: lee los apartados 1 a 9 seguidos (son la base), y luego consulta el resto cuando te los encuentres en el código.

---

## Índice

1. [Cargo: el proyecto, las dependencias y cómo lo compila colcon](#1-cargo-el-proyecto-las-dependencias-y-cómo-lo-compila-colcon)
2. [Módulos: `mod`, `use`, `pub`](#2-módulos-mod-use-pub)
3. [Variables: `let`, `mut`, `const`, `static`](#3-variables-let-mut-const-static)
4. [Tipos básicos y conversiones con `as`](#4-tipos-básicos-y-conversiones-con-as)
5. [Tipos compuestos: arrays, tuplas, slices, `Vec`, `String`, mapas](#5-tipos-compuestos-arrays-tuplas-slices-vec-string-mapas)
6. [Funciones, expresiones y bloques](#6-funciones-expresiones-y-bloques)
7. [Structs, `impl` y métodos (`self`, `&self`, `&mut self`)](#7-structs-impl-y-métodos-self-self-mut-self)
8. [Enums y `match`](#8-enums-y-match)
9. [`Option`, `Result` y el operador `?`](#9-option-result-y-el-operador-)
10. [Propiedad y préstamos (ownership y borrowing)](#10-propiedad-y-préstamos-ownership-y-borrowing)
11. [Traits y genéricos](#11-traits-y-genéricos)
12. [Closures](#12-closures)
13. [Iteradores](#13-iteradores)
14. [Macros y `format!`](#14-macros-y-format)
15. [Errores con `anyhow`](#15-errores-con-anyhow)
16. [Concurrencia y async: `Arc`, `Mutex`, tokio, streams](#16-concurrencia-y-async-arc-mutex-tokio-streams)
17. [r2r: ROS 2 desde Rust](#17-r2r-ros-2-desde-rust)
18. [El crate `opencv`](#18-el-crate-opencv)
19. [Tests](#19-tests)
20. [Chuleta C++ → Rust](#20-chuleta-c--rust)
21. [Recorrido del nodo de principio a fin](#21-recorrido-del-nodo-de-principio-a-fin)
22. [Cómo seguir](#22-cómo-seguir)

---

## 1. Cargo: el proyecto, las dependencias y cómo lo compila colcon

**Cargo** es el gestor de proyectos de Rust: compila, descarga dependencias y ejecuta los tests. Hace lo que en C++ hacen CMake y apt juntos. Un proyecto de Cargo se llama **crate**.

`Cargo.toml` describe el crate:

```toml
[package]
name = "net_hole_detector_rust"
edition = "2021"            # versión del lenguaje (como -std=c++17)

[lib]                       # una librería: src/lib.rs
path = "src/lib.rs"

[[bin]]                     # un ejecutable: src/main.rs  →  net_hole_detector_node
name = "net_hole_detector_node"
path = "src/main.rs"

[profile.colcon]            # perfil propio que usa colcon (igual que release)
inherits = "release"

[dependencies]              # crates que se descargan de crates.io
r2r = "0.9"                 # "0.9" = cualquier 0.9.x
tokio = { version = "1", features = ["full"] }
ort = { version = "=2.0.0-rc.9", ... }   # "=" fija la versión exacta
```

- **features**: partes opcionales de un crate. Por ejemplo `opencv` solo compila los módulos `imgproc` e `imgcodecs` que pedimos.
- **`Cargo.lock`**: guarda la versión exacta de cada dependencia que se usó. Va en git para que todos compilen lo mismo.
- **`target/`**: carpeta de compilación (como `build/`). No va en git.

Comandos que usarás:

```bash
cargo build            # compila en modo debug  → target/debug/
cargo build --release  # optimizado             → target/release/
cargo test             # compila y pasa los tests unitarios (si los hay)
cargo run -- --ros-args -p inference.engine:=cpu   # compila y ejecuta; lo que va tras -- son argumentos del programa
cargo check            # solo comprueba errores, sin generar binario (rápido)
cargo clippy           # linter: sugiere código más idiomático
cargo doc --open       # documentación de tu crate y de todas sus dependencias, en el navegador
```

**Cómo lo compila colcon.** colcon no sabe compilar Rust. `CMakeLists.txt` incluye `r2r_cargo.cmake`, que busca los paquetes de ROS que le pasas a `r2r_cargo(...)`, mete sus rutas en variables de entorno y llama a `cargo build --profile colcon`. Después, CMake instala `target/colcon/net_hole_detector_node` en `install/.../lib/net_hole_detector_rust/`, que es donde lo busca `ros2 run`. Por eso existe `dummy.c`: es un truco para que CMake calcule las rutas de las librerías de C de ROS.

**`.cargo/config.toml`** define variables de entorno para el build. `IDL_PACKAGE_FILTER` le dice a r2r de qué paquetes debe generar código Rust para los mensajes; si no, generaría todos los del sistema y tardaría mucho más.

---

## 2. Módulos: `mod`, `use`, `pub`

Cada archivo `.rs` es un **módulo**. Hay que declararlo para que exista:

```rust
// src/lib.rs
pub mod camera_geometry;   // carga src/camera_geometry.rs como módulo público
pub mod clahe;
...
```

```rust
// src/main.rs
mod cv_bridge;             // módulo privado del ejecutable (src/cv_bridge.rs)
```

`use` trae nombres al ámbito actual, como `using` en C++:

```rust
use std::collections::HashMap;                      // de la librería estándar
use opencv::core::{Mat, Point, Rect, Scalar};      // varios a la vez
use opencv::prelude::*;                             // todo lo que exporte prelude
use net_hole_detector_rust::clahe::{Clahe, ClaheParams};  // de nuestra librería
use crate::inference_backend::InferenceBackend;     // crate:: = la raíz de este crate
use super::*;                                       // super:: = el módulo padre
```

- `::` separa rutas, igual que en C++ (`opencv::core::Mat` ≈ `cv::Mat`).
- **Todo es privado por defecto.** `pub` lo hace visible fuera del módulo. En `CameraGeometry` los campos no son `pub`, así que desde fuera solo se puede usar a través de sus métodos `pub fn`, igual que con `private:` en una clase de C++.
- `main.rs` usa la librería por el nombre del crate (`net_hole_detector_rust::...`), como si fuera una dependencia externa. Así la librería no depende de ROS y se puede probar sola.

---

## 3. Variables: `let`, `mut`, `const`, `static`

```rust
let share = package_share_directory();   // inmutable: no se puede reasignar
let mut out = Mat::default();            // mut: se puede modificar
out = otra_cosa;                         // ok porque es mut
```

**Por defecto todo es inmutable**, como si en C++ escribieras `const` en todas partes. Solo pones `mut` cuando lo necesitas, y el compilador avisa si pusiste `mut` sin necesidad.

**Inferencia de tipos.** Casi nunca hace falta escribir el tipo: el compilador lo deduce (como `auto`). Se escribe cuando no se puede deducir o para aclarar:

```rust
let mut sqrt_areas = Vec::new();            // Vec<f64>: lo deduce cuando haces push(area.sqrt())
let rects: Vec<Rect2f> = dets.iter().map(|d| d.box_px).collect();  // collect necesita saber a qué tipo
```

**Shadowing**: puedes declarar otra variable con el mismo nombre, que tapa a la anterior. Se usa mucho para "transformar" un valor:

```rust
let mat = if mat.is_continuous() { mat } else { ... };   // en cv_bridge::mat_to_image
```

**Constantes:**

```rust
const PACKAGE: &str = "net_hole_detector_rust";            // valor fijo, conocido al compilar
const CHAIN: [(&str, &str); 3] = [("tensorrt", "TensorrtExecutionProvider"), ...];
static INIT: OnceLock<Result<(), String>> = OnceLock::new();   // una variable global (una sola instancia)
```

Las constantes van en `MAYÚSCULAS`, las variables y funciones en `snake_case` y los tipos en `CamelCase`.

---

## 4. Tipos básicos y conversiones con `as`

| Rust | C++ | Uso en el código |
|---|---|---|
| `i32`, `i64` | `int32_t`, `int64_t` | tamaños de imagen (`i32`), parámetros enteros de ROS (`i64`) |
| `u8`, `u32` | `uint8_t`, `uint32_t` | píxeles (`u8`), `width`/`height` de `sensor_msgs/Image` (`u32`) |
| `usize` | `size_t` | índices y longitudes (`vec.len()` devuelve `usize`) |
| `f32`, `f64` | `float`, `double` | scores y cajas (`f32`), geometría (`f64`) |
| `bool` | `bool` | |
| `char` | (UTF-8, 4 bytes) | casi no se usa aquí |
| `()` | `void` | "no devuelve nada": `Result<(), Error>` |

**Rust no convierte tipos numéricos solo.** En C++ `int + double` funciona; en Rust es un error de compilación. Hay que convertir con `as`:

```rust
let aspect = r.width as f64 / r.height as f64;      // i32 → f64
let input_size = declare_int(node, "model.input_size", 640)? as i32;   // i64 → i32
scale_m_per_px: scale as f32,                        // f64 → f32
```

Cuidado: `as` entre enteros trunca sin avisar (`300_i32 as u8` da `44`) y de float a entero redondea hacia cero y satura (`-1.5 as u32` da `0`). Por eso en `draw_debug` se hace `b.x.round() as i32`, que redondea al entero más cercano antes de convertir.

**Literales:** `0.0` es float y `0` es entero: `let mut out = [0.0; N];` crea floats. Puedes poner el tipo como sufijo (`1u8`, `0.5f32`) y usar `_` para separar dígitos (`1_000_000`).

**Floats y comparación:** `f64` no tiene un orden total (por `NaN`), así que no se puede usar `.sort()` directamente. Para ordenar se usa `total_cmp`:

```rust
dets.sort_by(|a, b| b.score.total_cmp(&a.score));   // de mayor a menor
```

---

## 5. Tipos compuestos: arrays, tuplas, slices, `Vec`, `String`, mapas

**Array de tamaño fijo** `[T; N]`, como `std::array<T, N>`:

```rust
k: [f64; 9],                 // la matriz K
let p = [0.0; 12];           // 12 ceros
self.p[5]                    // acceso (comprueba el límite: si te sales, el programa para con un panic)
```

**Tupla**: varios valores de tipos distintos sin nombre, como `std::tuple`/`std::pair`:

```rust
let (b, g, r) = (pixels[3 * i], pixels[3 * i + 1], pixels[3 * i + 2]);   // destructuring
fn infer(&mut self, blob: &[f32]) -> anyhow::Result<(Vec<f32>, Vec<i64>)>  // devuelve dos cosas
let (out, shape) = self.backend.infer(&lb.blob)?;
```

**`Vec<T>`**: array dinámico, como `std::vector<T>`:

```rust
let mut dets = Vec::new();
dets.push(d);
dets.len();
dets.truncate(k);            // como resize(k) cuando k es menor
vec![0.0; 3 * s * s]         // macro vec!: un Vec con 3·s·s ceros
```

**Slice `&[T]`**: una vista prestada de una secuencia (de un `Vec`, de un array o de parte de uno), como `std::span<const T>`. Es la forma normal de recibir "una lista" en una función:

```rust
pub fn decode(data: &[f32], rows: usize, anchors: usize, ...)   // acepta &Vec<f32>, &[f32; N], ...
&msg.data[..row * h]          // los primeros row·h bytes
&CHAIN[start..]               // desde start hasta el final
```

Rangos: `a..b` va de `a` a `b-1`, `a..=b` incluye `b`, y `..b`/`a..` dejan un extremo abierto.

**Texto: `String` y `&str`.**

- `String`: texto propio y modificable (como `std::string`).
- `&str`: una vista prestada de un texto (como `std::string_view`). Los literales `"hola"` son `&'static str`.

```rust
fn declare_string(node: &Node, name: &str, default: &str) -> anyhow::Result<String>
//                                ^ recibe vistas          devuelve uno propio ^
"compressed".into()            // &str → String (también .to_string() o String::from(...))
msg.encoding.as_str()          // String → &str
format!("{prefix}/share/{PACKAGE}")   // construye un String
```

Regla práctica: los parámetros de funciones suelen ser `&str` y los campos de structs `String`.

**Mapas:**

```rust
use std::collections::{HashMap, BTreeMap};
let mut last: HashMap<&'static str, Instant>;         // sin orden (unordered_map)
let mut by_class: BTreeMap<i32, Vec<Detection2D>>;   // ordenado por clave (std::map)
by_class.entry(d.class_id).or_default().push(*d);    // "busca la clave; si no está, inserta un Vec vacío"
```

En `nms` se usa `BTreeMap` para recorrer las clases en orden, igual que el `std::map` del C++.

---

## 6. Funciones, expresiones y bloques

```rust
fn make_odd(k: i32, min: i32) -> i32 {
    min.max(if k % 2 == 0 { k + 1 } else { k })
}
```

- `fn nombre(param: Tipo, ...) -> TipoDeRetorno`. Sin `->`, devuelve `()`.
- **La última expresión sin `;` es el valor de retorno.** `return` solo hace falta para salir antes de tiempo.
- **Casi todo es una expresión**, incluidos `if`, `match` y los bloques `{ }`. Por eso `if` puede ir dentro de una llamada (no hay operador ternario `?:`, se usa `if`):

```rust
let blue = if image.channels() == 1 {
    image.try_clone()?
} else {
    let mut b = Mat::default();
    core::extract_channel(image, &mut b, 0)?;
    b                     // valor del bloque else
};
```

- `min.max(...)`: los números tienen métodos. `a.max(b)` es `std::max(a, b)`.
- **No hay sobrecarga de funciones ni parámetros por defecto.** Por eso el crate `opencv` tiene `cvt_color` (todos los parámetros) y `cvt_color_def` (los opcionales con su valor por defecto).
- **Etiquetas en bucles:** `loop { ... }` es un bucle infinito del que se sale con `break`; `while cond { }` y `for x in iterable { }` funcionan como esperas. `continue` salta a la siguiente vuelta.

---

## 7. Structs, `impl` y métodos (`self`, `&self`, `&mut self`)

Rust no tiene clases. Tiene **structs** (los datos) y bloques **`impl`** (los métodos). No hay herencia: el comportamiento común se comparte con traits (apartado 11).

```rust
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Intrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}
```

**`#[derive(...)]`** le pide al compilador que escriba código por ti:

| derive | Qué da | Equivalente C++ |
|---|---|---|
| `Debug` | imprimir con `{:?}` | `operator<<` para depurar |
| `Clone` | `.clone()`, copia explícita | constructor de copia |
| `Copy` | se copia solo al asignar (solo tipos pequeños sin memoria dinámica) | tipo trivialmente copiable |
| `Default` | `Tipo::default()`, todo a cero/vacío | constructor por defecto |
| `PartialEq` | `==` y `!=` | `operator==` |

**Crear un struct**: se dan todos los campos por nombre.

```rust
Intrinsics { fx: self.p[0], fy: self.p[5], cx: self.p[2], cy: self.p[6] }
```

**Sintaxis de actualización `..`**: el resto de campos se toma de otro valor. Así se imitan los valores por defecto de los campos de C++:

```rust
let only_yolo = EstimateOutputs { mask_yolo: true, ..Default::default() };
let p = ClaheParams { grid_size: 4, ..Default::default() };
```

**Implementar `Default` a mano**, cuando los valores por defecto no son cero:

```rust
impl Default for ClaheParams {
    fn default() -> Self {
        Self { clip_limit: 2.5, grid_size: 8 }     // Self = el tipo que se está implementando
    }
}
```

**Métodos:**

```rust
impl CameraGeometry {
    pub fn calibrated(&self) -> bool { self.calibrated }       // &self: solo lee   (método const de C++)
    pub fn select(&mut self, is_rectified: bool) { ... }        // &mut self: modifica el objeto
    pub fn new(params: ScaleEstimatorParams) -> Self { ... }    // sin self: función asociada (static)
}
```

- `&self` ≈ `const` en C++, `&mut self` ≈ no-const, y sin `self` ≈ `static`.
- `self` (sin `&`) consume el objeto: después de la llamada ya no se puede usar.
- Se llaman con `.` (`geo.select(true)`) y las funciones asociadas con `::` (`ScaleEstimator::new(sp)`).
- **No hay constructores.** Por convención se escribe una función `new` que devuelve `Self`, o `Result<Self>` si puede fallar (`Clahe::new`, `OrtBackend::new`).
- **No hay destructores explícitos**: cuando un valor sale de su ámbito se libera solo (RAII, como en C++). Para hacer algo especial al destruir se implementa el trait `Drop`. `drop(x)` lo libera antes de tiempo.

---

## 8. Enums y `match`

Un `enum` de Rust es mucho más potente que el de C++: cada variante puede llevar datos (como un `std::variant`, pero mucho más cómodo de usar).

```rust
// en r2r
pub enum ParameterValue {
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    ...
}

// en main.rs: la imagen llega por uno de los dos topics
enum InputImage {
    Raw(Image),
    Compressed(CompressedImage),
}
```

**`match`** mira qué variante es y saca sus datos. El compilador obliga a cubrir todos los casos:

```rust
fn declare_number(node: &Node, name: &str, default: f64) -> anyhow::Result<f64> {
    match declare(node, name, ParameterValue::Double(default)) {
        ParameterValue::Double(d) => Ok(d),          // si es Double, d es el f64 de dentro
        ParameterValue::Integer(i) => Ok(i as f64),  // acepta 1 además de 1.0
        other => bail!("... no {other:?}"),          // cualquier otra: la guarda en `other`
    }
}
```

`match` también sirve con textos, números y tuplas, y es una expresión que devuelve valor:

```rust
let (channels, conversion) = match msg.encoding.as_str() {
    "bgr8" => (3, None),
    "rgb8" => (3, Some(imgproc::COLOR_RGB2BGR)),
    other => bail!("codificación '{other}' no soportada"),
};
```

- `_` es "cualquier cosa, no me interesa": `_ => vec![]`.
- **Guardas**: `Some(t) if now.duration_since(*t) < ... => false` solo entra si se cumple la condición (en `Throttle::ready`).

**`if let`**: un `match` de un solo caso.

```rust
if let Some(scale) = net.scale_m_per_px {   // solo entra si hay escala, y la saca en `scale`
    ...
}
```

**`let ... else`**: saca el valor o sale de la función. Es el patrón "early return" más limpio:

```rust
let Ok(text) = std::fs::read_to_string(path) else {
    return false;                 // el else tiene que salir (return, break, continue...)
};
// aquí text ya es el String leído
```

```rust
let Some(z) = z else {
    continue;                     // en draw_debug: sin Z, siguiente caja
};
```

---

## 9. `Option`, `Result` y el operador `?`

Este es **el apartado más importante**. Rust no tiene `null` ni excepciones. En su lugar usa dos enums de la librería estándar:

```rust
enum Option<T> { Some(T), None }        // hay un valor, o no lo hay   (≈ std::optional<T>)
enum Result<T, E> { Ok(T), Err(E) }     // salió bien con T, o falló con el error E
```

Como son tipos, **el compilador no te deja usar el valor sin mirar antes si está**. En C++ un `std::optional` sin comprobar o una excepción no capturada solo fallan en ejecución.

### `Option` en el código

```rust
pub scale_m_per_px: Option<f64>,   // None si no se ve la red
latest_z: Option<f64>,             // None hasta que se calcula la primera Z
pub mask_all: Option<Mat>,         // None si nadie la pidió (en C++ era un Mat vacío)
```

### `Result` en el código

Cada función que puede fallar devuelve `Result`:

```rust
pub fn new(params: &ClaheParams) -> opencv::Result<Self>      // opencv::Result<T> = Result<T, opencv::Error>
fn detect(&mut self, ...) -> anyhow::Result<()>               // () = no devuelve nada si va bien
```

### El operador `?`

`?` va detrás de una expresión de tipo `Result` u `Option` y significa: **"si es `Ok`/`Some`, dame lo de dentro y sigue; si es `Err`/`None`, sal de la función ahora devolviendo ese error/`None`"**.

```rust
let data = node.get("data")?.as_sequence()?;
```

equivale a:

```rust
let data = match node.get("data") {
    Some(d) => d,
    None => return None,          // ← el ? sale de read_matrix devolviendo None
};
let data = match data.as_sequence() {
    Some(s) => s,
    None => return None,
};
```

Esto es lo que hace tu `read_matrix`: `?` no comprueba que sea `None`, sino que **sale de la función devolviendo `None` en cuanto algo es `None`**, y si no, continúa con el valor de dentro. Por eso la función devuelve `Option<[f64; N]>`: `?` sobre un `Option` solo se puede usar dentro de una función que también devuelve `Option`.

Con `Result` funciona igual:

```rust
pub fn apply(&mut self, bgr: &Mat) -> opencv::Result<Mat> {
    let mut lab = Mat::default();
    imgproc::cvt_color_def(bgr, &mut lab, imgproc::COLOR_BGR2Lab)?;  // si OpenCV falla, apply devuelve ese Err
    ...
    Ok(out)                       // al final, el valor bueno envuelto en Ok
}
```

Comparado con C++: es como si cada llamada que puede lanzar una excepción la tuvieras que marcar con `?`. El error se propaga hacia arriba igual, pero se ve en el código por dónde puede salir.

`?` también **convierte el tipo de error** si hace falta: en una función que devuelve `anyhow::Result`, un `?` sobre un `opencv::Result` convierte el `opencv::Error` en `anyhow::Error` automáticamente (gracias al trait `From`).

### Otras formas de sacar el valor

| Método | Qué hace | En el código |
|---|---|---|
| `.unwrap()` | da el valor o **para el programa** (panic) si es `None`/`Err` | `lock().unwrap()` |
| `.expect("msg")` | igual que `unwrap` pero con tu mensaje | `x.expect("debería haber escala")` |
| `.unwrap_or(v)` | da el valor o `v` | `.unwrap_or([0.0; 12])`, `.unwrap_or(0)` |
| `.unwrap_or_else(\|\| ...)` | como `unwrap_or`, pero calcula `v` solo si hace falta | `.unwrap_or_else(\|\| env!("CARGO_MANIFEST_DIR").to_string())` |
| `.unwrap_or_default()` | da el valor o el `Default` del tipo | `std::env::var("HOME").unwrap_or_default()` → `""` |
| `.map(f)` | transforma lo de dentro si existe | `.map(\|d\| d.as_secs_f64())` |
| `.ok_or_else(\|\| err)` | `Option` → `Result` | en `provider_chain` |
| `.ok()` | `Result` → `Option` (descarta el error) | `c[1].parse().ok()?` |
| `.is_some()`, `.is_none()`, `.is_ok()`, `.is_err()` | preguntar sin sacar el valor | `self.latest_z.is_none()` |
| `.as_ref()` | `&Option<T>` → `Option<&T>` (mirar sin quitarle el valor) | `net.mask_all.as_ref()` |
| `cond.then_some(v)` | `Some(v)` si `cond`, si no `None` | `(d.box_px.area() > 0.0).then_some(d)` |

**Regla:** en el código del nodo se usa `?` o se maneja el error. `unwrap()` solo se usa cuando fallar significa un bug (un `Mutex` envenenado, por ejemplo).

---

## 10. Propiedad y préstamos (ownership y borrowing)

Es lo que más cuesta al principio y lo que hace a Rust distinto. Son tres reglas que comprueba el compilador:

1. **Cada valor tiene un único dueño.** Cuando el dueño sale de ámbito, el valor se libera.
2. **Asignar o pasar por valor *mueve* el valor** (como `std::move` en C++, pero automático). El original ya no se puede usar.
3. **Puedes prestar el valor**: o muchas referencias de solo lectura `&T`, o **una sola** referencia mutable `&mut T`, pero nunca las dos a la vez.

```rust
let a = String::from("hola");
let b = a;              // a se mueve a b
println!("{a}");        // ERROR de compilación: a ya no es válido
```

### Mover, prestar, copiar

```rust
fn median(mut v: Vec<f64>) -> f64            // recibe el Vec por valor: se lo queda (lo va a reordenar)
let median_l_px = median(sqrt_areas);        // sqrt_areas se mueve y ya no se usa después

pub fn estimate(&self, image: &Mat, ...)     // &Mat: prestado, solo lectura
fn on_image(&mut self, bgr: &Mat, mut header: Header)   // header por valor: es nuestra copia y la cambiamos
on_image(&bgr, header)                       // & al llamar = prestar
imgproc::cvt_color_def(bgr, &mut lab, ...)   // &mut = prestar para que lo modifique (parámetro de salida)
```

- En la llamada también se escribe `&` / `&mut`, así se ve al leer qué se presta y qué se modifica.
- **Tipos `Copy`** (números, `bool`, structs pequeños con `#[derive(Copy)]` como `Detection2D` o `Rect2f`) no se mueven: se copian. Por eso `dets.iter().map(|d| d.box_px)` funciona sin `clone`.
- **`.clone()`** hace una copia explícita (y a veces cara). En el código: `header.clone()`, `self.cfg.camera_frame.clone()`.
- `*` desreferencia: `*slot = v.as_f64()?;` escribe en el elemento al que apunta `slot`; `by_class.entry(...).push(*d)` copia el `Detection2D` al que apunta `d`.

### Ejemplos del código donde se nota

**`Mat::try_clone()`**: en el crate `opencv`, copiar un `Mat` puede fallar (reserva memoria en C++), así que no se llama `clone()` sino `try_clone()?`, que devuelve `Result`.

**Un bloque para acabar un préstamo** (en `letterbox`):

```rust
{
    // roi presta `padded` de forma mutable hasta el final de este bloque
    let mut roi = padded.roi_mut(Rect::new(left, top, new_w, new_h))?;
    resized.copy_to(&mut roi)?;
}
let pixels = padded.data_bytes()?;   // ya podemos leer padded: el préstamo mutable terminó
```

Mientras `roi` existe hay un `&mut padded` vivo, y la regla 3 no deja leer `padded` a la vez. Cerrar el bloque destruye `roi` y libera el préstamo.

**`drop(meta)`** (en `OrtBackend::from_session`): `meta` toma prestada la sesión, y al final hay que *mover* la sesión dentro del struct. No se puede mover algo prestado, así que primero se suelta el préstamo.

**`Cow`** (Clone on Write, en `cv_bridge::image_to_bgr`): "o te presto los datos, o te doy una copia mía". Si las filas de la imagen no tienen relleno usamos los bytes del mensaje directamente (`Cow::Borrowed`); si lo tienen, construimos un `Vec` nuevo sin el relleno (`Cow::Owned`). El resto del código no necesita saber cuál de los dos es.

### Lifetimes (`'a`, `'static`)

Una referencia no puede vivir más que lo que apunta, y el compilador lo comprueba con **lifetimes**. Casi siempre los deduce solo. Los únicos que verás aquí:

- `&'static str`: una referencia que vale durante todo el programa, como los literales de texto. `HashMap<&'static str, Instant>` en `Throttle` guarda claves que son literales (`"sin_z"`).
- `BoxStream<'static, InputImage>`: un stream que no toma prestado nada temporal, así que puede guardarse y usarse durante todo el programa.

Si el compilador te pide un lifetime, normalmente la solución es guardar un valor propio (`String`, `Vec`) en vez de una referencia.

---

## 11. Traits y genéricos

Un **trait** es un conjunto de métodos que un tipo promete tener. Es lo más parecido a una clase abstracta (interfaz) de C++:

```rust
// inference_backend.rs  (en C++: class InferenceBackend { virtual ... = 0; })
pub trait InferenceBackend {
    fn infer(&mut self, blob: &[f32]) -> anyhow::Result<(Vec<f32>, Vec<i64>)>;
    fn provider(&self) -> &str;
    fn input_size(&self) -> i32;
    fn class_names(&self) -> &[String];
}

// onnxruntime_backend.rs  (en C++: class OrtBackend : public InferenceBackend)
impl InferenceBackend for OrtBackend {
    fn infer(&mut self, blob: &[f32]) -> ... { ... }
    ...
}
```

Para llamar a un método de un trait, el trait tiene que estar importado con `use`. Por eso el nodo hace `use net_hole_detector_rust::inference_backend::InferenceBackend;` y por eso todo el código de OpenCV hace `use opencv::prelude::*;`, que trae los traits con los métodos de `Mat` (`rows()`, `channels()`, ...).

### `Box<dyn Trait>`: polimorfismo en ejecución

```rust
backend: Box<dyn InferenceBackend>,       // ≈ std::unique_ptr<InferenceBackend>
backend: Box::new(backend),               // ≈ std::make_unique<OrtBackend>(...)
```

`dyn Trait` = "algún tipo que implementa el trait, se sabrá en ejecución" (llamadas virtuales). `Box` = puntero propio al heap.

### Genéricos

```rust
fn read_matrix<const N: usize>(node: &Value) -> Option<[f64; N]>   // ≈ template<size_t N>
read_matrix::<9>(&root["camera_matrix"])                          // ::<9> ("turbofish") fija N
node.subscribe::<Image>("image", qos)                             // el tipo del mensaje
```

Restricciones con traits (≈ concepts de C++20):

```rust
pub fn new(model_path: &str, opts: &OrtOptions, info: impl Fn(&str), warn: impl Fn(&str))
//                                                    ^ cualquier tipo que se pueda llamar con un &str
```

`impl Trait` en un parámetro es un genérico abreviado. La forma larga sería `fn new<F: Fn(&str)>(..., info: F)` o con `where F: Fn(&str)`.

**Traits de la librería estándar que verás:** `Default`, `Clone`, `Debug`, `PartialEq` (los de `derive`), `From`/`Into` (conversiones: `"cpu".into()`), `Iterator`, `Fn` (closures), `Send`/`Sync` (seguro entre hilos), `Drop` (destructor).

---

## 12. Closures

Funciones anónimas, como las lambdas de C++:

```rust
|x| x + 1                                    // un parámetro, devuelve x + 1
|a, b| b.score.total_cmp(&a.score)           // dos parámetros
|m| r2r::log_info!(&logger, "{}", m)         // captura `logger` del entorno
|| -> anyhow::Result<Self> { ... }           // sin parámetros, con tipo de retorno explícito
```

- Capturan automáticamente lo que usan del entorno (en C++ habría que escribir `[&]` o `[=]`). Por defecto lo toman prestado.
- `move |...| ...` obliga a capturar por valor (mover). Se usa cuando la closure va a otro hilo que puede vivir más que la función actual: `spawn_blocking(move || { ... node.spin_once(...) ... })` se lleva `node` al hilo.

**Una closure que devuelve `Result`** (en `OrtBackend::new`):

```rust
let attempt = || -> anyhow::Result<Self> {
    let mut builder = Session::builder()?...;      // este ? sale de la closure, no de new()
    ...
    Ok(backend)
};
match attempt() {
    Ok(backend) => return Ok(backend),
    Err(e) => warn(&format!("{p} no disponible: {e:#}; probando el siguiente")),
}
```

Esto sustituye al `try { ... } catch` del C++: cualquier fallo dentro de `attempt` (el `?`) termina solo ese intento, y el bucle prueba el siguiente motor.

---

## 13. Iteradores

En Rust se recorre casi todo con iteradores y métodos encadenados, en lugar de bucles con índice. Son perezosos (no hacen nada hasta que algo los consume) y el compilador los optimiza tanto como un bucle a mano.

```rust
for (slot, v) in out.iter_mut().zip(data) {    // recorre out (mutable) y data a la vez
    *slot = v.as_f64()?;
}
```

| Método | Qué hace |
|---|---|
| `.iter()` | recorre prestando cada elemento (`&T`) |
| `.iter_mut()` | prestando de forma mutable (`&mut T`) |
| `.into_iter()` | consumiendo la colección (da `T`) — es lo que hace `for x in vec` |
| `.map(f)` | transforma cada elemento |
| `.filter(f)` | se queda con los que cumplen `f` |
| `.filter_map(f)` | `map` + `filter`: `f` devuelve `Option` y se descartan los `None` |
| `.zip(otro)` | empareja con otro iterador |
| `.enumerate()` | da `(índice, elemento)` |
| `.take(n)`, `.step_by(n)` | los n primeros / uno de cada n |
| `.chunks(n)` | trozos de n elementos (de un slice) |
| `.flat_map(f)` | `map` donde `f` devuelve varios, y los aplana |
| `.copied()` | `&T` → `T` para tipos `Copy` |
| `.any(f)`, `.all(f)` | ¿alguno / todos cumplen? |
| `.position(f)` | índice del primero que cumple (`Option<usize>`) |
| `.find(f)` | el primero que cumple (`Option`) |
| `.fold(inicial, f)` | acumula (como `std::accumulate`) |
| `.collect()` | consume y construye una colección (`Vec`, `BTreeMap`, `String`, ...) |

Ejemplos del código:

```rust
// ¿alguna P distinta de cero?   (std::any_of)
self.has_p = p.iter().any(|&v| v != 0.0);

// motores disponibles a partir del pedido
CHAIN[start..]
    .iter()
    .filter(|(_, ort_name)| ort_available.contains(ort_name))
    .map(|(name, _)| name.to_string())
    .collect()

// quitar el relleno de cada fila de la imagen
msg.data.chunks(step).take(h).flat_map(|r| &r[..row]).copied().collect()

// cajas del letterbox → cajas de la imagen, descartando las vacías
dets.into_iter().filter_map(|mut d| { ...; (d.box_px.area() > 0.0).then_some(d) }).collect()
```

`|&v|` y `|(name, _)|` son **patrones** en los parámetros: desempaquetan la referencia o la tupla directamente.

---

## 14. Macros y `format!`

Lo que termina en `!` es una **macro**: código que genera código al compilar. Se reconocen por el `!`.

| Macro | Para qué |
|---|---|
| `println!`, `eprintln!` | imprimir en stdout / stderr |
| `format!` | construir un `String` |
| `vec![...]` | crear un `Vec` |
| `assert!`, `assert_eq!`, `assert_ne!` | comprobar condiciones (si fallan, panic); sobre todo en tests |
| `panic!` | parar el programa con un mensaje (un bug, no un error esperado) |
| `bail!`, `ensure!`, `anyhow!` | errores de `anyhow` (apartado 15) |
| `r2r::log_info!` (`log_warn!`, `log_error!`, `log_fatal!`) | logs de ROS, como `RCLCPP_INFO` |
| `ort::inputs![...]` | entradas de una sesión de ONNX Runtime |
| `env!("X")`, `concat!(...)` | variable de entorno **al compilar** / unir literales |
| `tokio::select!`, `tokio::pin!` | async (apartado 16) |
| `#[derive(...)]`, `#[test]`, `#[cfg(test)]`, `#[tokio::main]` | atributos: anotan el elemento que les sigue |

**Formato** (como `printf`/`std::format`):

```rust
format!("{} y {}", a, b)              // por posición
format!("{prefix}/share/{PACKAGE}")   // variables por nombre, directamente en el texto
"{:.1}"    // 1 decimal:            "fx={:.1}"  → fx=937.1
"{:.2}"    // 2 decimales:          "Z:{:.2}m"
"{:?}"     // formato de depuración (necesita Debug): imprime structs, Vec, enums...
"{e:#}"    // formato alternativo: con anyhow, el error con toda su cadena de causas
"{{" "}}"  // llaves literales
```

`env!("CARGO_MANIFEST_DIR")` da al compilar la ruta absoluta de la carpeta del crate. `package_share_directory()` la usa como último recurso para encontrar `config/` y `models/` cuando el nodo se ejecuta con `cargo run` sin instalar.

---

## 15. Errores con `anyhow`

Cada librería tiene su tipo de error (`opencv::Error`, `ort::Error`, `std::io::Error`, `r2r::Error`...). El crate **`anyhow`** da un único tipo, `anyhow::Error`, en el que cabe cualquiera de ellos, y es el que se usa en el nodo y en el backend:

```rust
fn detect(&mut self, ...) -> anyhow::Result<()>    // = Result<(), anyhow::Error>
```

- **`?`** convierte cualquier error en `anyhow::Error`.
- **`bail!("...")`**: `return Err(anyhow!("..."))`, el equivalente a `throw std::runtime_error("...")`.
- **`ensure!(cond, "...")`**: si no se cumple `cond`, hace `bail!`. Es un `assert` que devuelve error en vez de parar el programa.
- **`anyhow!("...")`**: crea el error sin devolverlo (para `ok_or_else`, `map_err`).
- **`.context("...")` / `.with_context(|| format!(...))`**: añade una explicación al error que sube:

```rust
std::fs::create_dir_all(&cache).with_context(|| format!("no se puede crear la caché {cache}"))?;
// el mensaje final: "no se puede crear la caché /proc/...: Permission denied (os error 13)"
```

- Al imprimir, `{e}` da el último mensaje y `{e:#}` toda la cadena de causas.

**¿Qué pasa si `main` devuelve `Err`?** `async fn main() -> anyhow::Result<()>`: si un `?` falla en el arranque (por ejemplo, no hay modelo), `main` termina, se imprime el error y el proceso sale con código 1. Equivale a que el constructor del nodo de C++ lance una excepción que nadie captura.

`.inspect_err(|e| log_fatal!(...))` (en `NetHoleDetector::new`) hace algo con el error (escribirlo en el log) sin consumirlo; el `?` de detrás lo sigue propagando.

---

## 16. Concurrencia y async: `Arc`, `Mutex`, tokio, streams

### Compartir datos entre hilos

- **`Arc<T>`** (Atomic Reference Counted) ≈ `std::shared_ptr<T>`: varios dueños, `.clone()` copia el puntero (no los datos).
- **`Mutex<T>`**: el `T` está *dentro* del mutex, así que no se puede tocar sin bloquear. `lock()` devuelve un guard que da acceso y desbloquea al destruirse:

```rust
let mut params = node.params.lock().unwrap();   // node.params: Arc<Mutex<IndexMap<String, Parameter>>>
params.entry(name.to_string()).or_insert_with(|| Parameter::new(default))
// al salir de la función, `params` se destruye y el mutex se libera
```

  El `unwrap()` es porque `lock()` falla si otro hilo hizo panic con el mutex cogido ("envenenado").

- **`AtomicBool`**: un `bool` que se puede leer/escribir desde varios hilos sin mutex. En `main` avisa al hilo de spin de que pare:

```rust
let running = Arc::new(AtomicBool::new(true));
let spin_running = running.clone();          // otra referencia al mismo bool para el hilo
// hilo: while spin_running.load(Ordering::Relaxed) { node.spin_once(...) }
running.store(false, Ordering::Relaxed);     // main: para
```

- **`OnceLock<T>`**: un valor que se inicializa una sola vez, aunque lo pidan varios hilos (como un `static` local de C++). ONNX Runtime se inicializa así en `init_runtime`.

Rust **no compila** un programa que comparta datos entre hilos de forma insegura (traits `Send` y `Sync`). Las condiciones de carrera de memoria son imposibles fuera de `unsafe`.

### async / await

Una función `async fn` no se ejecuta al llamarla: devuelve un **future**, una tarea que avanzará cuando alguien la espere con `.await`. Mientras espera (por ejemplo, a que llegue un mensaje), el hilo puede hacer otras cosas. Hace falta un **runtime** que ejecute los futures, y el más usado es **tokio**:

```rust
#[tokio::main]                        // arranca el runtime de tokio y ejecuta main dentro
async fn main() -> anyhow::Result<()> {
    tokio::spawn(param_handler);      // lanza un future en segundo plano (≈ un hilo ligero)
    let spinner = tokio::task::spawn_blocking(move || { ... });   // código bloqueante, en un hilo de verdad
    spinner.await;                    // espera a que termine
}
```

### Streams: las suscripciones de r2r

Un **`Stream`** es como un iterador asíncrono: va dando elementos a medida que llegan. En r2r, **cada suscripción es un stream de mensajes** (no hay callbacks como en rclcpp):

```rust
let mut camera_info = node.subscribe::<CameraInfo>("camera_info", qos)?;
while let Some(msg) = camera_info.next().await { ... }    // .next() necesita `use futures::StreamExt`
```

En el nodo se combinan con `.map(...).boxed()`, que convierte las dos suscripciones posibles (raw y compressed) en un único tipo `BoxStream<'static, InputImage>`.

### `tokio::select!`: esperar a varias cosas a la vez

```rust
loop {
    tokio::select! {
        Some(info) = camera_info.next() => detector.on_camera_info(&info),
        Some(mut img) = images.next() => { ...; detector.on_input(img); }
        _ = &mut ctrl_c => break,
        else => break,                // si todos los streams se cierran
    }
}
```

Espera a la vez a los tres futures y ejecuta la rama del primero que esté listo. Es el equivalente al executor de rclcpp que llama al callback que toque.

- `tokio::pin!(ctrl_c)`: fija el future en memoria para poder esperarlo con `&mut` en cada vuelta del bucle sin consumirlo. Si se creara un `ctrl_c()` nuevo en cada vuelta, un Ctrl+C que llegue mientras se procesa una imagen se perdería (esto pasó al probar el nodo).
- `images.next().now_or_never()`: mira si ya hay otro mensaje *sin esperar*. Devuelve `Some(Some(msg))` si lo hay. El bucle `while let Some(Some(newer)) = ...` vacía la cola y se queda con la imagen más reciente, imitando la QoS de profundidad 1 del C++.

---

## 17. r2r: ROS 2 desde Rust

r2r es un *binding* de la API de C de ROS 2 (`rcl`), no un port de rclcpp. La API es más pequeña y algunas cosas de rclcpp no existen.

### Contexto y nodo

```rust
let ctx = r2r::Context::create()?;                                // ≈ rclcpp::init (lee --ros-args)
let mut node = Node::create(ctx, "net_hole_detector", "")?;       // nombre y namespace
```

Los argumentos `--ros-args -r __ns:=/x -r __node:=y` y los remaps (`-r image:=/camara`) los aplica rcl, así que funcionan igual que en C++.

### Mensajes

r2r **genera los tipos Rust al compilar** a partir de los `.msg` de los paquetes que encuentra (filtrados por `IDL_PACKAGE_FILTER`). Están en `r2r::<paquete>::msg::<Tipo>`:

```rust
use r2r::sensor_msgs::msg::{CameraInfo, CompressedImage, Image};
use r2r::net_hole_detector_msgs_rust::msg::{BoundingBox, NetStats};
```

Son structs normales con campos públicos y `Default`. Los arrays de los `.msg` (también los de tamaño fijo, como `float64[9] k`) son `Vec`:

```rust
NetStats { header: header.clone(), scale_m_per_px: scale as f32, ..., }
<[f64; 9]>::try_from(msg.k.as_slice())      // Vec<f64> → [f64; 9], falla si no tiene 9
```

### Publicar

```rust
let pub_stats: Publisher<NetStats> = node.create_publisher("net_stats", QosProfile::default())?;
pub_stats.publish(&stats)?;
pub_stats.get_inter_process_subscription_count()?    // ≈ getNumSubscribers()
```

Un `Publisher` se puede guardar y usar lejos del nodo; por eso `NetHoleDetector` guarda los publicadores y el nodo se va al hilo de spin.

### Suscribirse

`node.subscribe::<T>(topic, qos)?` devuelve un **stream** (apartado 16). Los mensajes solo llegan al stream si alguien llama a `node.spin_once(timeout)` en bucle: por eso `main` lanza un hilo que solo hace eso.

### QoS

```rust
QosProfile::default()                       // reliable, depth 10
QosProfile::sensor_data().keep_last(1)      // ≈ rclcpp::SensorDataQoS().keep_last(1)
```

### Parámetros

r2r no tiene `declare_parameter`. Al crear el nodo, lee los parámetros de `--params-file`/`-p` y los deja en `node.params` (un `Arc<Mutex<IndexMap<String, Parameter>>>`). Las claves anidadas del YAML llegan con puntos (`"yolo.period_s"`). En `main.rs`, `declare(...)` busca el parámetro y, si no está, inserta el valor por defecto, y `declare_string`/`declare_bool`/`declare_int`/`declare_number` comprueban el tipo con `match`.

`node.make_parameter_handler()` crea los servicios `ros2 param list/get/set` y activa `use_sim_time`. Devuelve un future que hay que ejecutar (`tokio::spawn`).

### Logs y reloj

```rust
let logger = node.logger().to_string();
r2r::log_info!(&logger, "Modelo {} (motor {})", path, motor);      // ≈ RCLCPP_INFO
let clock = node.get_ros_clock();                                  // Arc<Mutex<Clock>>, respeta use_sim_time
clock.lock().unwrap().get_now()?                                   // Duration desde la época
```

r2r no tiene `RCLCPP_WARN_THROTTLE` ni `_ONCE`. El nodo usa el struct `Throttle` (como mucho un log cada 2 s por clave) y un `bool` (`warned_size`).

### Lo que no hay en r2r (y cómo se resolvió)

| rclcpp | Aquí |
|---|---|
| `cv_bridge` | `src/cv_bridge.rs` a mano |
| `image_transport` | suscripción directa a `image` o `image/compressed`; publicación solo raw |
| `resolve_topic_name` | el launch remapea `image` e `image/compressed` |
| `ament_index_cpp::get_package_share_directory` | `package_share_directory()` busca en `AMENT_PREFIX_PATH` |
| componentes | solo ejecutable |
| callbacks + executor | streams + `tokio::select!` + hilo con `spin_once` |

---

## 18. El crate `opencv`

Son bindings generados automáticamente de la API de C++, así que los nombres se parecen mucho:

| C++ | Rust |
|---|---|
| `cv::Mat` | `opencv::core::Mat` |
| `cv::cvtColor(src, dst, code)` | `imgproc::cvt_color_def(&src, &mut dst, code)?` |
| `cv::GaussianBlur(...)` | `imgproc::gaussian_blur_def(...)?` |
| `img.rows`, `img.cols` | `img.rows()`, `img.cols()` (métodos) |
| `img.at<uint8_t>(r, c)` | `*img.at_2d::<u8>(r, c)?` |
| `img(rect)` | `img.roi(rect)?` / `img.roi_mut(rect)?` |
| `img.clone()` | `img.try_clone()?` |
| `std::vector<cv::Point>` | `Vector<Point>` (el `std::vector` de C++, no el `Vec` de Rust) |
| `cv::Scalar(b, g, r)` | `Scalar::new(b, g, r, 0.0)`, `Scalar::all(v)` |
| `cv::Mat::zeros(size, type)` | `Mat::zeros_size(size, type)?.to_mat()?` |

Reglas:

- **Casi todas las funciones devuelven `opencv::Result`** (porque OpenCV puede lanzar excepciones en C++), así que llevan `?` detrás.
- **Las que tienen parámetros opcionales existen en dos versiones**: `nombre(...)` con todos y `nombre_def(...)` con los opcionales por defecto.
- **`use opencv::prelude::*;`** es obligatorio para tener los métodos de `Mat` (son traits).
- La entrada va como `&mat` y la salida como `&mut mat`, igual que en C++ con `const cv::Mat&` / `cv::Mat&`.
- `Mat::zeros`, `Mat::ones` y `roi` devuelven tipos intermedios (`MatExpr`, `BoxedRef<Mat>`). `.to_mat()?` convierte un `MatExpr` en `Mat`, y un `BoxedRef<Mat>` se usa como un `&Mat`.
- La documentación de cada función está en `cargo doc --open` → `opencv`.

---

## 19. Tests

Este repo no tiene tests unitarios; solo el de integración con ROS (`test/test_node.launch.py`, en Python como en el paquete C++), que se lanza con `colcon test`. Si añades tests en Rust, la forma habitual es un módulo al final del archivo que solo se compila con `cargo test`:

```rust
#[cfg(test)]                // solo existe en `cargo test`
mod tests {
    use super::*;           // todo lo del módulo de arriba, también lo privado

    #[test]
    fn pinhole() {
        let mut g = CameraGeometry::default();
        g.set_matrices([1000., 0., 640., 0., 1000., 360., 0., 0., 1.], [0.0; 12], Size::new(1280, 720));
        assert_eq!(g.z_from_scale(0.0005), 0.5);
    }
}
```

- Un test pasa si termina y falla si hace panic (un `assert` que no se cumple o un `unwrap()` sobre un error).
- `cargo test` los ejecuta todos en paralelo, `cargo test pinhole` solo los que contienen "pinhole" y `cargo test -- --nocapture` muestra los `println!`.

---

## 20. Chuleta C++ → Rust

| C++ | Rust |
|---|---|
| `int x = 5;` | `let x: i32 = 5;` o `let x = 5;` |
| `x = 6;` (no const) | `let mut x = 5; x = 6;` |
| `const double k = 1.0;` (global) | `const K: f64 = 1.0;` |
| `auto` | inferencia por defecto |
| `static_cast<double>(x)` | `x as f64` |
| `std::vector<T>` | `Vec<T>` |
| `std::array<T, N>` | `[T; N]` |
| `std::span<const T>` / `const std::vector<T>&` | `&[T]` |
| `std::string` / `std::string_view` | `String` / `&str` |
| `std::map` / `std::unordered_map` | `BTreeMap` / `HashMap` |
| `std::optional<T>` | `Option<T>` |
| `nullptr` | `None` |
| excepciones | `Result<T, E>` y `?` |
| `throw std::runtime_error(msg)` | `bail!(msg)` (anyhow) |
| `try { } catch (...) { }` | `match f() { Ok(v) => ..., Err(e) => ... }` o una closure con `?` |
| `const T&` | `&T` |
| `T&` | `&mut T` |
| `std::move(x)` | automático al pasar por valor |
| copia implícita | `.clone()` explícito (salvo tipos `Copy`) |
| `std::unique_ptr<T>` | `Box<T>` |
| `std::shared_ptr<T>` | `Arc<T>` (`Rc<T>` en un solo hilo) |
| `std::mutex` + dato | `Mutex<T>` (el dato dentro) |
| `class` | `struct` + `impl` |
| constructor | `fn new(...) -> Self` (convención) |
| destructor | automático; `impl Drop` si hace falta |
| método `const` | `fn f(&self)` |
| método `static` | `fn f()` sin `self` |
| clase abstracta / `virtual` | `trait` + `Box<dyn Trait>` |
| `template<typename T>` | `fn f<T: Trait>(...)` |
| `template<size_t N>` | `fn f<const N: usize>(...)` |
| lambda `[&](int x){...}` | closure `\|x\| ...` |
| `std::function<void(const std::string&)>` | `impl Fn(&str)` o `Box<dyn Fn(&str)>` |
| `for (auto& x : v)` | `for x in &v` / `for x in v.iter_mut()` |
| `for (int i = 0; i < n; ++i)` | `for i in 0..n` |
| `switch` | `match` |
| `namespace` | `mod` |
| `#include` + `using` | `use` |
| `printf("%.2f", x)` | `format!("{:.2}", x)` |
| `assert(x)` | `assert!(x)` |
| `RCLCPP_INFO(get_logger(), ...)` | `r2r::log_info!(&logger, ...)` |
| callback de suscripción | stream + `select!` |

---

## 21. Recorrido del nodo de principio a fin

Con todo lo anterior, así se lee `src/main.rs`:

1. **`main`** (`#[tokio::main] async fn main`): crea el contexto y el nodo, lanza el manejador de parámetros con `tokio::spawn`.
2. **`NetHoleDetector::new(&mut node)`** hace los 5 pasos del constructor del C++:
   1. lee los parámetros con `declare_*` (cada `?` aborta el arranque si un parámetro tiene un tipo incorrecto);
   2. crea `Clahe` y `ScaleEstimator`;
   3. carga la calibración de respaldo (`CameraGeometry::load_yaml`);
   4. crea el `OrtBackend` (prueba tensorrt → cuda → cpu);
   5. crea los publicadores y la suscripción a la imagen, y devuelve `(detector, images)`.
3. `main` se suscribe a `camera_info` y **mueve el nodo a un hilo** (`spawn_blocking`) que solo hace `spin_once`.
4. **Bucle `select!`**: por cada `CameraInfo` → `on_camera_info`; por cada imagen (la más reciente) → `on_input`.
5. **`on_input`** convierte el mensaje en `Mat` (`cv_bridge::image_to_bgr` o `decode_compressed`) y llama a **`on_image`**, que aplica CLAHE, publica `image_processed`, estima la escala con los blobs, actualiza Z, publica `net_stats` y, si pasó `period_s`, llama a **`detect`**.
6. **`detect`**: `letterbox` → `backend.infer` → `decode` → `nms` → `unletterbox` → `top_k`; publica `bounding_boxes` y, si hay una Z reciente, proyecta cada caja con `geo.project`/`geo.size_m` y publica `detections_3d`; las imágenes de depuración solo si hay suscriptores.
7. Ctrl+C → sale del bucle, para el hilo de spin y `main` devuelve `Ok(())`.

Los errores de una imagen concreta no tiran el nodo: `on_input`, `on_image` y `detect` devuelven `Result`, y quien las llama escribe el error en el log (con límite de frecuencia) y sigue con la siguiente imagen. Solo los errores del arranque terminan el proceso.

---

## 22. Cómo seguir

- **The Rust Programming Language** ("el libro"): <https://doc.rust-lang.org/book/> (hay traducción al español: <https://book.rustlang-es.org/>). Los capítulos 3–10 cubren casi todo lo de esta guía; el 4 (ownership) y el 9 (errores) son los más importantes.
- **Rust by Example**: <https://doc.rust-lang.org/rust-by-example/>, ejemplos cortos de cada concepto.
- **Rustlings**: <https://github.com/rust-lang/rustlings>, ejercicios pequeños que se corrigen solos en el terminal.
- **Async**: el tutorial de tokio, <https://tokio.rs/tokio/tutorial>.
- **r2r**: los ejemplos de <https://github.com/sequenceplanner/r2r/tree/master/r2r/examples> (publicadores, servicios, parámetros, acciones).
- **Documentación de las dependencias**: `cargo doc --open` desde la carpeta del crate. Tienes la documentación exacta de las versiones que usa el proyecto (r2r, opencv, ort, tokio...).

Ejercicios para practicar con este código:

1. Escribe un test unitario (apartado 19) en `camera_geometry.rs` que compruebe que `project` y `size_m` son coherentes (una caja de ancho `w` px centrada en `cx` mide `size_m(w, ..).width` metros).
2. Haz que `Throttle::ready` reciba el intervalo como parámetro en vez de usar 2 s fijos.
3. Añade el parámetro `debug.font_scale` y úsalo en `draw_debug` (pasos: `declare_number` en `new`, un campo en `Config`, usarlo en `get_text_size`/`put_text`).
4. Cambia `median` para que devuelva `Option<f64>` (`None` con un `Vec` vacío) y ajusta quien la llama con `?` o `if let`.
5. Ejecuta `cargo clippy` y entiende cada sugerencia que te dé.
