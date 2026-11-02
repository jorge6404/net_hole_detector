# net_hole_detector_rust

Port a **Rust** ([r2r](https://github.com/sequenceplanner/r2r)) del paquete C++ `net_hole_detector`: detecta agujeros rotos en redes con YOLO-World (ONNX Runtime) y estima su posición 3D respecto a la cámara a partir del tamaño de las celdas de la red. ROS 2 Humble.

El algoritmo, los parámetros (`config/net_hole_detector.yaml`) y los topics son los mismos que en C++; el README de `net_hole_detector` explica el método. Aquí solo se cuenta lo que cambia.

**¿Empiezas con Rust?** Lee [`docs/guia_rust.md`](docs/guia_rust.md): explica todo lo necesario para entender este código.

## Paquetes

| Paquete | Contenido |
|---|---|
| `net_hole_detector_msgs_rust` | `BoundingBox(Array)`, `Detection3D(Array)`, `NetStats` (copia de `net_hole_detector_msgs` con otro nombre) |
| `net_hole_detector_rust` | crate con una librería sin ROS (`src/lib.rs`) y el nodo `net_hole_detector_node` (`src/main.rs`) |

```
net_hole_detector_rust/
├── Cargo.toml              ← dependencias de Rust (r2r, opencv, ort, tokio, ...)
├── .cargo/config.toml      ← IDL_PACKAGE_FILTER: qué mensajes genera r2r
├── CMakeLists.txt          ← colcon llama a cargo a través de r2r_cargo.cmake
├── src/
│   ├── lib.rs                  ← declara los módulos de la librería
│   ├── camera_geometry.rs      ← camera_geometry.{hpp,cpp}
│   ├── clahe.rs                ← clahe.{hpp,cpp}
│   ├── scale_estimator.rs      ← scale_estimator.{hpp,cpp}
│   ├── yolo_postprocess.rs     ← yolo_postprocess.{hpp,cpp} (NMS hecho a mano, sin cv::dnn)
│   ├── inference_backend.rs    ← inference_backend.hpp (trait)
│   ├── onnxruntime_backend.rs  ← onnxruntime_backend.{hpp,cpp} (crate ort)
│   ├── cv_bridge.rs            ← lo que hacía cv_bridge (Image <-> Mat)
│   └── main.rs                 ← net_hole_detector_node.cpp
├── config/  launch/  models/  test/
└── tools/install_onnxruntime.sh
docker/                     ← Dockerfile, entrypoint.sh, fastdds_udp.xml
docker-compose.yml
```

## Requisitos

- Rust (`rustup`), `libopencv-dev`, `libclang-dev` (los bindings de r2r y opencv se generan con libclang).
- ONNX Runtime 1.20 (el mismo que la versión C++, `tools/install_onnxruntime.sh` de `net_hole_detector`). No se enlaza al compilar: se carga al arrancar desde `ORT_DYLIB_PATH`, si no `$ONNXRUNTIME_ROOT/lib/libonnxruntime.so`, si no `/opt/onnxruntime/lib/libonnxruntime.so`. Para GPU hacen falta las mismas librerías de CUDA/cuDNN/TensorRT en el `LD_LIBRARY_PATH`.

## Docker (lo más sencillo)

Igual que en la versión C++: la imagen lleva ROS 2 Humble, el paquete compilado, ONNX Runtime y, en la variante GPU, CUDA 12 + cuDNN 9 + TensorRT 10.4. En el host solo hace falta Docker (y, para GPU, el driver de NVIDIA y el NVIDIA Container Toolkit). Rust se instala solo en la etapa de build de la imagen.

```bash
docker compose up -d                                                    # PC con NVIDIA (TensorRT → CUDA → CPU)
docker compose --profile cpu up -d net_hole_detector_rust_cpu           # PC sin GPU o Raspberry Pi (solo CPU)
docker compose logs -f                                                  # ver qué motor ha quedado activo
```

- Usa la red del host y `ROS_DOMAIN_ID=4`, así que ve los topics del simulador y del robot.
- FastDDS va solo por UDP (`docker/fastdds_udp.xml`): el contenedor corre como root y los procesos del host no podrían abrir sus segmentos de memoria compartida.
- Topics, motor, modelo, etc. se cambian en el bloque `x-env` del `docker-compose.yml`; los parámetros finos, en `net_hole_detector_rust/config/net_hole_detector.yaml`, que está montado. En ambos casos basta con `docker compose up -d` de nuevo (o `restart`), sin reconstruir.
- `net_hole_detector_rust/models/` también está montado: para usar el modelo grande, cópialo ahí y pon `MODEL_PATH: models/yolov8_hole_detector_10_best.onnx`.
- Los engines de TensorRT se guardan en el volumen `trt_cache`: solo el primer arranque tarda (≈ 2 min).
- La imagen de CPU se construye para la arquitectura de la máquina: si la construyes en la Pi, sale para arm64.
- El primer build descarga y compila todos los crates (varios minutos). Los siguientes reutilizan la caché de BuildKit y solo recompilan el crate.
- Con argumentos, el contenedor ejecuta ese comando en vez del launch: `docker compose run --rm net_hole_detector_rust_cpu bash`.

## Compilar, probar y lanzar

```bash
cd sura_ws_meta
colcon build --packages-select net_hole_detector_msgs_rust net_hole_detector_rust
source install/setup.bash
ros2 launch net_hole_detector_rust net_hole_detector.launch.py \
    image_topic:=/bluerov/stonefish/front_camera/image_color \
    camera_info_topic:=/bluerov/stonefish/front_camera/camera_info image_transport:=raw
```

Test de integración (nodo completo en CPU con bgr8, mono8 y compressed):

```bash
colcon test --packages-select net_hole_detector_rust && colcon test-result --verbose
```

Para compilar solo con `cargo` (`cargo build`, `cargo check`) hace falta el paquete de mensajes compilado y el workspace sourceado: r2r genera los mensajes a partir de lo que encuentra en `AMENT_PREFIX_PATH`.

## Diferencias con la versión C++

- **Mensajes:** publica `net_hole_detector_msgs_rust/*`, que para ROS es un tipo distinto de `net_hole_detector_msgs/*` aunque los campos sean iguales. Un consumidor del nodo C++ no puede suscribirse al de Rust sin cambiar de paquete de mensajes. Por defecto el launch usa el namespace `net_hole_detector_rust` para que los dos puedan funcionar a la vez.
- **Transporte compressed:** se escucha `image/compressed` (relativo al namespace). r2r no deja resolver el remap de `image` para añadirle `/compressed`, así que el launch remapea los dos nombres. Si lanzas el nodo a mano, remapea `image/compressed:=<topic>/compressed`.
- **Imágenes de salida:** solo raw (`sensor_msgs/Image`). image_transport no existe en r2r, así que no hay variantes `/compressed` de las imágenes de depuración.
- **Parámetros:** se leen al arrancar. Cambiarlos con `ros2 param set` no afecta al nodo en marcha (en C++ tampoco).
- **Componente:** no hay componente de `rclcpp_components`, solo el ejecutable.
- **NMS:** implementado en Rust con el mismo algoritmo que `cv::dnn::NMSBoxes`. Al portarlo se comprobó contra la referencia de Ultralytics de `net_hole_detector` (`test/data/net_sample_expected.json`): da las mismas cajas.
