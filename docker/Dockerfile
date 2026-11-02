# Imagen del detector de agujeros en la red, versión Rust (ROS 2 Humble + r2r + ONNX Runtime).
#   ORT_VARIANT=gpu : x86 con NVIDIA. TensorRT, CUDA y CPU (CUDA 12, cuDNN 9, TensorRT 10.4 incluidos)
#   ORT_VARIANT=cpu : solo CPU, x86 o arm64 (Raspberry Pi) según la arquitectura del build
# Se construye desde la raíz del repo:  docker compose build   (o docker build -f docker/Dockerfile .)
ARG ROS_DISTRO=humble

# ---------- build ----------
FROM ros:${ROS_DISTRO}-ros-base AS build
ARG ORT_VARIANT=gpu
ARG TARGETARCH
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Rust (rustup) + libclang y pkg-config, que usan r2r (bindgen) y el crate opencv al compilar
RUN apt-get update && apt-get install -y --no-install-recommends curl clang libclang-dev pkg-config \
    && rm -rf /var/lib/apt/lists/*
ENV RUSTUP_HOME=/opt/rustup CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:$PATH
RUN curl -fsSL --retry 3 https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain stable

# ONNX Runtime en /opt/onnxruntime. El nodo no lo enlaza: lo carga al arrancar desde ahí
COPY net_hole_detector_rust/tools/install_onnxruntime.sh /tmp/install_onnxruntime.sh
RUN if [ "$ORT_VARIANT" = gpu ]; then v=x64-gpu; \
    elif [ "$TARGETARCH" = arm64 ]; then v=aarch64; else v=x64; fi; \
    bash /tmp/install_onnxruntime.sh "$v" /opt/onnxruntime

WORKDIR /ws
COPY net_hole_detector_msgs_rust src/net_hole_detector_msgs_rust
COPY net_hole_detector_rust src/net_hole_detector_rust
RUN apt-get update && rosdep update --rosdistro "$ROS_DISTRO" \
    && rosdep install --from-paths src --ignore-src -y --rosdistro "$ROS_DISTRO" \
       --dependency-types buildtool --dependency-types build --dependency-types build_export \
       --dependency-types exec \
    && rm -rf /var/lib/apt/lists/*
# Caché de BuildKit para los crates descargados y lo compilado (target/): reconstruir tras
# cambiar el código solo recompila el crate. La instalación copia el binario fuera de la caché.
RUN --mount=type=cache,target=/opt/cargo/registry \
    --mount=type=cache,target=/ws/src/net_hole_detector_rust/target \
    source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --merge-install --install-base /opt/nhd_ws \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# ---------- runtime ----------
FROM ros:${ROS_DISTRO}-ros-base
ARG ORT_VARIANT=gpu
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Solo dependencias de ejecución (se resuelven con los package.xml)
COPY net_hole_detector_msgs_rust/package.xml /tmp/deps/net_hole_detector_msgs_rust/package.xml
COPY net_hole_detector_rust/package.xml /tmp/deps/net_hole_detector_rust/package.xml
RUN apt-get update && rosdep update --rosdistro "$ROS_DISTRO" \
    && rosdep install --from-paths /tmp/deps --ignore-src -y --rosdistro "$ROS_DISTRO" \
       --dependency-types exec \
    && if [ "$ORT_VARIANT" = gpu ]; then apt-get install -y --no-install-recommends python3-pip; fi \
    && rm -rf /var/lib/apt/lists/* /tmp/deps

# Librerías de NVIDIA de usuario (el driver lo pone el host con el NVIDIA Container Toolkit)
RUN if [ "$ORT_VARIANT" = gpu ]; then \
      pip3 install --no-cache-dir --target /opt/gpu_libs \
        nvidia-cuda-runtime-cu12 nvidia-cublas-cu12 "nvidia-cudnn-cu12==9.*" nvidia-cufft-cu12 \
        nvidia-curand-cu12 nvidia-cuda-nvrtc-cu12 "tensorrt-cu12-libs==10.4.*"; \
    fi

COPY --from=build /opt/onnxruntime /opt/onnxruntime
COPY --from=build /opt/nhd_ws /opt/nhd_ws
COPY docker/entrypoint.sh /entrypoint.sh

ENV NHD_ENGINE_DEFAULT=${ORT_VARIANT} ONNXRUNTIME_ROOT=/opt/onnxruntime
ENTRYPOINT ["/entrypoint.sh"]
