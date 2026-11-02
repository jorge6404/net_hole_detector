#!/usr/bin/env bash
# Descarga ONNX Runtime precompilado y lo deja en <prefijo>/{include,lib}.
#   x64-gpu : PC x86 con NVIDIA (proveedores TensorRT, CUDA y CPU; necesita CUDA 12 + cuDNN 9, y TensorRT 10 para TRT)
#   x64     : PC x86 sin GPU
#   aarch64 : Raspberry Pi (solo CPU)
# En Jetson se usa el build de ONNX Runtime GPU para JetPack (ver README) y se apunta ONNXRUNTIME_ROOT a él.
set -euo pipefail

VERSION=1.20.1
variant="${1:-}"
prefix="${2:-/opt/onnxruntime}"

case "$variant" in
  x64-gpu|x64|aarch64) ;;
  *)
    echo "Uso: $0 <x64-gpu|x64|aarch64> [prefijo=/opt/onnxruntime]" >&2
    exit 1
    ;;
esac

name="onnxruntime-linux-${variant}-${VERSION}"
url="https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/${name}.tgz"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

echo "Descargando $url"
curl -fsSL --retry 3 -o "$tmp/ort.tgz" "$url"
tar -xzf "$tmp/ort.tgz" -C "$tmp"

mkdir -p "$prefix"
cp -a "$tmp/$name/include" "$tmp/$name/lib" "$prefix/"
echo "ONNX Runtime $VERSION ($variant) instalado en $prefix"
