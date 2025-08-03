build:
  #!/bin/bash

  cd build && emmake make -j12
  if ! tail -n 5 ./uv-unwrap.js | grep -q "export { UVUnwrap }"; then
    echo "export { UVUnwrap }" >> ./uv-unwrap.js
  fi

  wasm-opt uv-unwrap.wasm -g -O4 --enable-simd --enable-nontrapping-float-to-int --precompute-propagate --detect-features --strip-dwarf -c -o uv-unwrap.wasm --enable-bulk-memory --enable-nontrapping-float-to-int

install:
  cp build/uv-unwrap.* ~/dream/src/viz/wasm/uv_unwrap
