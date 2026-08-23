#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>

#include "bff/mesh/MeshIO.h"
#include "bff/project/Bff.h"
#include "bff/project/ConePlacement.h"
#include "bff/project/Cutter.h"
#include "bff/project/Distortion.h"
#include "bff/project/Generators.h"
#include "bff/project/HoleFiller.h"

using namespace bff;

template <typename T> uint32_t getVecDataPtr(std::vector<T> &vec) {
  return reinterpret_cast<uint32_t>(vec.data());
}

struct UnwrapUVsOutput {
  std::vector<float> uvs;
  std::vector<float> verts;
  std::vector<uint32_t> indices;
  std::vector<float> tangents; // xyz + handedness w per vertex (glTF convention)
  std::string error;
  Model model;
  std::vector<uint8_t> isSurfaceMappedToSphere;
  std::vector<Vector> originalUvIslandCenters;
  std::vector<Vector> newUvIslandCenters;
  std::vector<uint8_t> isUvIslandFlipped;
  Vector modelMinBounds;
  Vector modelMaxBounds;

  explicit UnwrapUVsOutput(std::string errorMsg) : error(std::move(errorMsg)) {}

  explicit UnwrapUVsOutput(std::vector<float> uvs, std::vector<float> verts,
                           std::vector<uint32_t> indices, Model model,
                           std::vector<Vector> originalUvIslandCenters,
                           std::vector<Vector> newUvIslandCenters,
                           std::vector<uint8_t> isUvIslandFlipped,
                           Vector modelMinBounds, Vector modelMaxBounds,
                           std::vector<uint8_t> isSurfaceMappedToSphere)
      : uvs(std::move(uvs)), verts(std::move(verts)),
        indices(std::move(indices)), error(""), model(std::move(model)),
        isSurfaceMappedToSphere(std::move(isSurfaceMappedToSphere)),
        originalUvIslandCenters(std::move(originalUvIslandCenters)),
        newUvIslandCenters(std::move(newUvIslandCenters)),
        isUvIslandFlipped(std::move(isUvIslandFlipped)),
        modelMinBounds(modelMinBounds), modelMaxBounds(modelMaxBounds) {}

  // Non-copyable
  UnwrapUVsOutput(const UnwrapUVsOutput &) = delete;
  UnwrapUVsOutput &operator=(const UnwrapUVsOutput &) = delete;

  std::string getDistortionSvg() {
    Distortion::computeAreaScaling(model);

    std::stringstream svg;
    svg << "<svg viewBox='-0.005 -0.005 1.01 1.01' "
           "xmlns='http://www.w3.org/2000/svg'>";
    // add a black border around the valid area (0,0) to (1,1)
    svg << "<rect x='0' y='0' width='1' height='1' fill='none' "
           "stroke='black' "
           "stroke-width='0.001' />";
    svg << "<g>";

    for (int i = 0; i < model.size(); i++) {
      double lengthRatio = std::sqrt(model[i].areaRatio());

      Vector minExtent(modelMinBounds.x, modelMinBounds.y);
      double dx = modelMaxBounds.x - minExtent.x;
      double dy = modelMaxBounds.y - minExtent.y;
      double extent = std::max(dx, dy);
      minExtent.x -= (extent - dx) / 2.;
      minExtent.y -= (extent - dy) / 2.;

      // compute sphere radius if component has been mapped to a sphere
      double sphereRadius = 1.;
      if (isSurfaceMappedToSphere[i] == 1) {
        for (WedgeCIter w = model[i].wedges().begin();
             w != model[i].wedges().end(); w++) {
          sphereRadius = std::max(w->uv.norm(), sphereRadius);
        }
      }

      const std::vector<Face> &faces = model[i].faces;
      for (FaceCIter f = faces.begin(); f != faces.end(); f++) {
        if (f->fillsHole) {
          continue;
        }

        Vector rgb = Distortion::color(f, i, false);

        svg << "<polygon points='";
        HalfEdgeCIter h = f->halfEdge();
        do {
          Vector uv = h->wedge()->uv;

          if (isSurfaceMappedToSphere[i] == 1) {
            uv /= sphereRadius;
            uv.x = 0.5 + atan2(uv.z, uv.x) / (2 * M_PI);
            uv.y = 0.5 - asin(uv.y) / M_PI;

          } else {
            uv *= model[i].radius * lengthRatio;
          }

          // apply scaling for this UV island in the same way as in packing
          uv -= originalUvIslandCenters[i];
          if (isUvIslandFlipped[i] == 1) {
            uv = Vector(-uv.y, uv.x);
          }
          uv += newUvIslandCenters[i];
          uv -= minExtent;
          // if (normalizeUvs) {
          uv /= extent;
          // }

          svg << uv.x << "," << uv.y << " ";
          h = h->next();
        } while (h != f->halfEdge());

        svg << "' style='fill:rgb(" << int(rgb.x * 255) << ","
            << int(rgb.y * 255) << "," << int(rgb.z * 255)
            << ");stroke:black;stroke-width:0.001' />";
      }
    }

    svg << "</g></svg>";
    return svg.str();
  }
};

// mostly copied from the `CommandLine.cpp` file
bool loadModel(const std::vector<float> &positions,
               const std::vector<uint32_t> &indices, Model &model,
               std::string &error, std::vector<bool> &isSurfaceClosed) {
  std::vector<Vector> vecPositions;
  vecPositions.reserve(positions.size() / 3);
  for (size_t i = 0; i < positions.size(); i += 3) {
    vecPositions.emplace_back(positions[i], positions[i + 1], positions[i + 2]);
  }

  std::vector<int> intIndices;
  intIndices.reserve(indices.size());
  for (size_t i = 0; i < indices.size(); i++) {
    intIndices.push_back(static_cast<int>(indices[i]));
  }

  bool success =
      MeshIO::buildModelFromBuffers(vecPositions, intIndices, model, error);
  if (!success) {
    if (error.empty()) {
      error = "Failed to load model from buffers.";
    }
    return false;
  }

  int nMeshes = model.size();
  isSurfaceClosed.resize(nMeshes, 0);

  for (int i = 0; i < nMeshes; i++) {
    Mesh &mesh = model[i];
    int nBoundaries = (int)mesh.boundaries.size();

    if (nBoundaries >= 1) {
      // mesh has boundaries
      int eulerPlusBoundaries = mesh.eulerCharacteristic() + nBoundaries;

      if (eulerPlusBoundaries == 2) {
        // fill holes if mesh has more than 1 boundary
        if (nBoundaries > 1) {
          if (HoleFiller::fill(mesh)) {
            // all holes were filled
            isSurfaceClosed[i] = true;
          }
        }

      } else {
        // mesh probably has holes and handles
        HoleFiller::fill(mesh, true);
        Generators::compute(mesh);
      }

    } else if (nBoundaries == 0) {
      if (mesh.eulerCharacteristic() == 2) {
        // mesh is closed
        isSurfaceClosed[i] = 1;

      } else {
        // mesh has handles
        Generators::compute(mesh);
      }
    }
  }

  return true;
}

struct UvAlignment {
  Vector up, fallback;
  int quarterTurns;
};

bool flattenMesh(Mesh &mesh, bool isSurfaceClosed, int nCones,
                 bool flattenToDisk, bool mapToSphere, std::string &error,
                 const UvAlignment *align) {
  BFF bff(mesh);

  if (nCones > 0) {
    std::vector<VertexIter> cones;
    DenseMatrix coneAngles(bff.data->iN);
    int S = std::min(nCones, (int)mesh.vertices.size() - bff.data->bN);

    if (ConePlacement::findConesAndPrescribeAngles(S, cones, coneAngles,
                                                   bff.data, mesh) ==
        ConePlacement::ErrorCode::ok) {
      if (!isSurfaceClosed || cones.size() > 0) {
        Cutter::cut(cones, mesh);
        bff.flattenWithCones(coneAngles, true);
        if (align) {
          mesh.alignUvsToAxes(align->up, align->fallback, align->quarterTurns);
        }
      }
    }
  } else {
    if (isSurfaceClosed) {
      if (mapToSphere) {
        bff.mapToSphere();

      } else {
        error = "Surface is closed. Either specify nCones or mapToSphere.";
        return false;
      }

    } else {
      if (flattenToDisk) {
        bff.flattenToDisk();

      } else {
        DenseMatrix u(bff.data->bN);
        bff.flatten(u, true);
      }

      if (align) {
        mesh.alignUvsToAxes(align->up, align->fallback, align->quarterTurns);
      } else {
        mesh.projectUvsToPcaAxis();
      }
    }
  }

  return true;
}

// Also copied from `CommandLine.cpp`
bool flatten(Model &model, const std::vector<bool> &isSurfaceClosed,
             const std::vector<int> &nCones, bool flattenToDisk,
             bool mapToSphere, std::string &error, const UvAlignment *align) {
  int nMeshes = model.size();
  for (int i = 0; i < nMeshes; i++) {
    Mesh &mesh = model[i];
    bool ok = flattenMesh(mesh, isSurfaceClosed[i], nCones[i], flattenToDisk,
                          mapToSphere, error, align);
    if (!ok) {
      if (error.empty()) {
        error = "Failed to flatten mesh " + std::to_string(i) + ".";
      }
      return false;
    }
  }

  return true;
}

// Per-vertex tangents (xyz + handedness w) from the packed buffers via Lengyel's method.
// Normals are recomputed here (area-weighted, matching three's `computeVertexNormals`) purely to
// orthogonalize the tangent; the renderer keeps using its own normals for shading.
static std::vector<float> computeTangents(const std::vector<float> &positions,
                                          const std::vector<float> &uvs,
                                          const std::vector<uint32_t> &indices) {
  size_t nVerts = positions.size() / 3;
  std::vector<float> tan(nVerts * 3, 0.f), bit(nVerts * 3, 0.f), nrm(nVerts * 3, 0.f);

  for (size_t t = 0; t + 2 < indices.size(); t += 3) {
    uint32_t tri[3] = {indices[t], indices[t + 1], indices[t + 2]};
    uint32_t a = tri[0], b = tri[1], c = tri[2];
    float ax = positions[a * 3], ay = positions[a * 3 + 1], az = positions[a * 3 + 2];
    float e1x = positions[b * 3] - ax, e1y = positions[b * 3 + 1] - ay, e1z = positions[b * 3 + 2] - az;
    float e2x = positions[c * 3] - ax, e2y = positions[c * 3 + 1] - ay, e2z = positions[c * 3 + 2] - az;
    float fnx = e1y * e2z - e1z * e2y, fny = e1z * e2x - e1x * e2z, fnz = e1x * e2y - e1y * e2x;
    float au = uvs[a * 2], av = uvs[a * 2 + 1];
    float s1 = uvs[b * 2] - au, t1 = uvs[b * 2 + 1] - av;
    float s2 = uvs[c * 2] - au, t2 = uvs[c * 2 + 1] - av;
    float d = s1 * t2 - s2 * t1;
    float r = std::abs(d) < 1e-12f ? 0.f : 1.f / d;
    float tx = (e1x * t2 - e2x * t1) * r, ty = (e1y * t2 - e2y * t1) * r, tz = (e1z * t2 - e2z * t1) * r;
    float bx = (e2x * s1 - e1x * s2) * r, by = (e2y * s1 - e1y * s2) * r, bz = (e2z * s1 - e1z * s2) * r;
    for (int k = 0; k < 3; k++) {
      uint32_t i = tri[k];
      tan[i * 3] += tx; tan[i * 3 + 1] += ty; tan[i * 3 + 2] += tz;
      bit[i * 3] += bx; bit[i * 3 + 1] += by; bit[i * 3 + 2] += bz;
      nrm[i * 3] += fnx; nrm[i * 3 + 1] += fny; nrm[i * 3 + 2] += fnz;
    }
  }

  std::vector<float> out(nVerts * 4);
  for (size_t i = 0; i < nVerts; i++) {
    float nx = nrm[i * 3], ny = nrm[i * 3 + 1], nz = nrm[i * 3 + 2];
    float nl = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (nl > 1e-20f) { nx /= nl; ny /= nl; nz /= nl; }
    float tx = tan[i * 3], ty = tan[i * 3 + 1], tz = tan[i * 3 + 2];
    float nd = nx * tx + ny * ty + nz * tz;
    tx -= nx * nd; ty -= ny * nd; tz -= nz * nd; // Gram-Schmidt against the unit normal
    float tl = std::sqrt(tx * tx + ty * ty + tz * tz);
    if (tl < 1e-8f) {
      // Under-constrained UVs (seam/degenerate triangle): arbitrary tangent perpendicular to N.
      int axis = (std::abs(nx) <= std::abs(ny) && std::abs(nx) <= std::abs(nz)) ? 0
                 : (std::abs(ny) <= std::abs(nz) ? 1 : 2);
      float ux = axis == 0 ? 1.f : 0.f, uy = axis == 1 ? 1.f : 0.f, uz = axis == 2 ? 1.f : 0.f;
      tx = ny * uz - nz * uy; ty = nz * ux - nx * uz; tz = nx * uy - ny * ux;
      tl = std::sqrt(tx * tx + ty * ty + tz * tz);
    }
    float inv = tl > 0.f ? 1.f / tl : 0.f;
    tx *= inv; ty *= inv; tz *= inv;
    float cx = ny * tz - nz * ty, cy = nz * tx - nx * tz, cz = nx * ty - ny * tx;
    float w = (cx * bit[i * 3] + cy * bit[i * 3 + 1] + cz * bit[i * 3 + 2]) < 0.f ? -1.f : 1.f;
    out[i * 4] = tx; out[i * 4 + 1] = ty; out[i * 4 + 2] = tz; out[i * 4 + 3] = w;
  }
  return out;
}

std::unique_ptr<UnwrapUVsOutput>
unwrapUVs(const std::vector<uint32_t> &targetMeshIndices,
          const std::vector<float> &targetMeshPositions, int nCones,
          bool flattenToDisk, bool mapToSphere, bool alignUVs, float upX,
          float upY, float upZ, float fallbackX, float fallbackY,
          float fallbackZ, int quarterTurns) {
  std::string error;
  Model model;
  std::vector<bool> isSurfaceClosed;
  bool success = loadModel(targetMeshPositions, targetMeshIndices, model, error,
                           isSurfaceClosed);
  if (!success) {
    return std::make_unique<UnwrapUVsOutput>(error);
  }

  // set nCones to 8 for closed surfaces
  std::vector<int> nConesPerMesh(model.size(), nCones);
  for (int i = 0; i < model.size(); i += 1) {
    if (isSurfaceClosed[i] && !mapToSphere && nCones < 3) {
      nConesPerMesh[i] = 8;
    }
  }

  UvAlignment align{Vector(upX, upY, upZ),
                    Vector(fallbackX, fallbackY, fallbackZ), quarterTurns};
  align.up.normalize();
  align.fallback.normalize();
  if (!flatten(model, isSurfaceClosed, nConesPerMesh, flattenToDisk,
               mapToSphere, error, alignUVs ? &align : nullptr)) {
    return std::make_unique<UnwrapUVsOutput>(error);
  }

  Vector modelMinBounds, modelMaxBounds;
  std::vector<Vector> originalUvIslandCenters, newUvIslandCenters;
  std::vector<uint8_t> isUvIslandFlipped;
  std::vector<Vector> outPositions;
  std::vector<Vector> outUvs;
  std::vector<int> outIndices;
  std::vector<uint8_t> isSurfaceMappedToSphere(model.size(), 0);
  MeshIO::packAndGetBuffers(model, isSurfaceMappedToSphere, true, 1.,
                            outPositions, outUvs, outIndices,
                            originalUvIslandCenters, newUvIslandCenters,
                            isUvIslandFlipped, modelMinBounds, modelMaxBounds,
                            !alignUVs);

  std::vector<float> uvs;
  uvs.reserve(outUvs.size() * 2);
  for (const auto &uv : outUvs) {
    uvs.push_back(uv.x);
    uvs.push_back(uv.y);
  }

  std::vector<float> verts;
  verts.reserve(outPositions.size() * 3);
  for (const auto &pos : outPositions) {
    verts.push_back(pos.x);
    verts.push_back(pos.y);
    verts.push_back(pos.z);
  }

  std::vector<uint32_t> indices(outIndices.begin(), outIndices.end());

  auto output = std::make_unique<UnwrapUVsOutput>(
      uvs, verts, indices, std::move(model), std::move(originalUvIslandCenters),
      std::move(newUvIslandCenters), std::move(isUvIslandFlipped),
      modelMinBounds, modelMaxBounds, std::move(isSurfaceMappedToSphere));
  output->tangents = computeTangents(output->verts, output->uvs, output->indices);
  return output;
}

template <typename T>
emscripten::class_<std::vector<T>> register_vector_custom(const char *name) {
  typedef std::vector<T> VecType;

  // void (VecType::*push_back)(const T&) = &VecType::push_back;
  void (VecType::*resize)(const size_t, const T &) = &VecType::resize;
  // void (VecType::*reserve)(const size_t) = &VecType::reserve;
  size_t (VecType::*size)() const = &VecType::size;
  return emscripten::class_<std::vector<T>>(name)
      .template constructor<>()
      // .function("push_back", push_back)
      .function("resize", resize)
      .function("size", size)
      // .function("reserve", reserve)
      // .function("get", &internal::VectorAccess<VecType>::get)
      // .function("set", &internal::VectorAccess<VecType>::set)
      .function("data", &getVecDataPtr<T>, emscripten::allow_raw_pointers());
}

EMSCRIPTEN_BINDINGS(my_module) {
  emscripten::function("unwrapUVs", &unwrapUVs);

  emscripten::class_<UnwrapUVsOutput>("UnwrapUVsOutput")
      .property("uvs", &UnwrapUVsOutput::uvs)
      .property("verts", &UnwrapUVsOutput::verts)
      .property("indices", &UnwrapUVsOutput::indices)
      .property("tangents", &UnwrapUVsOutput::tangents)
      .property("error", &UnwrapUVsOutput::error)
      .function("getDistortionSvg", &UnwrapUVsOutput::getDistortionSvg);

  register_vector_custom<float>("vector<float>");
  register_vector_custom<uint32_t>("vector<uint32_t>");
}
