#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <algorithm>
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

bool flattenMesh(Mesh &mesh, bool isSurfaceClosed, int nCones,
                 bool flattenToDisk, bool mapToSphere, std::string &error) {
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

      mesh.projectUvsToPcaAxis();
    }
  }

  return true;
}

// Also copied from `CommandLine.cpp`
bool flatten(Model &model, const std::vector<bool> &isSurfaceClosed,
             const std::vector<int> &nCones, bool flattenToDisk,
             bool mapToSphere, std::string &error) {
  int nMeshes = model.size();
  for (int i = 0; i < nMeshes; i++) {
    Mesh &mesh = model[i];
    bool ok = flattenMesh(mesh, isSurfaceClosed[i], nCones[i], flattenToDisk,
                          mapToSphere, error);
    if (!ok) {
      if (error.empty()) {
        error = "Failed to flatten mesh " + std::to_string(i) + ".";
      }
      return false;
    }
  }

  return true;
}

std::unique_ptr<UnwrapUVsOutput>
unwrapUVs(const std::vector<uint32_t> &targetMeshIndices,
          const std::vector<float> &targetMeshPositions, int nCones,
          bool flattenToDisk, bool mapToSphere) {
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

  if (!flatten(model, isSurfaceClosed, nConesPerMesh, flattenToDisk,
               mapToSphere, error)) {
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
                            isUvIslandFlipped, modelMinBounds, modelMaxBounds);

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

  return std::make_unique<UnwrapUVsOutput>(
      uvs, verts, indices, std::move(model), std::move(originalUvIslandCenters),
      std::move(newUvIslandCenters), std::move(isUvIslandFlipped),
      modelMinBounds, modelMaxBounds, std::move(isSurfaceMappedToSphere));
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
      .property("error", &UnwrapUVsOutput::error)
      .function("getDistortionSvg", &UnwrapUVsOutput::getDistortionSvg);

  register_vector_custom<float>("vector<float>");
  register_vector_custom<uint32_t>("vector<uint32_t>");
}
