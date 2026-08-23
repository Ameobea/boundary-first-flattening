#include "bff/project/BinPacking.h"
#include "Rect.h"
#include "SkylineBinPack.h"
#include <algorithm>
#include <limits>

namespace bff {

using namespace rbp;

// Translation-only skyline bottom-left packer; rbp::SkylineBinPack always tries the 90° flip too.
struct FixedOrientationSkyline {
	struct Node { int x, y, width; };
	int size;
	std::vector<Node> nodes;

	explicit FixedOrientationSkyline(int size_): size(size_), nodes{{0, 0, size_}} {}

	bool fits(size_t i, int width, int height, int& y) const {
		if (nodes[i].x + width > size) return false;
		y = nodes[i].y;
		for (int left = width; left > 0; i++) {
			y = std::max(y, nodes[i].y);
			if (y + height > size) return false;
			left -= nodes[i].width;
		}
		return true;
	}

	Rect insert(int width, int height) {
		Rect best{0, 0, 0, 0};
		if (width <= 0 || height <= 0) return best;

		int bestTop = std::numeric_limits<int>::max(), bestWidth = bestTop;
		size_t bestIndex = 0;
		for (size_t i = 0; i < nodes.size(); i++) {
			int y;
			if (!fits(i, width, height, y)) continue;
			if (y + height < bestTop || (y + height == bestTop && nodes[i].width < bestWidth)) {
				bestTop = y + height;
				bestWidth = nodes[i].width;
				bestIndex = i;
				best = Rect{nodes[i].x, y, width, height};
			}
		}
		if (best.height == 0) return best;

		nodes.insert(nodes.begin() + bestIndex, Node{best.x, best.y + best.height, best.width});
		for (size_t i = bestIndex + 1; i < nodes.size();) {
			int shrink = nodes[i - 1].x + nodes[i - 1].width - nodes[i].x;
			if (shrink <= 0) break;
			nodes[i].x += shrink;
			nodes[i].width -= shrink;
			if (nodes[i].width > 0) break;
			nodes.erase(nodes.begin() + i);
		}
		for (size_t i = 0; i + 1 < nodes.size();) {
			if (nodes[i].y == nodes[i + 1].y) {
				nodes[i].width += nodes[i + 1].width;
				nodes.erase(nodes.begin() + i + 1);
			} else {
				i++;
			}
		}
		return best;
	}
};

bool attemptPacking(int boxLength, double unitsPerInt,
					const std::vector<std::pair<RectSize, int>>& rectangleSizes,
					std::vector<Vector>& newUvIslandCenters,
					std::vector<uint8_t>& isUvIslandFlipped,
					Vector& modelMinBounds, Vector& modelMaxBounds,
					bool rotateIslands)
{
	// initialize packer
	int n = (int)rectangleSizes.size();
	modelMinBounds = Vector(std::numeric_limits<double>::max(),
							std::numeric_limits<double>::max());
	modelMaxBounds = Vector(std::numeric_limits<double>::lowest(),
							std::numeric_limits<double>::lowest());
	SkylineBinPack packer(boxLength, boxLength, false);
	FixedOrientationSkyline fixedPacker(boxLength);

	for (int i = 0; i < n; i++) {
		RectSize rectSize = rectangleSizes[i].first;
		int index = rectangleSizes[i].second;

		Rect rect = rotateIslands
			? packer.Insert(rectSize.width, rectSize.height,
							SkylineBinPack::LevelChoiceHeuristic::LevelBottomLeft)
			: fixedPacker.insert(rectSize.width, rectSize.height);

		// check for failure
		if (rect.width == 0 || rect.height == 0) {
			if (rectSize.width != 0 && rectSize.height != 0) {
				return false;
			}
		}

		// check if flipped
		isUvIslandFlipped[index] = rect.width == rectSize.width ? 0 : 1;

		// compute new centers
		newUvIslandCenters[index] = Vector(rect.x + rect.width/2.0, rect.y + rect.height/2.0);
		newUvIslandCenters[index] *= unitsPerInt;
		modelMinBounds.x = std::min((double)rect.x, modelMinBounds.x);
		modelMinBounds.y = std::min((double)rect.y, modelMinBounds.y);
		modelMaxBounds.x = std::max((double)(rect.x + rect.width), modelMaxBounds.x);
		modelMaxBounds.y = std::max((double)(rect.y + rect.height), modelMaxBounds.y);
	}

	return true;
}

void BinPacking::pack(const Model& model, double scaling,
					  const std::vector<uint8_t>& isSurfaceMappedToSphere,
					  std::vector<Vector>& originalUvIslandCenters,
					  std::vector<Vector>& newUvIslandCenters,
					  std::vector<uint8_t>& isUvIslandFlipped,
					  Vector& modelMinBounds, Vector& modelMaxBounds,
					  bool rotateIslands)
{
	// compute bounding boxes
	int n = model.size();
	double totalArea = 0.0;
	std::vector<Vector> minBounds(n, Vector(std::numeric_limits<double>::max(),
											std::numeric_limits<double>::max()));
	std::vector<Vector> maxBounds(n, Vector(std::numeric_limits<double>::lowest(),
											std::numeric_limits<double>::lowest()));

	for (int i = 0; i < n; i++) {
		// compute island radius
		double radius = 1e-8;
		if (isSurfaceMappedToSphere[i] == 1) {
			for (WedgeCIter w = model[i].wedges().begin(); w != model[i].wedges().end(); w++) {
				radius = std::max(w->uv.norm(), radius);
			}

		} else {
			radius = model[i].radius;
		}

		// compute the ratio of the surface areas of the mesh and flattened mesh
		double lengthRatio = std::sqrt(model[i].areaRatio())*scaling;

		// scale UVs by radius and compute bounds
		for (WedgeCIter w = model[i].wedges().begin(); w != model[i].wedges().end(); w++) {
			Vector uv = w->uv;
			if (isSurfaceMappedToSphere[i] == 1) {
				uv /= radius;
				uv.x = 0.5 + atan2(uv.z, uv.x)/(2*M_PI);
				uv.y = 0.5 - asin(uv.y)/M_PI;

			} else {
				uv *= radius*lengthRatio;
			}

			minBounds[i].x = std::min(uv.x, minBounds[i].x);
			minBounds[i].y = std::min(uv.y, minBounds[i].y);
			maxBounds[i].x = std::max(uv.x, maxBounds[i].x);
			maxBounds[i].y = std::max(uv.y, maxBounds[i].y);
		}

		totalArea += (maxBounds[i].x - minBounds[i].x)*(maxBounds[i].y - minBounds[i].y);
	}

	// quantize boxes
	originalUvIslandCenters.resize(n);
	std::vector<std::pair<RectSize, int>> rectangleSizes(n);
	int minBoxLength = 10000;
	int maxBoxLength = 10000;
	double unitsPerInt = std::sqrt(totalArea)/(double)maxBoxLength;

	for (int i = 0; i < n; i++) {
		int minX = static_cast<int>(std::floor(minBounds[i].x/unitsPerInt));
		int minY = static_cast<int>(std::floor(minBounds[i].y/unitsPerInt));
		int maxX = static_cast<int>(std::ceil(maxBounds[i].x/unitsPerInt));
		int maxY = static_cast<int>(std::ceil(maxBounds[i].y/unitsPerInt));

		int width = maxX - minX;
		int height = maxY - minY;
		rectangleSizes[i] = std::make_pair(RectSize{width, height}, i);
		originalUvIslandCenters[i].x = (minX + maxX)/2.0;
		originalUvIslandCenters[i].y = (minY + maxY)/2.0;
		originalUvIslandCenters[i] *= unitsPerInt;
	}

	if (n > 1) {
		// sort rectangle sizes by the shorter side first, followed by a comparison of the longer side
		std::sort(rectangleSizes.begin(), rectangleSizes.end(),
			[](const std::pair<RectSize, int>& a, const std::pair<RectSize, int>& b) {
			return a.first.width == b.first.width ?
				   a.first.height > b.first.height :
				   a.first.width > b.first.width;
		});
	}

	// pack islands
	newUvIslandCenters.resize(n);
	isUvIslandFlipped.resize(n);
	int iter = 0;

	do {
		if (attemptPacking(maxBoxLength, unitsPerInt, rectangleSizes,
						   newUvIslandCenters, isUvIslandFlipped,
						   modelMinBounds, modelMaxBounds, rotateIslands)) {
			break;
		}

		minBoxLength = maxBoxLength;
		maxBoxLength = static_cast<int>(std::ceil(minBoxLength*1.1));
		iter++;
	} while (iter < 50);

	if (iter < 50 && n > 1) {
		// binary search on box length
		minBoxLength = 5000;
		maxBoxLength += 1;

		while (minBoxLength <= maxBoxLength) {
			int boxLength = (minBoxLength + maxBoxLength)/2;
			if (boxLength == minBoxLength) break;

			if (attemptPacking(boxLength, unitsPerInt, rectangleSizes,
							   newUvIslandCenters, isUvIslandFlipped,
							   modelMinBounds, modelMaxBounds, rotateIslands)) {
				maxBoxLength = boxLength;

			} else {
				minBoxLength = boxLength;
			}
		}

		attemptPacking(maxBoxLength, unitsPerInt, rectangleSizes,
					   newUvIslandCenters, isUvIslandFlipped,
					   modelMinBounds, modelMaxBounds, rotateIslands);
	}

	modelMinBounds *= unitsPerInt;
	modelMaxBounds *= unitsPerInt;
}

} // namespace bff
