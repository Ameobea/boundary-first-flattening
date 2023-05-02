#include "bff/mesh/PolygonSoup.h"
#include <queue>
#include <array>
#include <unordered_set>

namespace bff {

void VertexAdjacencyMaps::construct(int nV, const std::vector<int>& indices)
{
	// collect vertex-face and vertex-vertex pairs
	faceCount.clear();
	faceCount.resize(nV, std::make_pair(0, 0));
	std::vector<std::pair<int, int>> vertexPairs;
	for (int I = 0; I < (int)indices.size(); I += 3) {
		for (int J = 0; J < 3; J++) {
			int K = (J + 1) % 3;
			int i = indices[I + J];
			int j = indices[I + K];

			faceCount[i].first++;
			faceCount[i].second = I;
			if (i < j) std::swap(i, j);
			vertexPairs.emplace_back(std::make_pair(i, j));
		}
	}

	// construct map
	data.clear();
	offsets.clear();
	offsets.emplace_back(0);
	insert(nV, vertexPairs);
}

void VertexAdjacencyMaps::insert(int nV, std::vector<std::pair<int, int>>& vertexPairs)
{
	// sort edges and remove duplicates
	std::sort(vertexPairs.begin(), vertexPairs.end());
	std::vector<std::pair<int, int>>::iterator end = std::unique(vertexPairs.begin(), vertexPairs.end());
	vertexPairs.resize(std::distance(vertexPairs.begin(), end));

	// update map
	if (vertexPairs.size() > 0) {
		int cV = (int)offsets.size() - 1;
		int j = 0;

		for (int i = cV; i < nV; i++) {
			while (i == vertexPairs[j].first) {
				data.emplace_back(vertexPairs[j].second);
				j++;
			}

			offsets.emplace_back(offsets[cV] + j);
		}
	}
}

int VertexAdjacencyMaps::getEdgeIndex(int vi, int vj) const
{
	if (vi < vj) std::swap(vi, vj);

	int k = 0;
	for (int l = offsets[vi]; l < offsets[vi + 1]; l++) {
		if (data[l] == vj) break;
		k++;
	}

	return offsets[vi] + k;
}

int VertexAdjacencyMaps::getEdgeCount() const
{
	return offsets[offsets.size() - 1];
}

void EdgeFaceAdjacencyMap::construct(const VertexAdjacencyMaps& vertexAdjacency,
									 const std::vector<int>& indices)
{
	// collect edge face pairs
	int nE = vertexAdjacency.getEdgeCount();
	std::vector<std::pair<int, int>> edgeFacePairs;
	edgeFacePairs.reserve(nE*2);

	for (int I = 0; I < (int)indices.size(); I += 3) {
		for (int J = 0; J < 3; J++) {
			int K = (J + 1) % 3;
			int i = indices[I + J];
			int j = indices[I + K];

			int eIndex = vertexAdjacency.getEdgeIndex(i, j);
			edgeFacePairs.emplace_back(std::make_pair(eIndex, I));
		}
	}

	// construct map
	data.clear();
	offsets.clear();
	offsets.emplace_back(0);
	isAdjacentFace.clear();
	insert(vertexAdjacency, edgeFacePairs);
}

void EdgeFaceAdjacencyMap::insert(const VertexAdjacencyMaps& vertexAdjacency,
                                  std::vector<std::pair<int, int>>& edgeFacePairs)
{
	// sort edge face pairs
	std::sort(edgeFacePairs.begin(), edgeFacePairs.end());

	// update map
	if (edgeFacePairs.size() > 0) {
		int nE = vertexAdjacency.getEdgeCount();
		int cE = (int)offsets.size() - 1;
		int j = 0;

		for (int i = cE; i < nE; i++) {
			while (i == edgeFacePairs[j].first) {
				data.emplace_back(edgeFacePairs[j].second);
				isAdjacentFace.emplace_back(1);
				j++;
			}

			offsets.emplace_back(offsets[cE] + j);
		}
	}
}

int EdgeFaceAdjacencyMap::getAdjacentFaceCount(int e) const
{
	return offsets[e + 1] - offsets[e];
}

std::pair<int, int> EdgeFaceAdjacencyMap::getAdjacentFaceIndex(int e, int f) const
{
	int offset = offsets[e] + f;
	return std::make_pair(data[offset], isAdjacentFace[offset]);
}

int PolygonSoup::identifyNonManifoldVertices(std::vector<uint8_t>& isNonManifoldVertex) const
{
	int nNonManifoldVertices = 0;
	for (int v = 0; v < (int)isNonManifoldVertex.size(); v++) {
		// get adjacent face count
		const std::pair<int, int>& adjacentFaceData = vertexAdjacency.faceCount[v];
		int nAdjacentFaces = adjacentFaceData.first;

		if (nAdjacentFaces > 0) {
			// traverse adjacent faces to compute vertex degree
			int degree = 0;
			int f = adjacentFaceData.second;

			// visit adjacent faces
			std::unordered_set<int> seenFaces;
			std::queue<int> queue;
			queue.push(f);
			seenFaces.emplace(f);

			while (!queue.empty()) {
				int f = queue.front();
				queue.pop();
				degree++;

				for (int I = 0; I < 3; I++) {
					int J = (I + 1) % 3;
					int i = indices[f + I];
					int j = indices[f + J];

					// enqueue face on other side of interior, manifold edge
					int eIndex = vertexAdjacency.getEdgeIndex(i, j);
					if (edgeFaceAdjacency.getAdjacentFaceCount(eIndex) == 2 && (v == i || v == j)) {
						int g = edgeFaceAdjacency.getAdjacentFaceIndex(eIndex, 0).first;
						if (g == f) g = edgeFaceAdjacency.getAdjacentFaceIndex(eIndex, 1).first;

						if (seenFaces.find(g) == seenFaces.end()) {
							queue.push(g);
							seenFaces.emplace(g);
						}
					}
				}
			}

			if (degree != nAdjacentFaces) {
				isNonManifoldVertex[v] = 1;
				nNonManifoldVertices++;
			}
		}
	}

	return nNonManifoldVertices;
}

void PolygonSoup::collectAdjacentFaces(const std::vector<uint8_t>& isNonManifoldVertex,
									   std::unordered_map<int, std::vector<int>>& vertexToFacesMap) const
{
	for (int I = 0; I < (int)indices.size(); I += 3) {
		for (int J = 0; J < 3; J++) {
			int i = indices[I + J];

			if (isNonManifoldVertex[i] == 1) {
				vertexToFacesMap[i].emplace_back(I);
			}
		}
	}
}

void PolygonSoup::splitNonManifoldVertices()
{
	// identify non-manifold vertices
	std::vector<uint8_t> isNonManifoldVertex(positions.size(), 0);
	int nNonManifoldVertices = identifyNonManifoldVertices(isNonManifoldVertex);

	if (nNonManifoldVertices > 0) {
		// collect adjacent faces to each non-manifold vertex
		std::unordered_map<int, std::vector<int>> vertexToFacesMap;
		collectAdjacentFaces(isNonManifoldVertex, vertexToFacesMap);

		// iterate over non-manifold vertices and split them
		std::unordered_map<int, std::array<int, 3>> updatedFaceIndices;
		for (auto it = vertexToFacesMap.begin(); it != vertexToFacesMap.end(); it++) {
			int v = it->first;
			const std::vector<int>& adjacentFaces = it->second;

			// assign components to adjacent faces
			int nFacesSeen = 0;
			int nComponents = 0;
			int start = 0;
			int faceCount = (int)adjacentFaces.size();
			std::unordered_set<int> seenFaces;
			std::queue<int> queue;

			while (nFacesSeen != faceCount) {
				// put an unvisited face on the queue
				for (int i = start; i < faceCount; i++) {
					int f = adjacentFaces[i];

					if (seenFaces.find(f) == seenFaces.end()) {
						queue.push(f);
						seenFaces.emplace(f);
						start = i + 1;
						break;
					}
				}

				// isolate a component
				std::vector<int> componentFaces;
				while (!queue.empty()) {
					int f = queue.front();
					queue.pop();
					componentFaces.emplace_back(f);
					nFacesSeen++;

					for (int I = 0; I < 3; I++) {
						int J = (I + 1) % 3;
						int i = indices[f + I];
						int j = indices[f + J];

						// enqueue face on other side of interior, manifold edge
						int eIndex = vertexAdjacency.getEdgeIndex(i, j);
						if (edgeFaceAdjacency.getAdjacentFaceCount(eIndex) == 2 && (v == i || v == j)) {
							int g = edgeFaceAdjacency.getAdjacentFaceIndex(eIndex, 0).first;
							if (g == f) g = edgeFaceAdjacency.getAdjacentFaceIndex(eIndex, 1).first;

							if (seenFaces.find(g) == seenFaces.end()) {
								queue.push(g);
								seenFaces.emplace(g);
							}
						}
					}
				}

				nComponents++;
				if (nComponents > 1) {
					// split vertex
					int u = (int)positions.size();
					Vector p = positions[v];
					positions.emplace_back(p);

					// record updated split vertex index
					for (int k = 0; k < (int)componentFaces.size(); k++) {
						int f = componentFaces[k];
						if (updatedFaceIndices.find(f) == updatedFaceIndices.end()) {
							updatedFaceIndices[f][0] = indices[f + 0];
							updatedFaceIndices[f][1] = indices[f + 1];
							updatedFaceIndices[f][2] = indices[f + 2];
						}

						if (indices[f + 0] == v) updatedFaceIndices[f][0] = u;
						if (indices[f + 1] == v) updatedFaceIndices[f][1] = u;
						if (indices[f + 2] == v) updatedFaceIndices[f][2] = u;
					}
				}
			}
		}

		// update indices
		std::vector<std::pair<int, int>> newEdgeFacePairs;
		for (auto it = updatedFaceIndices.begin(); it != updatedFaceIndices.end(); it++) {
			int f = it->first;
			const std::array<int, 3>& updatedIndices = it->second;

			for (int I = 0; I < 3; I++) {
				int J = (I + 1) % 3;
				int ii = updatedIndices[I];
				int jj = updatedIndices[J];

				indices[f + I] = ii;
				indices[f + J] = jj;
			}
		}

		// update adjacency maps
		// TODO: reconstruct locally
		vertexAdjacency.construct(positions.size(), indices);
		edgeFaceAdjacency.construct(vertexAdjacency, indices);
	}
}

int PolygonSoup::separateFacesIntoComponents()
{
	int nComponents = 0;
	int nIndices = (int)indices.size();
	faceComponent.resize(nIndices);
	std::vector<uint8_t> seenFace(nIndices, 0);

	for (int I = 0; I < nIndices; I += 3) {
		// continue if face has already been seen
		if (seenFace[I] == 1) continue;

		// collect all faces in a single component and mark them as seen
		seenFace[I] = 1;
		faceComponent[I] = nComponents;
		std::queue<int> q;
		q.push(I);

		while (!q.empty()) {
			int f = q.front();
			q.pop();

			// loop over all edges in the face
			for (int J = 0; J < 3; J++) {
				int K = (J + 1) % 3;
				int i = indices[f + J];
				int j = indices[f + K];

				int eIndex = vertexAdjacency.getEdgeIndex(i, j);

				// check if the edge is not on the boundary
				int nAdjacentFaces = edgeFaceAdjacency.getAdjacentFaceCount(eIndex);
				if (nAdjacentFaces > 1) {
					for (int L = 0; L < nAdjacentFaces; L++) {
						std::pair<int, int> faceIndex = edgeFaceAdjacency.getAdjacentFaceIndex(eIndex, L);
						int g = faceIndex.first;
						bool isAdjacentFace = faceIndex.second == 1;

						if (g != f && isAdjacentFace && !seenFace[g]) {
							seenFace[g] = 1;
							faceComponent[g] = nComponents;
							q.push(g);
						}
					}
				}
			}
		}

		nComponents++;
	}

	return nComponents;
}

} // namespace bff