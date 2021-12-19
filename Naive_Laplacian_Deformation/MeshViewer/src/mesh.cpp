#include "mesh.h"
#include <iostream>
#include <igl/read_triangle_mesh.h>
//#include <Eigen/Sparse>


HEdge::HEdge(bool b) {
	mBoundary = b;

	mTwin = nullptr;
	mPrev = nullptr;
	mNext = nullptr;

	mStart = nullptr;
	mFace = nullptr;

	mFlag = false;
	mValid = true;
}

HEdge* HEdge::twin() const {
	return mTwin;
}

HEdge* HEdge::setTwin(HEdge* e) {
	mTwin = e;
	return mTwin;
}

HEdge* HEdge::prev() const {
	return mPrev;
}

HEdge* HEdge::setPrev(HEdge* e) {
	mPrev = e;
	return mPrev;
}

HEdge* HEdge::next() const {
	return mNext;
}

HEdge* HEdge::setNext(HEdge* e) {
	mNext = e;
	return mNext;
}

Vertex* HEdge::start() const {
	return mStart;
}

Vertex* HEdge::setStart(Vertex* v) {
	mStart = v;
	return mStart;
}

Vertex* HEdge::end() const {
	return mNext->start();
}

Face* HEdge::leftFace() const {
	return mFace;
}

Face* HEdge::setFace(Face* f) {
	mFace = f;
	return mFace;
}

bool HEdge::flag() const {
	return mFlag;
}

bool HEdge::setFlag(bool b) {
	mFlag = b;
	return mFlag;
}

bool HEdge::isBoundary() const {
	return mBoundary;
}

bool HEdge::isValid() const {
	return mValid;
}

bool HEdge::setValid(bool b) {
	mValid = b;
	return mValid;
}

OneRingHEdge::OneRingHEdge(const Vertex* v) {
	if (v == nullptr) {
		mStart = nullptr;
		mNext = nullptr;
	} else {
		mStart = v->halfEdge();
		mNext = v->halfEdge();
	}
}
//this next HEdge start from mStart = mNext = ret
HEdge* OneRingHEdge::nextHEdge() {
	HEdge* ret = mNext;
	if (mNext != nullptr && mNext->prev()->twin() != mStart) {
		mNext = mNext->prev()->twin();
	} else {
		mNext = nullptr;
	}
	return ret;
}

HEdge* OneRingHEdge::getmStart() const{
	return mStart;
}

HEdge* OneRingHEdge::getmNext() const{
	return mNext;
}

OneRingVertex::OneRingVertex(const Vertex* v): ring(v) {
}

Vertex* OneRingVertex::nextVertex() {
	HEdge* he = ring.nextHEdge();
	return he != nullptr ? he->end() : nullptr;
}

Vertex::Vertex() : mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f::Zero();
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(const Eigen::Vector3f& v): mPosition(v), mHEdge(nullptr), mFlag(0) {
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(float x, float y, float z): mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f(x, y, z);
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}


const Eigen::Vector3f& Vertex::position() const {
	return mPosition;
}

const Eigen::Vector3f& Vertex::setPosition(const Eigen::Vector3f& p) {
	mPosition = p;
	return mPosition;
}

const Eigen::Vector3f& Vertex::normal() const {
	return mNormal;
}

const Eigen::Vector3f& Vertex::setNormal(const Eigen::Vector3f& n) {
	mNormal = n;
	return mNormal;
}

const Eigen::Vector3f& Vertex::color() const {
	return mColor;
}

const Eigen::Vector3f& Vertex::setColor(const Eigen::Vector3f& c) {
	mColor = c;
	return mColor;
}

HEdge* Vertex::halfEdge() const {
	return mHEdge;
}

HEdge* Vertex::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

int Vertex::index() const {
	return mIndex;
}

int Vertex::setIndex(int i) {
	mIndex = i;
	return mIndex;
}

int Vertex::flag() const {
	return mFlag;
}

int Vertex::setFlag(int f) {
	mFlag = f;
	return mFlag;
}

bool Vertex::isValid() const {
	return mValid;
}

bool Vertex::setValid(bool b) {
	mValid = b;
	return mValid;
}

bool Vertex::isBoundary() const {
	OneRingHEdge ring(this);
	HEdge* curr = nullptr;
	while (curr = ring.nextHEdge()) {
		if (curr->isBoundary()) {
			return true;
		}
	}
	return false;
}

int Vertex::valence() const {
	int count = 0;
	OneRingVertex ring(this);
	Vertex* curr = nullptr;
	while (curr = ring.nextVertex()) {
		++count;
	}
	return count;
}

Face::Face() : mHEdge(nullptr), mValid(true) {
}

HEdge* Face::halfEdge() const {
	return mHEdge;
}

HEdge* Face::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

bool Face::isBoundary() const {
	HEdge* curr = mHEdge;
	do {
		if (curr->twin()->isBoundary()) {
			return true;
		}
		curr = curr->next();
	} while (curr != mHEdge);
	return false;
}

bool Face::isValid() const {
	return mValid;
}

bool Face::setValid(bool b) {
	mValid = b;
	return mValid;
}

double Face::getmAera() const {
	return mAera;
}

Eigen::Vector3f Face::getmNormal() const {
	return mNormal;
}

double Face::computeFaceAera() {
	//mAera = 1;
	Vertex* v0 = mHEdge->start();
	Vertex* v1 = mHEdge->end();
	Vertex* v2 = mHEdge->prev()->start();

	Eigen::Vector3f vp0 = v0->position();
	Eigen::Vector3f vp1 = v1->position();
	Eigen::Vector3f vp2 = v2->position();

	mAera = (((vp1 - vp0).cross(vp2 - vp0)).norm()) / 2;
	return mAera;
}

Eigen::Vector3f Face::computeFaceNormal() {
	//mNormal << 0, 0, 1;
	Vertex* v0 = mHEdge->start();
	Vertex* v1 = mHEdge->end();
	Vertex* v2 = mHEdge->prev()->start();

	Eigen::Vector3f vp0 = v0->position();
	Eigen::Vector3f vp1 = v1->position();
	Eigen::Vector3f vp2 = v2->position();

	mNormal = (vp1 - vp0).cross(vp2 - vp0);
	mNormal = mNormal.normalized();
	return mNormal;
}

Mesh::Mesh() {
	mVertexPosFlag = true;
	mVertexNormalFlag = true;
	mVertexColorFlag = true;
	smoothNum = 0;
	runtime = 0;
}

Mesh::~Mesh() {
	clear();
}

const std::vector< HEdge* >& Mesh::edges() const {
	return mHEdgeList;
}

const std::vector< HEdge* >& Mesh::boundaryEdges() const {
	return mBHEdgeList;
}

const std::vector< Vertex* >& Mesh::vertices() const {
	return mVertexList;
}

const std::vector< Face* >& Mesh::faces() const {
	return mFaceList;
}


bool Mesh::isVertexPosDirty() const {
	return mVertexPosFlag;
}

void Mesh::setVertexPosDirty(bool b) {
	mVertexPosFlag = b;
}

bool Mesh::isVertexNormalDirty() const {
	return mVertexNormalFlag;
}

void Mesh::setVertexNormalDirty(bool b) {
	mVertexNormalFlag = b;
}

bool Mesh::isVertexColorDirty() const {
	return mVertexColorFlag;
}

void Mesh::setVertexColorDirty(bool b) {
	mVertexColorFlag = b;
}

bool Mesh::loadMeshFile(const std::string filename) {
	// Use libigl to parse the mesh file
	bool iglFlag = igl::read_triangle_mesh(filename, mVertexMat, mFaceMat);
	if (iglFlag) {
		clear();

		// Construct the half-edge data structure.
		int numVertices = mVertexMat.rows();
		int numFaces = mFaceMat.rows();

		// Fill in the vertex list
		for (int vidx = 0; vidx < numVertices; ++vidx) {
			mVertexList.push_back(new Vertex(mVertexMat(vidx, 0),
			                                 mVertexMat(vidx, 1),
			                                 mVertexMat(vidx, 2)));
		}
		// Fill in the face list
		for (int fidx = 0; fidx < numFaces; ++fidx) {
			addFace(mFaceMat(fidx, 0), mFaceMat(fidx, 1), mFaceMat(fidx, 2));
		}

		std::vector< HEdge* > hedgeList;
		for (int i = 0; i < mBHEdgeList.size(); ++i) {
			if (mBHEdgeList[i]->start()) {
				hedgeList.push_back(mBHEdgeList[i]);
			}
			// TODO
		}
		mBHEdgeList = hedgeList;

		for (int i = 0; i < mVertexList.size(); ++i) {
			mVertexList[i]->adjHEdges.clear();
			mVertexList[i]->setIndex(i);
			mVertexList[i]->setFlag(0);
		}
	} else {
		std::cout << __FUNCTION__ << ": mesh file loading failed!\n";
	}
	return iglFlag;
}

static void _setPrevNext(HEdge* e1, HEdge* e2) {
	e1->setNext(e2);
	e2->setPrev(e1);
}

static void _setTwin(HEdge* e1, HEdge* e2) {
	e1->setTwin(e2);
	e2->setTwin(e1);
}

static void _setFace(Face* f, HEdge* e) {
	f->setHalfEdge(e);
	e->setFace(f);
}

void Mesh::addFace(int v1, int v2, int v3) {
	Face* face = new Face();

	HEdge* hedge[3];
	HEdge* bhedge[3]; // Boundary half-edges
	Vertex* vert[3];

	for (int i = 0; i < 3; ++i) {
		hedge[i] = new HEdge();
		bhedge[i] = new HEdge(true);
	}
	vert[0] = mVertexList[v1];
	vert[1] = mVertexList[v2];
	vert[2] = mVertexList[v3];

	// Connect prev-next pointers
	for (int i = 0; i < 3; ++i) {
		_setPrevNext(hedge[i], hedge[(i + 1) % 3]);
		_setPrevNext(bhedge[i], bhedge[(i + 1) % 3]);
	}

	// Connect twin pointers
	_setTwin(hedge[0], bhedge[0]);
	_setTwin(hedge[1], bhedge[2]);
	_setTwin(hedge[2], bhedge[1]);

	// Connect start pointers for bhedge
	bhedge[0]->setStart(vert[1]);
	bhedge[1]->setStart(vert[0]);
	bhedge[2]->setStart(vert[2]);
	for (int i = 0; i < 3; ++i) {
		hedge[i]->setStart(vert[i]);
	}

	// Connect start pointers
	// Connect face-hedge pointers
	for (int i = 0; i < 3; ++i) {
		vert[i]->setHalfEdge(hedge[i]);
		vert[i]->adjHEdges.push_back(hedge[i]);
		_setFace(face, hedge[i]);
	}
	vert[0]->adjHEdges.push_back(bhedge[1]);
	vert[1]->adjHEdges.push_back(bhedge[0]);
	vert[2]->adjHEdges.push_back(bhedge[2]);

	// Merge boundary if needed
	for (int i = 0; i < 3; ++i) {
		Vertex* start = bhedge[i]->start();
		Vertex* end = bhedge[i]->end();

		for (int j = 0; j < end->adjHEdges.size(); ++j) {
			HEdge* curr = end->adjHEdges[j];
			if (curr->isBoundary() && curr->end() == start) {
				_setPrevNext(bhedge[i]->prev(), curr->next());
				_setPrevNext(curr->prev(), bhedge[i]->next());
				_setTwin(bhedge[i]->twin(), curr->twin());
				bhedge[i]->setStart(nullptr); // Mark as unused
				curr->setStart(nullptr); // Mark as unused
				break;
			}
		}
	}

	// Finally add hedges and faces to list
	for (int i = 0; i < 3; ++i) {
		mHEdgeList.push_back(hedge[i]);
		mBHEdgeList.push_back(bhedge[i]);
	}
	mFaceList.push_back(face);
}

Eigen::Vector3f Mesh::initBboxMin() const {
	return (mVertexMat.colwise().minCoeff()).transpose();
}

Eigen::Vector3f Mesh::initBboxMax() const {
	return (mVertexMat.colwise().maxCoeff()).transpose();
}

void Mesh::groupingVertexFlags() {
	// Init to 255
	for (Vertex* vert : mVertexList) {
		if (vert->flag() != 0) {
			vert->setFlag(255);
		}
	}
	// Group handles
	int id = 0;
	std::vector< Vertex* > tmpList;
	for (Vertex* vert : mVertexList) {
		if (vert->flag() == 255) {
			++id;
			vert->setFlag(id);

			// Do search
			tmpList.push_back(vert);
			while (!tmpList.empty()) {
				Vertex* v = tmpList.back();
				tmpList.pop_back();

				OneRingVertex orv = OneRingVertex(v);
				while (Vertex* v2 = orv.nextVertex()) {
					if (v2->flag() == 255) {
						v2->setFlag(id);
						tmpList.push_back(v2);
					}
				}
			}
		}
	}
}

void Mesh::clear() {
	for (int i = 0; i < mHEdgeList.size(); ++i) {
		delete mHEdgeList[i];
	}
	for (int i = 0; i < mBHEdgeList.size(); ++i) {
		delete mBHEdgeList[i];
	}
	for (int i = 0; i < mVertexList.size(); ++i) {
		delete mVertexList[i];
	}
	for (int i = 0; i < mFaceList.size(); ++i) {
		delete mFaceList[i];
	}

	mHEdgeList.clear();
	mBHEdgeList.clear();
	mVertexList.clear();
	mFaceList.clear();
}

std::vector< int > Mesh::collectMeshStats() {
	int V = 0; // # of vertices
	int E = 0; // # of half-edges
	int F = 0; // # of faces
	int B = 0; // # of boundary loops
	int C = 0; // # of connected components
	int G = 0; // # of genus

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Collect mesh information as listed above.
	/**********************************************/

	V = vertices().size();
	E = edges().size() + boundaryEdges().size();
	F = faces().size();
	B = countBoundaryLoops();
	C = countConnectedComponents();
	G = 1 - (V - (edges().size()/2 + boundaryEdges().size()) + F) / 2;

	/*====== Programming Assignment 0 ======*/

	std::vector< int > stats;
	stats.push_back(V);
	stats.push_back(E);
	stats.push_back(F);
	stats.push_back(B);
	stats.push_back(C);
	stats.push_back(G);
	return stats;
}

int Mesh::countBoundaryLoops() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Helper function for Mesh::collectMeshStats()
	/**********************************************/
	/*====== Programming Assignment 0 ======*/

	return count;
}

int Mesh::countConnectedComponents() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Helper function for Mesh::collectMeshStats()
	/* Count the number of connected components of
	/* the mesh. (Hint: use a stack)
	/**********************************************/


	/*====== Programming Assignment 0 ======*/

	return count;
}

void Mesh::computeVertexNormals() {
	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Compute per-vertex normal using neighboring
	/* facet information. (Hint: remember using a 
	/* weighting scheme. Plus, do you notice any
	/* disadvantages of your weighting scheme?)
	/**********************************************/
	for (auto curr_vertex : mVertexList) {
		OneRingHEdge curr_ring(curr_vertex);
		HEdge* curr_edge = curr_ring.getmStart();
		double sum_aera = 0;
		Eigen::Vector3f sum_normal = Eigen::Vector3f::Zero();
		while (curr_edge) {
			if (Face* curr_left_face = curr_edge->leftFace()) {
				sum_aera = sum_aera + curr_left_face->computeFaceAera();
				sum_normal = sum_normal + curr_left_face->computeFaceNormal() * curr_left_face->getmAera();
			}
			curr_edge = curr_ring.nextHEdge();
		}
		Eigen::Vector3f normal = sum_normal / sum_aera;
		//std::cout << normal << std::endl;
		curr_vertex->setNormal(normal);
	}
	/*====== Programming Assignment 0 ======*/
    
	// Notify mesh shaders
	setVertexNormalDirty(true);
}

void Mesh::generateUniformLforSmooth(Eigen::SparseMatrix<float>* L, std::vector<Vertex*>& vertexList) {
	typedef Eigen::Triplet<double> T;
	std::vector<T> tripletList;
	tripletList.reserve(7 * vertexList.size());//E~3V
	for (int i = 0; i < vertexList.size(); i++) {
		tripletList.push_back(T(i, i, -1));
		OneRingVertex curr_vertex(vertexList[i]);
		int neigh_num = 0;
		std::vector<int> neigh_index;
		while (Vertex* neigh_vertex = curr_vertex.nextVertex()) {
			neigh_num = neigh_num + 1;
			neigh_index.push_back(neigh_vertex->index());
		}
		//std::cout << "neigh_num" << neigh_num << std::endl;
		for (int j = 0; j < neigh_num; j++) {
			tripletList.push_back(T(i, neigh_index[j], 1.0 / neigh_num));
		}
	}
	L->resize(mVertexList.size(), mVertexList.size());
	L->setFromTriplets(tripletList.begin(), tripletList.end());
}

void Mesh::generateCotangtLforSmooth(Eigen::SparseMatrix<float>* L, std::vector<Vertex*>& vertexList) {
	typedef Eigen::Triplet<double> T;
	std::vector<T> tripletList;
	tripletList.reserve(7 * vertexList.size());//E~3V
	for (int i = 0; i < vertexList.size(); i++) {
		tripletList.push_back(T(i, i, -1));
		Eigen::Vector3f curr_p = vertexList[i]->position();
		OneRingVertex curr_vertex(vertexList[i]);
		std::vector<int> neigh_index;
		std::vector<double> neigh_weight;
		int neigh_num = 0;
		double weight_sum = 0;
		while (Vertex* neigh_vertex = curr_vertex.nextVertex()) {
			neigh_num = neigh_num + 1;
			neigh_index.push_back(neigh_vertex->index());
		}
		for (int j = 0; j < neigh_num; j++) {
			Eigen::Vector3f pre_p = vertexList[neigh_index[(j - 1 + neigh_num) % neigh_num]]->position();
			Eigen::Vector3f neigh_p = vertexList[neigh_index[j]]->position();
			Eigen::Vector3f next_p = vertexList[neigh_index[(j + 1) % neigh_num]]->position();
			double weight = (fabs((curr_p - next_p).dot((neigh_p - next_p))) / ((curr_p - next_p).cross((neigh_p - next_p))).norm() + fabs((curr_p - pre_p).dot((neigh_p - pre_p))) / ((curr_p - pre_p).cross((neigh_p - pre_p))).norm())/2;
			if (std::isnan(weight) || std::isinf(weight)) {
				neigh_weight.push_back(0);
			}
			else {
				neigh_weight.push_back(weight);
				weight_sum = weight_sum + weight;
			}
		}
		//std::cout << "weight sum: " << weight_sum << std::endl;
		for (int j = 0; j < neigh_index.size(); j++) {
			double weight = neigh_weight[j] / weight_sum;
			tripletList.push_back(T(i, neigh_index[j], weight));
		}
	}
	L->resize(mVertexList.size(), mVertexList.size());
	L->setFromTriplets(tripletList.begin(), tripletList.end());
}

void Mesh::umbrellaSmooth(bool cotangentWeights) {
	/*====== Programming Assignment 1 ======*/
	auto start = std::chrono::high_resolution_clock::now();
	smoothNum = smoothNum + 1;
	if (cotangentWeights) {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 1: Implement the cotangent weighting 
		/* scheme for explicit mesh smoothing. 
		/*
		/* Hint:
		/* It is advised to double type to store the 
		/* weights to avoid numerical issues.
		/**********************************************/
		std::cout << "The " << smoothNum << " iteration of explict cotangent weighting smoothing takes ";
		Eigen::SparseMatrix<float> L(mVertexList.size(), mVertexList.size());
		generateCotangtLforSmooth(&L, mVertexList);
		double lambda = 1;
		Eigen::MatrixXf curr_position(mVertexList.size(), 3);
		Eigen::MatrixXf updated_position(mVertexList.size(), 3);
		for (int i = 0; i < mVertexList.size(); i++) {
			curr_position.row(i) = mVertexList[i]->position();
		}
		updated_position = curr_position + lambda * L * curr_position;
		for (int i = 0; i < mVertexList.size(); i++){
			mVertexList[i]->setPosition(updated_position.row(i));
		}
	} else {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the uniform weighting 
		/* scheme for explicit mesh smoothing.
		/**********************************************/
		std::cout << "The " << smoothNum << " iteration of explict uniform weighting smoothing takes ";
		Eigen::SparseMatrix<float> L(mVertexList.size(), mVertexList.size());
		generateUniformLforSmooth(&L, mVertexList);
		double lambda = 1;
		Eigen::MatrixXf curr_position(mVertexList.size(), 3);
		Eigen::MatrixXf updated_position(mVertexList.size(), 3);
		for (int i = 0; i < mVertexList.size(); i++) {
			curr_position.row(i) = mVertexList[i]->position();
		}
		updated_position = curr_position + lambda * L * curr_position;
		for (int i = 0; i < mVertexList.size(); i++)
		{
			mVertexList[i]->setPosition(updated_position.row(i));
		}
	}

	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << duration.count() << " milliseconds to perform." << std::endl;
	runtime = runtime + duration.count();
	std::cout << "total time :" << runtime << std::endl;
}

void Mesh::implicitUmbrellaSmooth(bool cotangentWeights) {

	/*====== Programming Assignment 1 ======*/
	auto start = std::chrono::high_resolution_clock::now();
	smoothNum = smoothNum + 1;
	/* A sparse linear system Ax=b solver using the conjugate gradient method. */
	auto fnConjugateGradient = [](const Eigen::SparseMatrix< float >& A,
	                              const Eigen::VectorXf& b,
	                              int maxIterations,
	                              float errorTolerance,
	                              Eigen::VectorXf& x)
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Params:
		/*  A: 
		/*  b: 
		/*  maxIterations:	Max number of iterations
		/*  errorTolerance: Error tolerance for the early stopping condition
		/*  x:				Stores the final solution, but should be initialized. 
		/**********************************************/
		/*
		/* Step 1: Implement the biconjugate gradient
		/* method.
		/* Hint: https://en.wikipedia.org/wiki/Biconjugate_gradient_method
		/**********************************************/
		Eigen::BiCGSTAB<Eigen::SparseMatrix<float> > solver;
		solver.setMaxIterations(maxIterations);
		solver.setTolerance(errorTolerance);
		solver.compute(A);
		x = solver.solve(b);
	};

	/* IMPORTANT:
	/* Please refer to the following link about the sparse matrix construction in Eigen. */
	/* http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3 */

	if (cotangentWeights) {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the cotangent weighting 
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/*
		/* Hint:
		/* It is advised to double type to store the
		/* weights to avoid numerical issues.
		/**********************************************/
		std::cout << "The " << smoothNum << " iteration of implict cotangent weighting smoothing takes ";
		Eigen::SparseMatrix<float> L(mVertexList.size(), mVertexList.size());
		generateCotangtLforSmooth(&L, mVertexList);
		double lambda = 1;
		Eigen::SparseMatrix<float> I(mVertexList.size(), mVertexList.size());
		I.setIdentity();
		Eigen::SparseMatrix<float> A(mVertexList.size(), mVertexList.size());
		Eigen::VectorXf b(mVertexList.size());
		Eigen::VectorXf X(mVertexList.size());
		Eigen::MatrixXf updated_position(mVertexList.size(), 3);
		A = (I - lambda * L);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < mVertexList.size(); j++) {
				b[j] = mVertexList[j]->position()[i];
			}
			fnConjugateGradient(A, b, 100, 1e-8, X);
			updated_position.col(i) = X;
		}
		for (int i = 0; i < mVertexList.size(); i++) {
			mVertexList[i]->setPosition(updated_position.row(i));
		}
	} else {
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 3: Implement the uniform weighting 
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/**********************************************/
		std::cout << "The " << smoothNum << " iteration of implict uniform weighting smoothing takes ";
		Eigen::SparseMatrix<float> L(mVertexList.size(), mVertexList.size());
		generateUniformLforSmooth(&L, mVertexList);
		double lambda = 1;
		Eigen::SparseMatrix<float> I(mVertexList.size(), mVertexList.size());
		I.setIdentity();
		Eigen::SparseMatrix<float> A(mVertexList.size(), mVertexList.size());
		Eigen::VectorXf b(mVertexList.size());
		Eigen::VectorXf X(mVertexList.size());
		Eigen::MatrixXf updated_position(mVertexList.size(), 3);
		A = (I - lambda * L);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < mVertexList.size(); j++) {
				b[j] = mVertexList[j]->position()[i];
			}
			fnConjugateGradient(A, b, 100, 1e-8, X);
			updated_position.col(i) = X;
		}
		for (int i = 0; i < mVertexList.size(); i++) {
			mVertexList[i]->setPosition(updated_position.row(i));
		}
	}

	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << duration.count() << " milliseconds to perform." << std::endl;
	runtime = runtime + duration.count();
	std::cout << "total time :" << runtime << std::endl;
}
