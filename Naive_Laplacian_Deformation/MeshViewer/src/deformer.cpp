#include "deformer.h"
#include <iostream>

Deformer::Deformer() : mMesh(nullptr),
mCholeskySolver(nullptr) {
}

Deformer::~Deformer() {
	clear();
}

void Deformer::clear() {
	if (mCholeskySolver) {
		delete mCholeskySolver;
	}
	mCholeskySolver = nullptr;
	mRoiList.clear();
}

void Deformer::setMesh(Mesh* mesh) {
	mMesh = mesh;
	clear();
	// Record the handle vertices
	for (Vertex* vert : mMesh->vertices()) {
		if (vert->flag() > 0 || vert->isBoundary()) {
			mRoiList.push_back(vert);
		}
	}
	// Build system matrix for deformation
	buildSystemMat();
}

void Deformer::buildLb() {
	typedef Eigen::Triplet<double> T;
	std::vector<T> tripletList;
	tripletList.reserve(7 * mMesh->vertices().size());//E~3V
	mb.resize(mMesh->vertices().size() + mRoiList.size(), 3);
	for (int i = 0; i < mMesh->vertices().size(); i++) {
		tripletList.push_back(T(i, i, 1));
		Eigen::Vector3f curr_p = mMesh->vertices()[i]->position();
		Eigen::Vector3f delta_p = curr_p;
		OneRingVertex curr_vertex(mMesh->vertices()[i]);
		std::vector<int> neigh_index;
		std::vector<double> neigh_weight;
		int neigh_num = 0;
		double weight_sum = 0;
		while (Vertex* neigh_vertex = curr_vertex.nextVertex()) {
			neigh_num = neigh_num + 1;
			neigh_index.push_back(neigh_vertex->index());
		}
		for (int j = 0; j < neigh_num; j++) {
			Eigen::Vector3f pre_p = mMesh->vertices()[neigh_index[(j - 1 + neigh_num) % neigh_num]]->position();
			Eigen::Vector3f neigh_p = mMesh->vertices()[neigh_index[j]]->position();
			Eigen::Vector3f next_p = mMesh->vertices()[neigh_index[(j + 1) % neigh_num]]->position();
			double weight = (fabs((curr_p - next_p).dot((neigh_p - next_p))) / ((curr_p - next_p).cross((neigh_p - next_p))).norm() + fabs((curr_p - pre_p).dot((neigh_p - pre_p))) / ((curr_p - pre_p).cross((neigh_p - pre_p))).norm()) / 2;
			if (std::isnan(weight) || std::isinf(weight)) {
				neigh_weight.push_back(0);
			}
			else {
				neigh_weight.push_back(weight);
				weight_sum = weight_sum + weight;
			}
		}
		//	//std::cout << "weight sum: " << weight_sum << std::endl;
		for (int j = 0; j < neigh_index.size(); j++) {
			double weight = -neigh_weight[j] / weight_sum;
			tripletList.push_back(T(i, neigh_index[j], weight));
			delta_p = delta_p + weight * mMesh->vertices()[neigh_index[j]]->position();
		}
		mb.row(i) = Eigen::Vector3d(delta_p[0],delta_p[1],delta_p[2]);
	}
	for (int k = 0; k < mRoiList.size(); k++) {
		tripletList.push_back(T(mMesh->vertices().size() + k, mRoiList[k]->index(), 1.0));
	}

	mL.resize(mMesh->vertices().size() + mRoiList.size(), mMesh->vertices().size());
	mL.setFromTriplets(tripletList.begin(), tripletList.end());

}

void Deformer::buildSystemMat() {
	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Build the matrix of the linear system for
	/* deformation and do factorization, in order
	/* to reuse and speed up in Deformer::deform().
	/* Handle vertices are maked by Vertex::flag() > 0
	/* Movements of the specified handle are already
	/* recorded in Vertex::position()
	/**********************************************/

	Eigen::SparseMatrix< double > systemMat;
	buildLb();
	systemMat = mL.transpose() * mL;

	/*====== Programming Assignment 2 ======*/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html

	// Do factorization
	if (systemMat.nonZeros() > 0) {
		mCholeskySolver = new Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > >();
		mCholeskySolver->compute(systemMat);
		if (mCholeskySolver->info() != Eigen::Success) {
			// Decomposition failed
			std::cout << "Sparse decomposition failed\n";
		}
		else {
			std::cout << "Sparse decomposition succeeded\n";
		}
	}
}

void Deformer::deform() {
	if (mCholeskySolver == nullptr) {
		return;
	}

	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* This is the place where the editing techniques
	/* take place.
	/* Solve for the new vertex positions after the
	/* specified handles move using the factorized
	/* matrix from Deformer::buildSystemMat(), i.e.,
	/* mCholeskySolver defined in deformer.h
	/**********************************************/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html
	for (int i = 0; i < mRoiList.size(); i++) {
		mb.row(i + mMesh->vertices().size()) = Eigen::Vector3d(mRoiList[i]->position()(0), mRoiList[i]->position()(1), mRoiList[i]->position()(2));
	}
	Eigen::MatrixXd LTb = mL.transpose() * mb;
	Eigen::MatrixXd v(mMesh->vertices().size(), 3);
	for (int i = 0; i < 3; i++) {
		v.col(i) = mCholeskySolver->solve(LTb.col(i));
	}
	for (int i = 0; i < mMesh->vertices().size(); i++)
	{
		Eigen::Vector3f update_pos(v.row(i)[0],v.row(i)[1],v.row(i)[2]);
		mMesh->vertices()[i]->setPosition(update_pos);
	}

	/*====== Programming Assignment 2 ======*/
}
