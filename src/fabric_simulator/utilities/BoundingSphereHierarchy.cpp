#include "fabric_simulator/utilities/BoundingSphereHierarchy.h"

#include <iostream>
#include <unordered_set>
#include <set>

using pool_set = std::set<unsigned int>;
using namespace utilities;

PointCloudBSH::PointCloudBSH() : super(0, 10) {}

void PointCloudBSH::init(const Eigen::Matrix<Real, Eigen::Dynamic, 3> *vertices, 
                         const unsigned int numVertices) {
    m_lst.resize(numVertices);
    m_vertices = vertices;
}

const Eigen::Matrix<Real, 3, 1>& PointCloudBSH::entity_position(unsigned int i) const {
    m_tempPosition = m_vertices->row(i).transpose();
    return m_tempPosition;
}

void PointCloudBSH::compute_hull(unsigned int b, 
                                 unsigned int n, 
                                 BoundingSphere& hull) const {
    auto vertices_subset = std::vector<Eigen::Matrix<Real, 3, 1>>(n);
    for (unsigned int i = b; i < b + n; ++i)
        vertices_subset[i - b] = m_vertices->row(m_lst[i]).transpose();

    const BoundingSphere s(vertices_subset);

    hull.x() = s.x();
    hull.r() = s.r();
}

void PointCloudBSH::compute_hull_approx(unsigned int b, 
                                        unsigned int n, 
                                        BoundingSphere& hull) const {
	// compute center											
    Eigen::Matrix<Real, 3, 1> x;
    x.setZero();
    for (unsigned int i = b; i < b + n; i++) {
        x += m_vertices->row(m_lst[i]).transpose();
    }
    x /= static_cast<Real>(n);

    Real radius2 = 0.0;
    for (unsigned int i = b; i < b + n; i++) {
        radius2 = std::max(radius2, (x - m_vertices->row(m_lst[i]).transpose()).squaredNorm());
    }

    hull.x() = x;
    hull.r() = sqrt(radius2);
}
