#ifndef __BOUNDINGSPHEREHIERARCHY_H__
#define __BOUNDINGSPHEREHIERARCHY_H__

#include "fabric_simulator/Common.h"
#include "fabric_simulator/utilities/BoundingSphere.h"
#include "fabric_simulator/utilities/kdTree.h"


namespace utilities
{
    class PointCloudBSH : public KDTree<BoundingSphere>
    {
    public:
        using super = KDTree<BoundingSphere>;

        PointCloudBSH();

        void init(const Eigen::Matrix<Real, Eigen::Dynamic, 3> *vertices, 
                  const unsigned int numVertices);

        const Eigen::Matrix<Real, 3, 1>& entity_position(unsigned int i) const override;

        void compute_hull(unsigned int b, 
                          unsigned int n, 
                          BoundingSphere& hull) const override;
        
        void compute_hull_approx(unsigned int b, 
                                 unsigned int n, 
                                 BoundingSphere& hull) const override;

    private:
        const Eigen::Matrix<Real, Eigen::Dynamic, 3> *m_vertices;
        mutable Eigen::Matrix<Real, 3, 1> m_tempPosition; // Temporary storage for entity position
    };
}

#endif // __BOUNDINGSPHEREHIERARCHY_H__
