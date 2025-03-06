#ifndef __AABB_H__
#define __AABB_H__

#include "fabric_simulator/Common.h"

#define FORCE_INLINE __attribute__((always_inline))

namespace utilities
{
	// Axis-Aligned Bounding-Box Class
	class AABB
	{
	public:
		// Two Points are sufficiemt to describe AABB
		
		// m_p[0]: The minimum corner of the bounding box, 
		// which contains the SMALLEST x, y, and z coordinates
		// that are encompassed by the box.
		// m_p[1]: The maximum corner of the bounding box, 
		// which contains the LARGEST x, y, and z coordinates 
		// that are encompassed by the box.
		Eigen::Matrix<Real, 3, 1> m_p[2];

		// Assignment operator: copies the min and max points from another AABB.
		AABB& operator = (const AABB& aabb)
		{ 
			m_p[0] = aabb.m_p[0]; 
			m_p[1] = aabb.m_p[1]; 
			return *this; 
		}

		// Checks if a point is inside this AABB.
		static bool pointInAABB(const AABB& a, const Eigen::Matrix<Real, 3, 1>& p)
		{
			if ((p[0] < a.m_p[0][0]) || (p[1] < a.m_p[0][1]) || (p[2] < a.m_p[0][2]))
				return false;
			if ((p[0] > a.m_p[1][0]) || (p[1] > a.m_p[1][1]) || (p[2] > a.m_p[1][2]))
				return false;
			return true;
		}

		// Retrieves the endpoints(p1 & p2) of an edge of the AABB(a), given its index(i).
		static void getEdge(const AABB& a, char i, Eigen::Matrix<Real, 3, 1>& p1, Eigen::Matrix<Real, 3, 1>& p2)
		{
			char c1, c2;
			getEdgeIndex(i, c1, c2);
			cornerPoint(a, c1, p1);
			cornerPoint(a, c2, p2);
		}

		// Maps an edge index to its corresponding corner points' indices.
		static void getEdgeIndex(char i, char& p1, char& p2)
		{
			//                         0    1    2    3    4    5    6    7    8    9    10   11
			static char index[12*2] = {0,1, 0,2, 1,3, 2,3, 0,4, 1,5, 2,6, 3,7, 4,5, 4,6, 5,7, 6,7};
			p1 = index[2*i+0];
			p2 = index[2*i+1];
		}

		// Computes a corner point of the AABB based on the index.
		static void cornerPoint(const AABB& a, char i, Eigen::Matrix<Real, 3, 1>& p)
		{
			switch (i)
			{
			case 0:
				p = Eigen::Matrix<Real, 3, 1>(a.m_p[0][0], a.m_p[0][1], a.m_p[0][2]);
				break;
			case 1:
				p = Eigen::Matrix<Real, 3, 1>(a.m_p[1][0], a.m_p[0][1], a.m_p[0][2]);
				break;
			case 2:
				p = Eigen::Matrix<Real, 3, 1>(a.m_p[0][0], a.m_p[1][1], a.m_p[0][2]);
				break;
			case 3:
				p = Eigen::Matrix<Real, 3, 1>(a.m_p[1][0], a.m_p[1][1], a.m_p[0][2]);
				break;
			case 4:
				p = Eigen::Matrix<Real, 3, 1>(a.m_p[0][0], a.m_p[0][1], a.m_p[1][2]);
				break;
			case 5:
				p = Eigen::Matrix<Real, 3, 1>(a.m_p[1][0], a.m_p[0][1], a.m_p[1][2]);
				break;
			case 6:
				p = Eigen::Matrix<Real, 3, 1>(a.m_p[0][0], a.m_p[1][1], a.m_p[1][2]);
				break;
			case 7:
				p = Eigen::Matrix<Real, 3, 1>(a.m_p[1][0], a.m_p[1][1], a.m_p[1][2]);
				break;
			}
		}

		// Checks if two AABBs intersect with each other.
		static FORCE_INLINE bool intersection(const AABB& a1, const AABB& a2)
		{
			for(char i=0;i<3;i++)
			{
				const Real min0 = a1.m_p[0][i];
				const Real max0 = a1.m_p[1][i];
				const Real min1 = a2.m_p[0][i];
				const Real max1 = a2.m_p[1][i];
				if (((max0 < min1) || (min0 > max1)))
					return false;
			}
			return true;
		}
	};
}

#endif
