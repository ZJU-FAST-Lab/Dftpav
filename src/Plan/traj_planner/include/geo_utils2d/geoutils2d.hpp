#ifndef GEOUTILS_HPP
#define GEOUTILS_HPP


#include "sdlp.hpp"
#include "quickhull.hpp"
#include <Eigen/Eigen>

#include <cfloat>
#include <cstdint>
#include <set>
#include <chrono>

namespace geoutils
{

// Each col of hPoly denotes a facet (outter_normal^T,point^T)^T
// The outter_normal is assumed to be NORMALIZED
inline bool findInterior(const Eigen::MatrixXd &hPoly,
                         Eigen::Vector3d &interior)
{
    int m = hPoly.cols();

    Eigen::MatrixXd A(m, 4);
    Eigen::VectorXd b(m), c(4), x(4);
    A.leftCols<3>() = hPoly.topRows<3>().transpose();
    A.rightCols<1>().setConstant(1.0);
    b = hPoly.topRows<3>().cwiseProduct(hPoly.bottomRows<3>()).colwise().sum().transpose();
    c.setZero();
    c(3) = -1.0;

    double minmaxsd = sdlp::linprog(c, A, b, x);
    interior = x.head<3>();

    return minmaxsd < 0.0 && !std::isinf(minmaxsd);
}

struct filterLess
{
    inline bool operator()(const Eigen::Vector3d &l,
                           const Eigen::Vector3d &r)
    {
        return l(0) < r(0) ||
               (l(0) == r(0) &&
                (l(1) < r(1) ||
                 (l(1) == r(1) &&
                  l(2) < r(2))));
    }
};

inline void filterVs(const Eigen::MatrixXd &rV,
                     const double &epsilon,
                     Eigen::MatrixXd &fV)
{
    double mag = std::max(fabs(rV.maxCoeff()), fabs(rV.minCoeff()));
    double res = mag * std::max(fabs(epsilon) / mag, DBL_EPSILON);
    std::set<Eigen::Vector3d, filterLess> filter;
    fV = rV;
    int offset = 0;
    Eigen::Vector3d quanti;
    for (int i = 0; i < rV.cols(); i++)
    {
        quanti = (rV.col(i) / res).array().round();
        if (filter.find(quanti) == filter.end())
        {
            filter.insert(quanti);
            fV.col(offset) = rV.col(i);
            offset++;
        }
    }
    fV = fV.leftCols(offset).eval();
    return;
}

// Each col of hPoly denotes a facet (outter_normal^T,point^T)^T
// The outter_normal is assumed to be NORMALIZED
// proposed epsilon is 1.0e-6
inline void enumerateVs(const Eigen::MatrixXd &hPoly,
                        const Eigen::Vector3d &inner,
                        Eigen::MatrixXd &vPoly,
                        const double epsilon = 1.0e-6)
{
    Eigen::RowVectorXd b = hPoly.topRows<3>().cwiseProduct(hPoly.bottomRows<3>()).colwise().sum() -
                           inner.transpose() * hPoly.topRows<3>();
    Eigen::MatrixXd A = hPoly.topRows<3>().array().rowwise() / b.array();

    quickhull::QuickHull<double> qh;
    double qhullEps = std::min(epsilon, quickhull::defaultEps<double>());
    // CCW is false because the normal in quickhull towards interior
    const auto cvxHull = qh.getConvexHull(A.data(), A.cols(), false, true, qhullEps);
    const auto &idBuffer = cvxHull.getIndexBuffer();
    int hNum = idBuffer.size() / 3;
    Eigen::MatrixXd rV(3, hNum);
    Eigen::Vector3d normal, point, edge0, edge1;
    for (int i = 0; i < hNum; i++)
    {
        point = A.col(idBuffer[3 * i + 1]);
        edge0 = point - A.col(idBuffer[3 * i]);
        edge1 = A.col(idBuffer[3 * i + 2]) - point;
        normal = edge0.cross(edge1); //cross in CW gives an outter normal
        rV.col(i) = normal / normal.dot(point);
    }
    filterVs(rV, epsilon, vPoly);
    vPoly = (vPoly.array().colwise() + inner.array()).eval();
    return;
}

// Each col of hPoly denotes a facet (outter_normal^T,point^T)^T
// The outter_normal is assumed to be NORMALIZED
// proposed epsilon is 1.0e-6
inline bool enumerateVs(const Eigen::MatrixXd &hPoly,
                        Eigen::MatrixXd &vPoly,
                        const double epsilon = 1.0e-6)
{
    Eigen::Vector3d inner;
    if (findInterior(hPoly, inner))
    {
        enumerateVs(hPoly, inner, vPoly, epsilon);
        return true;
    }
    else
    {
        return false;
    }
}

} // namespace geoutils

#endif