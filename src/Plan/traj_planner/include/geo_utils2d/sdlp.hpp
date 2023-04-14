/* 
 * Copyright (c) 1990 Michael E. Hohmeyer, 
 *       hohmeyer@icemcfd.com
 * Permission is granted to modify and re-distribute this code in any manner
 * as long as this notice is preserved.  All standard disclaimers apply.
 * 
 * R. Seidel's algorithm for solving LPs (linear programs.)
 */

/* 
 * Copyright (c) 2021 Zhepei Wang,
 *       wangzhepei@live.com
 * 1. Bug fix in "move_to_front" function that "prev[m]" is illegally accessed
 *    while "prev" originally has only m ints. It is fixed by allocating a 
 *    "prev" with m + 1 ints.  
 * 2. Add Eigen interface.
 * Permission is granted to modify and re-distribute this code in any manner
 * as long as this notice is preserved.  All standard disclaimers apply.
 * 
 * Reference: Seidel, R. (1991), "Small-dimensional linear programming and convex hulls made easy", 
 *            Discrete & Computational Geometry 6 (1): 423–434, doi:10.1007/BF02574699
 */

#ifndef SDLP_HPP
#define SDLP_HPP

#include <Eigen/Eigen>

#include <cmath>
#include <random>

namespace sdlp
{
constexpr double prog_epsilon = 2.0e-16;

enum PROG_STATE
{
    /* minimum attained */
    MINIMUM = 0,
    /* no feasible region */
    INFEASIBLE,
    /* unbounded solution */
    UNBOUNDED,
    /* only a vertex in the solution set */
    AMBIGUOUS,
};

inline double dot2(const double a[2], const double b[2])
{
    return a[0] * b[0] + a[1] * b[1];
}

inline double cross2(const double a[2], const double b[2])
{
    return a[0] * b[1] - a[1] * b[0];
}

inline bool unit2(const double a[2], double b[2], double eps)
{
    double size;
    size = sqrt(a[0] * a[0] + a[1] * a[1]);
    if (size < 2 * eps)
    {
        return true;
    }
    b[0] = a[0] / size;
    b[1] = a[1] / size;
    return false;
}

/* unitize a d+1 dimensional point */
inline bool lp_d_unit(int d, const double *a, double *b)
{
    int i;
    double size;

    size = 0.0;
    for (i = 0; i <= d; i++)
    {
        size += a[i] * a[i];
    }
    if (size < (d + 1) * prog_epsilon * prog_epsilon)
    {
        return true;
    }
    size = 1.0 / sqrt(size);
    for (i = 0; i <= d; i++)
    {
        b[i] = a[i] * size;
    }
    return false;
}

/* optimize the objective function when there are no contraints */
inline int lp_no_constraints(int d, const double *n_vec, const double *d_vec, double *opt)
{
    int i;
    double n_dot_d, d_dot_d;

    n_dot_d = 0.0;
    d_dot_d = 0.0;
    for (i = 0; i <= d; i++)
    {
        n_dot_d += n_vec[i] * d_vec[i];
        d_dot_d += d_vec[i] * d_vec[i];
    }
    if (d_dot_d < prog_epsilon * prog_epsilon)
    {
        d_dot_d = 1.0;
        n_dot_d = 0.0;
    }
    for (i = 0; i <= d; i++)
    {
        opt[i] = -n_vec[i] + d_vec[i] * n_dot_d / d_dot_d;
    }
    /* normalize the optimal point */
    if (lp_d_unit(d, opt, opt))
    {
        opt[d] = 1.0;
        return AMBIGUOUS;
    }
    else
    {
        return MINIMUM;
    }
}

/* returns the index of the plane that is in i's place */
inline int move_to_front(int i, int *next, int *prev)
{
    int previ;
    if (i == 0 || i == next[0])
    {
        return i;
    }
    previ = prev[i];
    /* remove i from it's current position */
    next[prev[i]] = next[i];
    prev[next[i]] = prev[i];
    /* put i at the front */
    next[i] = next[0];
    prev[i] = 0;
    prev[next[i]] = i;
    next[0] = i;
    return previ;
}

inline void lp_min_lin_rat(int degen,
                           const double cw_vec[2],
                           const double ccw_vec[2],
                           const double n_vec[2],
                           const double d_vec[2],
                           double opt[2])
{
    double d_cw, d_ccw, n_cw, n_ccw;

    /* linear rational function case */
    d_cw = dot2(cw_vec, d_vec);
    d_ccw = dot2(ccw_vec, d_vec);
    n_cw = dot2(cw_vec, n_vec);
    n_ccw = dot2(ccw_vec, n_vec);
    if (degen)
    {
        /* if degenerate simply compare values */
        if (n_cw / d_cw < n_ccw / d_ccw)
        {
            opt[0] = cw_vec[0];
            opt[1] = cw_vec[1];
        }
        else
        {
            opt[0] = ccw_vec[0];
            opt[1] = ccw_vec[1];
        }
        /* check that the clock-wise and counter clockwise bounds are not near a poles */
    }
    else if (fabs(d_cw) > 2.0 * prog_epsilon &&
             fabs(d_ccw) > 2.0 * prog_epsilon)
    {
        /* the valid region does not contain a poles */
        if (d_cw * d_ccw > 0.0)
        {
            /* find which end has the minimum value */
            if (n_cw / d_cw < n_ccw / d_ccw)
            {
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
            else
            {
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
        }
        else
        {
            /* the valid region does contain a poles */
            if (d_cw > 0.0)
            {
                opt[0] = -d_vec[1];
                opt[1] = d_vec[0];
            }
            else
            {
                opt[0] = d_vec[1];
                opt[1] = -d_vec[0];
            }
        }
    }
    else if (fabs(d_cw) > 2.0 * prog_epsilon)
    {
        /* the counter clockwise bound is near a pole */
        if (n_ccw * d_cw > 0.0)
        {
            /* counter clockwise bound is a positive pole */
            opt[0] = cw_vec[0];
            opt[1] = cw_vec[1];
        }
        else
        {
            /* counter clockwise bound is a negative pole */
            opt[0] = ccw_vec[0];
            opt[1] = ccw_vec[1];
        }
    }
    else if (fabs(d_ccw) > 2.0 * prog_epsilon)
    {
        /* the clockwise bound is near a pole */
        if (n_cw * d_ccw > 2 * prog_epsilon)
        {
            /* clockwise bound is at a positive pole */
            opt[0] = ccw_vec[0];
            opt[1] = ccw_vec[1];
        }
        else
        {
            /* clockwise bound is at a negative pole */
            opt[0] = cw_vec[0];
            opt[1] = cw_vec[1];
        }
    }
    else
    {
        /* both bounds are near poles */
        if (cross2(d_vec, n_vec) > 0.0)
        {
            opt[0] = cw_vec[0];
            opt[1] = cw_vec[1];
        }
        else
        {
            opt[0] = ccw_vec[0];
            opt[1] = ccw_vec[1];
        }
    }
}

inline int wedge(const double (*halves)[2],
                 int m,
                 int *next,
                 int *prev,
                 double cw_vec[2],
                 double ccw_vec[2],
                 int *degen)
{
    int i;
    double d_cw, d_ccw;
    int offensive;

    *degen = 0;
    for (i = 0; i != m; i = next[i])
    {
        if (!unit2(halves[i], ccw_vec, prog_epsilon))
        {
            /* clock-wise */
            cw_vec[0] = ccw_vec[1];
            cw_vec[1] = -ccw_vec[0];
            /* counter-clockwise */
            ccw_vec[0] = -cw_vec[0];
            ccw_vec[1] = -cw_vec[1];
            break;
        }
    }
    if (i == m)
    {
        return UNBOUNDED;
    }
    i = 0;
    while (i != m)
    {
        offensive = 0;
        d_cw = dot2(cw_vec, halves[i]);
        d_ccw = dot2(ccw_vec, halves[i]);
        if (d_ccw >= 2 * prog_epsilon)
        {
            if (d_cw <= -2 * prog_epsilon)
            {
                cw_vec[0] = halves[i][1];
                cw_vec[1] = -halves[i][0];
                unit2(cw_vec, cw_vec, prog_epsilon);
                offensive = 1;
            }
        }
        else if (d_cw >= 2 * prog_epsilon)
        {
            if (d_ccw <= -2 * prog_epsilon)
            {
                ccw_vec[0] = -halves[i][1];
                ccw_vec[1] = halves[i][0];
                unit2(ccw_vec, ccw_vec, prog_epsilon);
                offensive = 1;
            }
        }
        else if (d_ccw <= -2 * prog_epsilon && d_cw <= -2 * prog_epsilon)
        {
            return INFEASIBLE;
        }
        else if ((d_cw <= -2 * prog_epsilon) ||
                 (d_ccw <= -2 * prog_epsilon) ||
                 (cross2(cw_vec, halves[i]) < 0.0))
        {
            /* degenerate */
            if (d_cw <= -2 * prog_epsilon)
            {
                unit2(ccw_vec, cw_vec, prog_epsilon);
            }
            else if (d_ccw <= -2 * prog_epsilon)
            {
                unit2(cw_vec, ccw_vec, prog_epsilon);
            }
            *degen = 1;
            offensive = 1;
        }
        /* place this offensive plane in second place */
        if (offensive)
        {
            i = move_to_front(i, next, prev);
        }
        i = next[i];
        if (*degen)
        {
            break;
        }
    }
    if (*degen)
    {
        while (i != m)
        {
            d_cw = dot2(cw_vec, halves[i]);
            d_ccw = dot2(ccw_vec, halves[i]);
            if (d_cw < -2 * prog_epsilon)
            {
                if (d_ccw < -2 * prog_epsilon)
                {
                    return INFEASIBLE;
                }
                else
                {
                    cw_vec[0] = ccw_vec[0];
                    cw_vec[1] = ccw_vec[1];
                }
            }
            else if (d_ccw < -2 * prog_epsilon)
            {
                ccw_vec[0] = cw_vec[0];
                ccw_vec[1] = cw_vec[1];
            }
            i = next[i];
        }
    }
    return MINIMUM;
}

/*
* return the minimum on the projective line
*/
inline int lp_base_case(const double (*halves)[2], /* halves --- half lines */
                        int m,                     /* m      --- terminal marker */
                        const double n_vec[2],     /* n_vec  --- numerator funciton */
                        const double d_vec[2],     /* d_vec  --- denominator function */
                        double opt[2],             /* opt    --- optimum  */
                        int *next,                 /* next, prev  --- double linked list of indices */
                        int *prev)
{
    double cw_vec[2], ccw_vec[2];
    int degen;
    int status;
    double ab;

    /* find the feasible region of the line */
    status = wedge(halves, m, next, prev, cw_vec, ccw_vec, &degen);

    if (status == INFEASIBLE)
    {
        return status;
    }
    /* no non-trivial constraints one the plane: return the unconstrained optimum */
    if (status == UNBOUNDED)
    {
        return lp_no_constraints(1, n_vec, d_vec, opt);
    }
    ab = fabs(cross2(n_vec, d_vec));
    if (ab < 2 * prog_epsilon * prog_epsilon)
    {
        if (dot2(n_vec, n_vec) < 2 * prog_epsilon * prog_epsilon ||
            dot2(d_vec, d_vec) > 2 * prog_epsilon * prog_epsilon)
        {
            /* numerator is zero or numerator and denominator are linearly dependent */
            opt[0] = cw_vec[0];
            opt[1] = cw_vec[1];
            status = AMBIGUOUS;
        }
        else
        {
            /* numerator is non-zero and denominator is zero minimize linear functional on circle */
            if (!degen && cross2(cw_vec, n_vec) <= 0.0 &&
                cross2(n_vec, ccw_vec) <= 0.0)
            {
                /* optimum is in interior of feasible region */
                opt[0] = -n_vec[0];
                opt[1] = -n_vec[1];
            }
            else if (dot2(n_vec, cw_vec) > dot2(n_vec, ccw_vec))
            {
                /* optimum is at counter-clockwise boundary */
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
            else
            {
                /* optimum is at clockwise boundary */
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
            status = MINIMUM;
        }
    }
    else
    {
        /* niether numerator nor denominator is zero */
        lp_min_lin_rat(degen, cw_vec, ccw_vec, n_vec, d_vec, opt);
        status = MINIMUM;
    }
    return status;
}

/* find the largest coefficient in a plane */
inline void findimax(const double *pln, int idim, int *imax)
{
    double rmax;
    int i;

    *imax = 0;
    rmax = fabs(pln[0]);
    for (i = 1; i <= idim; i++)
    {
        double ab;
        ab = fabs(pln[i]);
        if (ab > rmax)
        {
            *imax = i;
            rmax = ab;
        }
    }
}

inline void vector_up(const double *equation, int ivar, int idim,
                      const double *low_vector, double *vector)
{
    int i;

    vector[ivar] = 0.0;
    for (i = 0; i < ivar; i++)
    {
        vector[i] = low_vector[i];
        vector[ivar] -= equation[i] * low_vector[i];
    }
    for (i = ivar + 1; i <= idim; i++)
    {
        vector[i] = low_vector[i - 1];
        vector[ivar] -= equation[i] * low_vector[i - 1];
    }
    vector[ivar] /= equation[ivar];
}

inline void vector_down(const double *elim_eqn, int ivar, int idim,
                        const double *old_vec, double *new_vec)
{
    int i;
    double fac, ve, ee;
    ve = 0.0;
    ee = 0.0;
    for (i = 0; i <= idim; i++)
    {
        ve += old_vec[i] * elim_eqn[i];
        ee += elim_eqn[i] * elim_eqn[i];
    }
    fac = ve / ee;
    for (i = 0; i < ivar; i++)
    {
        new_vec[i] = old_vec[i] - elim_eqn[i] * fac;
    }
    for (i = ivar + 1; i <= idim; i++)
    {
        new_vec[i - 1] = old_vec[i] - elim_eqn[i] * fac;
    }
}

inline void plane_down(const double *elim_eqn, int ivar, int idim,
                       const double *old_plane, double *new_plane)
{
    double crit;
    int i;

    crit = old_plane[ivar] / elim_eqn[ivar];
    for (i = 0; i < ivar; i++)
    {
        new_plane[i] = old_plane[i] - elim_eqn[i] * crit;
    }
    for (i = ivar + 1; i <= idim; i++)
    {
        new_plane[i - 1] = old_plane[i] - elim_eqn[i] * crit;
    }
}

inline int linfracprog(const double *halves, /* halves  --- half spaces */
                       int istart,           /* istart  --- should be zero unless doing incremental algorithm */
                       int m,                /* m       --- terminal marker */
                       const double *n_vec,  /* n_vec   --- numerator vector */
                       const double *d_vec,  /* d_vec   --- denominator vector */
                       int d,                /* d       --- projective dimension */
                       double *opt,          /* opt     --- optimum */
                       double *work,         /* work    --- work space (see below) */
                       int *next,            /* next    --- array of indices into halves */
                       int *prev,            /* prev    --- array of indices into halves */
                       int max_size)         /* max_size --- size of halves array */
/*
**
** half-spaces are in the form
** halves[i][0]*x[0] + halves[i][1]*x[1] + 
** ... + halves[i][d-1]*x[d-1] + halves[i][d]*x[d] >= 0
**
** coefficients should be normalized
** half-spaces should be in random order
** the order of the half spaces is 0, next[0] next[next[0]] ...
** and prev[next[i]] = i
**
** halves: (max_size)x(d+1)
**
** the optimum has been computed for the half spaces
** 0 , next[0], next[next[0]] , ... , prev[istart]
** the next plane that needs to be tested is istart
**
** m is the index of the first plane that is NOT on the list
** i.e. m is the terminal marker for the linked list.
**
** the objective function is dot(x,nvec)/dot(x,dvec)
** if you want the program to solve standard d dimensional linear programming
** problems then n_vec = ( x0, x1, x2, ..., xd-1, 0)
** and           d_vec = (  0,  0,  0, ...,    0, 1)
** and halves[0] = (0, 0, ... , 1)
**
** work points to (max_size+3)*(d+2)*(d-1)/2 double space
*/
{
    int status;
    int i, j, imax;
    double *new_opt, *new_n_vec, *new_d_vec, *new_halves, *new_work;
    const double *plane_i;
    double val;

    if (d == 1 && m != 0)
    {
        return lp_base_case((const double(*)[2])halves, m, n_vec,
                            d_vec, opt, next, prev);
    }
    else
    {
        int d_vec_zero;
        val = 0.0;
        for (j = 0; j <= d; j++)
        {
            val += d_vec[j] * d_vec[j];
        }
        d_vec_zero = (val < (d + 1) * prog_epsilon * prog_epsilon);

        /* find the unconstrained minimum */
        if (!istart)
        {
            status = lp_no_constraints(d, n_vec, d_vec, opt);
        }
        else
        {
            status = MINIMUM;
        }
        if (m == 0)
        {
            return status;
        }

        /* allocate memory for next level of recursion */
        new_opt = work;
        new_n_vec = new_opt + d;
        new_d_vec = new_n_vec + d;
        new_halves = new_d_vec + d;
        new_work = new_halves + max_size * d;
        for (i = istart; i != m; i = next[i])
        {
            /* if the optimum is not in half space i then project the problem onto that plane */
            plane_i = halves + i * (d + 1);
            /* determine if the optimum is on the correct side of plane_i */
            val = 0.0;
            for (j = 0; j <= d; j++)
            {
                val += opt[j] * plane_i[j];
            }
            if (val < -(d + 1) * prog_epsilon)
            {
                /* find the largest of the coefficients to eliminate */
                findimax(plane_i, d, &imax);
                /* eliminate that variable */
                if (i != 0)
                {
                    double fac;
                    fac = 1.0 / plane_i[imax];
                    for (j = 0; j != i; j = next[j])
                    {
                        const double *old_plane;
                        double *new_plane;
                        int k;
                        double crit;

                        old_plane = halves + j * (d + 1);
                        new_plane = new_halves + j * d;
                        crit = old_plane[imax] * fac;
                        for (k = 0; k < imax; k++)
                        {
                            new_plane[k] = old_plane[k] - plane_i[k] * crit;
                        }
                        for (k = imax + 1; k <= d; k++)
                        {
                            new_plane[k - 1] = old_plane[k] - plane_i[k] * crit;
                        }
                    }
                }
                /* project the objective function to lower dimension */
                if (d_vec_zero)
                {
                    vector_down(plane_i, imax, d, n_vec, new_n_vec);
                    for (j = 0; j < d; j++)
                    {
                        new_d_vec[j] = 0.0;
                    }
                }
                else
                {
                    plane_down(plane_i, imax, d, n_vec, new_n_vec);
                    plane_down(plane_i, imax, d, d_vec, new_d_vec);
                }
                /* solve sub problem */
                status = linfracprog(new_halves, 0, i, new_n_vec,
                                     new_d_vec, d - 1, new_opt, new_work, next, prev, max_size);
                /* back substitution */
                if (status != INFEASIBLE)
                {
                    vector_up(plane_i, imax, d, new_opt, opt);

                    /* in line code for unit */
                    double size;
                    size = 0.0;
                    for (j = 0; j <= d; j++)
                        size += opt[j] * opt[j];
                    size = 1.0 / sqrt(size);
                    for (j = 0; j <= d; j++)
                        opt[j] *= size;
                }
                else
                {
                    return status;
                }
                /* place this offensive plane in second place */
                i = move_to_front(i, next, prev);
            }
        }
        return status;
    }
}

inline void rand_permutation(int n, int *p)
{
    typedef std::uniform_int_distribution<int> rand_int;
    typedef rand_int::param_type rand_range;
    static std::mt19937_64 gen;
    static rand_int rdi(0, 1);
    int i, j, t;
    for (i = 0; i < n; i++)
    {
        p[i] = i;
    }
    for (i = 0; i < n; i++)
    {
        rdi.param(rand_range(0, n - i - 1));
        j = rdi(gen) + i;
        t = p[j];
        p[j] = p[i];
        p[i] = t;
    }
}

inline double linprog(const Eigen::VectorXd &c,
                      const Eigen::MatrixXd &A,
                      const Eigen::VectorXd &b,
                      Eigen::VectorXd &x)
/*
**  min cTx, s.t. Ax<=b
**  dim(x) << dim(b)
*/
{
    int d = c.size();
    int m = b.size() + 1;
    x = Eigen::VectorXd::Zero(d);
    double minimum = INFINITY;

    int *perm, *next, *prev;
    double *halves, *n_vec, *d_vec, *work, *opt;
    int i, status = sdlp::AMBIGUOUS;

    perm = (int *)malloc((m - 1) * sizeof(int));
    next = (int *)malloc(m * sizeof(int));
    /* original allocated size is m, here changed by m + 1 for legal tail accessing */
    prev = (int *)malloc((m + 1) * sizeof(int));
    halves = (double *)malloc(m * (d + 1) * sizeof(double));
    n_vec = (double *)malloc((d + 1) * sizeof(double));
    d_vec = (double *)malloc((d + 1) * sizeof(double));
    work = (double *)malloc((m + 3) * (d + 2) * (d - 1) / 2 * sizeof(double));
    opt = (double *)malloc((d + 1) * sizeof(double));

    Eigen::Map<Eigen::MatrixXd> Af(halves, d + 1, m);
    Eigen::Map<Eigen::VectorXd> nv(n_vec, d + 1);
    Eigen::Map<Eigen::VectorXd> dv(d_vec, d + 1);
    Eigen::Map<Eigen::VectorXd> xf(opt, d + 1);

    Af.col(0).setZero();
    Af(d, 0) = 1.0;
    Af.topRightCorner(d, m - 1) = -A.transpose();
    Af.bottomRightCorner(1, m - 1) = b.transpose();
    nv.head(d) = c;
    nv(d) = 0.0;
    dv.setZero();
    dv(d) = 1.0;

    /* randomize the input planes */
    rand_permutation(m - 1, perm);
    /* previous to 0 is actually never used */
    prev[0] = 0;
    /* link the zero position in at the beginning */
    next[0] = perm[0] + 1;
    prev[perm[0] + 1] = 0;
    /* link the other planes */
    for (i = 0; i < m - 2; i++)
    {
        next[perm[i] + 1] = perm[i + 1] + 1;
        prev[perm[i + 1] + 1] = perm[i] + 1;
    }
    /* flag the last plane */
    next[perm[m - 2] + 1] = m;

    status = sdlp::linfracprog(halves, 0, m, n_vec, d_vec,
                               d, opt, work, next, prev, m);

    /* handle states for linprog whose definitions differ from linfracprog */
    if (status != sdlp::INFEASIBLE)
    {
        if (xf(d) != 0.0 && status != sdlp::UNBOUNDED)
        {
            x = xf.head(d) / xf(d);
            minimum = c.dot(x);
        }

        if (xf(d) == 0.0 || status == sdlp::UNBOUNDED)
        {
            x = xf.head(d);
            minimum = -INFINITY;
        }
    }

    free(perm);
    free(next);
    free(prev);
    free(halves);
    free(n_vec);
    free(d_vec);
    free(work);
    free(opt);

    return minimum;
}

} // namespace sdlp

#endif