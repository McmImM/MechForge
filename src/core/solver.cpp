#include "solver.h"

#include <cmath>

// Constraint

// static constraints

// fixed constraint: joint position is fixed
VecXd FixedConstraint::eval(const VecXd& q, double time) const {
    VecXd f(2);

    // [x-x_0, y-y_0]^T
    f << q[2 * jointId] - position[0], q[2 * jointId + 1] - position[1];

    return f;
}

MatXd FixedConstraint::jacobian(const VecXd& q, double time) const {
    MatXd J(2, q.size());
    J.setZero();                 // Jacobian Matrix
    J(0, 2 * jointId) = 1.0;     // 0 ... 1 0 ... 0
    J(1, 2 * jointId + 1) = 1.0; // 0 ... 0 1 ... 0
    return J;
}

VecXd FixedConstraint::velRHS(const VecXd& q, double time) const {
    return VecXd::Zero(2);
}

VecXd FixedConstraint::accRHS(const VecXd& q, const VecXd& v, double time) const {
    return VecXd::Zero(2);
}

// distance constraint: joint positions are separated by a fixed distance
VecXd DistanceConstraint::eval(const VecXd& q, double time) const {
    VecXd f(1);
    double dx = q[2 * jointAId] - q[2 * jointBId];
    double dy = q[2 * jointAId + 1] - q[2 * jointBId + 1];
    f(0) = dx * dx + dy * dy - d * d;
    return f;
}

MatXd DistanceConstraint::jacobian(const VecXd& q, double time) const {
    MatXd J(1, q.size());
    J.setZero();
    double dx = q[2 * jointAId] - q[2 * jointBId];
    double dy = q[2 * jointAId + 1] - q[2 * jointBId + 1];
    J(0, 2 * jointAId) = 2 * dx;
    J(0, 2 * jointAId + 1) = 2 * dy;
    J(0, 2 * jointBId) = -2 * dx;
    J(0, 2 * jointBId + 1) = -2 * dy;
    return J;
}

VecXd DistanceConstraint::velRHS(const VecXd& q, double time) const {
    return VecXd::Zero(1);
}
VecXd DistanceConstraint::accRHS(const VecXd& q, const VecXd& v, double time) const {
    double dvx = v[2 * jointAId] - v[2 * jointBId];
    double dvy = v[2 * jointAId + 1] - v[2 * jointBId + 1];
    VecXd ba(1);
    ba(0) = -2 * (dvx * dvx + dvy * dvy);
    return ba;
}

// prismatic constraint: joint position is constrained to a line
VecXd PrismaticConstraint::eval(const VecXd& q, double time) const {
    VecXd f(1);
    // (p-P_0) x d
    f(0) = (q[2 * jointId + 1] - origin[1]) * direction[0]
           - (q[2 * jointId] - origin[0]) * direction[1];
    return f;
}

MatXd PrismaticConstraint::jacobian(const VecXd& q, double time) const {
    MatXd J(1, q.size());
    J.setZero();
    J(0, 2 * jointId) = -direction[1];
    J(0, 2 * jointId + 1) = direction[0];
    return J;
}

VecXd PrismaticConstraint::velRHS(const VecXd& q, double time) const {
    return VecXd::Zero(1);
}
VecXd PrismaticConstraint::accRHS(const VecXd& q, const VecXd& v, double time) const {
    return VecXd::Zero(1);
}

// driving constraints

// position driving: joint position is driven by a function of time
VecXd PositionDriving::eval(const VecXd& q, double time) const {
    VecXd f(2);
    Vec2 pos = posFn(time);
    f(0) = q[2 * jointId] - q[2 * relativeId] - pos[0];
    f(1) = q[2 * jointId + 1] - q[2 * relativeId + 1] - pos[1];
    return f;
}

MatXd PositionDriving::jacobian(const VecXd& q, double time) const {
    MatXd J(2, q.size());
    J.setZero();
    J(0, 2 * jointId) = 1.0;
    J(0, 2 * relativeId) = -1.0;
    J(1, 2 * jointId + 1) = 1.0;
    J(1, 2 * relativeId + 1) = -1.0;
    return J;
}

VecXd PositionDriving::velRHS(const VecXd& q, double time) const {
    return velFn(time);
}
VecXd PositionDriving::accRHS(const VecXd& q, const VecXd& v, double time) const {
    return accFn(time);
}

// angle driving: joint angle is driven by a function of time
VecXd AngleDriving::eval(const VecXd& q, double time) const {
    double dx = q[2 * jointId] - q[2 * relativeId];
    double dy = q[2 * jointId + 1] - q[2 * relativeId + 1];
    double theta = angleFn(time);
    VecXd f(1);
    f(0) = dx * sin(theta) - dy * cos(theta);
    return f;
}

MatXd AngleDriving::jacobian(const VecXd& q, double time) const {
    double theta = angleFn(time);
    double c = cos(theta), s = sin(theta);
    MatXd J(1, q.size());
    J.setZero();
    J(0, 2 * jointId) = s;
    J(0, 2 * jointId + 1) = -c;
    J(0, 2 * relativeId) = -s;
    J(0, 2 * relativeId + 1) = c;
    return J;
}

VecXd AngleDriving::velRHS(const VecXd& q, double time) const {
    double dx = q[2 * jointId] - q[2 * relativeId];
    double dy = q[2 * jointId + 1] - q[2 * relativeId + 1];
    double theta = angleFn(time);
    double c = cos(theta), s = sin(theta);
    VecXd f(1);
    f(0) = -(dx * c + dy * s) * velFn(time);
    return f;
}

VecXd AngleDriving::accRHS(const VecXd& q, const VecXd& v, double time) const {
    double dx = q[2 * jointId] - q[2 * relativeId];
    double dy = q[2 * jointId + 1] - q[2 * relativeId + 1];
    double theta = angleFn(time);
    double c = cos(theta), s = sin(theta);
    double omega = velFn(time), alpha = accFn(time);
    double dvx = v[2 * jointId] - v[2 * relativeId];
    double dvy = v[2 * jointId + 1] - v[2 * relativeId + 1];
    VecXd f(1);
    f(0) = -alpha * (dx * c + dy * s) - 2 * omega * (dvx * c + dvy * s)
           + omega * omega * (dx * s - dy * c);
    return f;
}

// distance driving: joint distance is driven by a function of time
VecXd DistanceDriving::eval(const VecXd& q, double time) const {
    VecXd f(1);
    double dx = q[2 * jointAId] - q[2 * jointBId];
    double dy = q[2 * jointAId + 1] - q[2 * jointBId + 1];
    double d = distanceFn(time);
    f(0) = dx * dx + dy * dy - d * d;
    return f;
}

MatXd DistanceDriving::jacobian(const VecXd& q, double time) const {
    MatXd J(1, q.size());
    J.setZero();
    double dx = q[2 * jointAId] - q[2 * jointBId];
    double dy = q[2 * jointAId + 1] - q[2 * jointBId + 1];
    J(0, 2 * jointAId) = 2 * dx;
    J(0, 2 * jointAId + 1) = 2 * dy;
    J(0, 2 * jointBId) = -2 * dx;
    J(0, 2 * jointBId + 1) = -2 * dy;
    return J;
}

VecXd DistanceDriving::velRHS(const VecXd& q, double time) const {
    VecXd f(1);
    f(0) = 2 * distanceFn(time) * velFn(time);
    return f;
}
VecXd DistanceDriving::accRHS(const VecXd& q, const VecXd& v, double time) const {
    double dvx = v[2 * jointAId] - v[2 * jointBId];
    double dvy = v[2 * jointAId + 1] - v[2 * jointBId + 1];
    double d = distanceFn(time), vd = velFn(time), ad = accFn(time);
    VecXd ba(1);
    ba(0) = 2 * (vd * vd + d * ad) - 2 * (dvx * dvx + dvy * dvy);
    return ba;
}

// Solver

Solver::Solver(const Mechanism& mech) :
    m_nq(mech.joints.size() * 2), m_constraints(mech.constraints) {
    m_q.resize(m_nq);
    m_v.resize(m_nq);
    m_a.resize(m_nq);
    for (const auto& joint : mech.joints) {
        int i = 2 * joint.id;
        m_q.segment(i, 2) = joint.initialState.position;
        m_v.segment(i, 2) = joint.initialState.velocity;
        m_a.segment(i, 2) = joint.initialState.acceleration;
    }

    for (const auto& constraint : m_constraints)
        m_totalEq += constraint->eqNum();
}

void Solver::setAccuracy(int maxIter, double tol) {
    m_maxIter = maxIter;
    m_tol = tol;
}

VecXd Solver::solvePosition_oneStep(double time) const {
    VecXd q = m_q; // initial guess q0
    bool converged = false;

    for (int i = 0; i < m_maxIter; i++) {
        VecXd F = assembleEval(q, time);
        MatXd J = assembleJacobian(q, time);

        auto qr = J.colPivHouseholderQr();

        VecXd dq;
        if (qr.rank() < J.cols()) {
            // Underdetermined system, use least squares solution
            auto svd = J.jacobiSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>();
            dq = svd.solve(-F);
        } else {
            // Determined or overdetermined system, use least squares solution
            dq = qr.solve(-F);
        }

        q += dq;
        if (dq.norm() < m_tol) {
            converged = true;
            break;
        }
    }

    if (!converged) {
        throw std::runtime_error("Solver: failed to converge after "
                                 + std::to_string(m_maxIter) + " iterations");
    }

    // residual check;
    // if the residual is too large, there may be a problem with the
    // constraints or the initial guess
    VecXd F_final = assembleEval(q, time);
    if (F_final.norm() > m_tol * 100) {
        throw std::runtime_error("Solver: conflicting constraints detected, "
                                 "residual ||F|| = "
                                 + std::to_string(F_final.norm()));
    }

    return q;
}

VecXd Solver::solveVelocity_oneStep(double time) const {
    MatXd J = assembleJacobian(m_q, time);
    VecXd Bv = assembleVelRHS(m_q, time);
    return J.colPivHouseholderQr().solve(Bv);
}

VecXd Solver::solveAcceleration_oneStep(double time) const {
    MatXd J = assembleJacobian(m_q, time);
    VecXd Ba = assembleAccRHS(m_q, m_v, time);
    return J.colPivHouseholderQr().solve(Ba);
}

void Solver::solveAll_oneStep(double time) {
    m_q = solvePosition_oneStep(time);
    m_v = solveVelocity_oneStep(time);
    m_a = solveAcceleration_oneStep(time);
}

void Solver::setTimeStep(double dt) {
    step = dt;
}

VecXd Solver::solvePosition(double endTime) {
    for (double t = step; t <= endTime; t += step)
        m_q = solvePosition_oneStep(t);

    return m_q;
}

VecXd Solver::solveVelocity(double endTime) {
    for (double t = step; t <= endTime; t += step) {
        m_q = solvePosition_oneStep(t);
        m_v = solveVelocity_oneStep(t);
    }

    return m_v;
}

VecXd Solver::solveAcceleration(double endTime) {
    for (double t = step; t <= endTime; t += step) {
        m_q = solvePosition_oneStep(t);
        m_v = solveVelocity_oneStep(t);
        m_a = solveAcceleration_oneStep(t);
    }

    return m_a;
}

void Solver::solveAll(double endTime) {
    for (double t = step; t <= endTime; t += step)
        solveAll_oneStep(t);
}

VecXd Solver::assembleEval(const VecXd& q, double time) const {
    VecXd F(m_totalEq);
    int row = 0;
    for (const auto& constraint : m_constraints) {
        VecXd f = constraint->eval(q, time);
        F.segment(row, f.size()) = f;
        row += f.size();
    }
    return F;
}

MatXd Solver::assembleJacobian(const VecXd& q, double time) const {
    MatXd J(m_totalEq, m_nq);
    int row = 0;
    for (const auto& constraint : m_constraints) {
        MatXd j = constraint->jacobian(q, time);
        J.middleRows(row, j.rows()) = j;
        row += j.rows();
    }
    return J;
}

VecXd Solver::assembleVelRHS(const VecXd& q, double time) const {
    VecXd Bv(m_totalEq);
    int row = 0;
    for (const auto& constraint : m_constraints) {
        VecXd bv = constraint->velRHS(q, time);
        Bv.segment(row, bv.size()) = bv;
        row += bv.size();
    }
    return Bv;
}

VecXd Solver::assembleAccRHS(const VecXd& q, const VecXd& v, double time) const {
    VecXd Ba(m_totalEq);
    int row = 0;
    for (const auto& constraint : m_constraints) {
        VecXd ba = constraint->accRHS(q, v, time);
        Ba.segment(row, ba.size()) = ba;
        row += ba.size();
    }
    return Ba;
}
