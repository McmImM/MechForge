#pragma once

#include "types.h"

#include <memory>

// static constraints

struct FixedConstraint final : Constraint {
    int jointId;
    Vec2 position;

    constexpr int eqNum() const override {
        return 2;
    }
    VecXd eval(const VecXd& q, double time) const override;
    MatXd jacobian(const VecXd& q, double time) const override;
    VecXd velRHS(const VecXd& q, double time) const override;
    VecXd accRHS(const VecXd& q, const VecXd& v, double time) const override;
};

struct DistanceConstraint final : Constraint {
    int jointAId, jointBId;
    double d;

    constexpr int eqNum() const override {
        return 1;
    }
    VecXd eval(const VecXd& q, double time) const override;
    MatXd jacobian(const VecXd& q, double time) const override;
    VecXd velRHS(const VecXd& q, double time) const override;
    VecXd accRHS(const VecXd& q, const VecXd& v, double time) const override;
};

struct PrismaticConstraint final : Constraint {
    int jointId;
    Vec2 origin;
    Vec2 direction;

    constexpr int eqNum() const override {
        return 1;
    }
    VecXd eval(const VecXd& q, double time) const override;
    MatXd jacobian(const VecXd& q, double time) const override;
    VecXd velRHS(const VecXd& q, double time) const override;
    VecXd accRHS(const VecXd& q, const VecXd& v, double time) const override;
};

// driving constraints

struct PositionDriving final : Constraint {
    int jointId, relativeId;
    R1toR2Fn posFn;
    R1toR2Fn velFn;
    R1toR2Fn accFn;

    constexpr int eqNum() const override {
        return 2;
    }
    VecXd eval(const VecXd& q, double time) const override;
    MatXd jacobian(const VecXd& q, double time) const override;
    VecXd velRHS(const VecXd& q, double time) const override;
    VecXd accRHS(const VecXd& q, const VecXd& v, double time) const override;
};

struct AngleDriving final : Constraint {
    int jointId, relativeId;
    R1toR1Fn angleFn;
    R1toR1Fn velFn;
    R1toR1Fn accFn;

    constexpr int eqNum() const override {
        return 1;
    }
    VecXd eval(const VecXd& q, double time) const override;
    MatXd jacobian(const VecXd& q, double time) const override;
    VecXd velRHS(const VecXd& q, double time) const override;
    VecXd accRHS(const VecXd& q, const VecXd& v, double time) const override;
};

struct DistanceDriving final : Constraint {
    int jointAId, jointBId;
    R1toR1Fn distanceFn;
    R1toR1Fn velFn;
    R1toR1Fn accFn;

    constexpr int eqNum() const override {
        return 1;
    }
    VecXd eval(const VecXd& q, double time) const override;
    MatXd jacobian(const VecXd& q, double time) const override;
    VecXd velRHS(const VecXd& q, double time) const override;
    VecXd accRHS(const VecXd& q, const VecXd& v, double time) const override;
};

// Solver
class Solver {
    int m_totalEq = 0;
    int m_nq = 0; // number of generalized coordinates
    std::vector<std::unique_ptr<Constraint>> m_constraints;

    int m_maxIter = 100;
    double m_tol = 1e-9;

public:
    Solver(std::vector<std::unique_ptr<Constraint>> constraints);

    void setAccuracy(int maxIter, double tol);

    VecXd solvePosition(const VecXd& q0, double time) const;

    VecXd solveVelocity(const VecXd& q, const VecXd& v0, double time) const;

    VecXd solveAcceleration(const VecXd& q, const VecXd& v, const VecXd& a0,
                            double time) const;

    void solveAll(const VecXd& q0, const VecXd& v0, const VecXd& a0, double time,
                  VecXd& q, VecXd& v, VecXd& a) const;

    static void writeBack(const VecXd& q, const VecXd& v, const VecXd& a,
                          Mechanism& mech);

    static VecXd extractQ(const Mechanism& mech);

private:
    VecXd assembleEval(const VecXd& q, double time) const;
    MatXd assembleJacobian(const VecXd& q, double time) const;
    VecXd assembleVelRHS(const VecXd& q, double time) const;
    VecXd assembleAccRHS(const VecXd& q, const VecXd& v, double time) const;
};