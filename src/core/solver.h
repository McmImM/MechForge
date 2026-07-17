#pragma once

#include "types.h"

#include <memory>

// static constraints

struct GroundedConstraint final : Constraint {
    int jointId;
    Vec2 position;

    GroundedConstraint(int jointId, const Vec2& position) :
        jointId(jointId), position(position) {
    }

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

    DistanceConstraint(int jointAId, int jointBId, double d) :
        jointAId(jointAId), jointBId(jointBId), d(d) {
    }

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

    PrismaticConstraint(int jointId, const Vec2& origin, const Vec2& direction) :
        jointId(jointId), origin(origin), direction(direction) {
    }

    constexpr int eqNum() const override {
        return 1;
    }
    VecXd eval(const VecXd& q, double time) const override;
    MatXd jacobian(const VecXd& q, double time) const override;
    VecXd velRHS(const VecXd& q, double time) const override;
    VecXd accRHS(const VecXd& q, const VecXd& v, double time) const override;
};

// driving constraints

enum class DrivingType { Position, Angle, Distance };

struct PositionDriving final : Constraint {
    int jointId, relativeId;
    R1toR2Fn posFn;
    R1toR2Fn velFn;
    R1toR2Fn accFn;

    PositionDriving(int jointId, int relativeId, R1toR2Fn posFn, R1toR2Fn velFn,
                    R1toR2Fn accFn) :
        jointId(jointId), relativeId(relativeId), posFn(std::move(posFn)),
        velFn(std::move(velFn)), accFn(std::move(accFn)) {
    }

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

    AngleDriving(int jointId, int relativeId, R1toR1Fn angleFn, R1toR1Fn velFn,
                 R1toR1Fn accFn) :
        jointId(jointId), relativeId(relativeId), angleFn(std::move(angleFn)),
        velFn(std::move(velFn)), accFn(std::move(accFn)) {
    }

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

    DistanceDriving(int jointAId, int jointBId, R1toR1Fn distanceFn, R1toR1Fn velFn,
                    R1toR1Fn accFn) :
        jointAId(jointAId), jointBId(jointBId), distanceFn(std::move(distanceFn)),
        velFn(std::move(velFn)), accFn(std::move(accFn)) {
    }

    constexpr int eqNum() const override {
        return 1;
    }
    VecXd eval(const VecXd& q, double time) const override;
    MatXd jacobian(const VecXd& q, double time) const override;
    VecXd velRHS(const VecXd& q, double time) const override;
    VecXd accRHS(const VecXd& q, const VecXd& v, double time) const override;
};

// Solver

enum class SolveLevel { Position, Velocity, Acceleration };

using StepCallback =
    std::function<void(double time, const VecXd& q, const VecXd& v, const VecXd& a)>;

class Solver {
    int m_totalEq = 0;
    int m_nq = 0; // number of generalized coordinates ?maybe not needed

    const std::vector<std::unique_ptr<Constraint>>& m_constraints;
    VecXd m_q, m_v, m_a; // generalized coordinates, velocities, accelerations

    int m_maxIter = 100;
    double m_tol = 1e-9;

    double m_timeStep = 0.01;
    SolveLevel m_level = SolveLevel::Position;

    bool m_logEnabled = true;                               // log the solver progress
    mutable std::unique_ptr<KinematicsLog> m_log = nullptr; // log of the kinematics

public:
    Solver(const Mechanism& mech, bool log = true);

    void setAccuracy(int maxIter = 100, double tol = 1e-9);
    void setSolveLevel(SolveLevel level);

    VecXd solvePosition_oneStep(const VecXd& q0, double time) const;
    VecXd solveVelocity_oneStep(const VecXd& q, double time) const;
    VecXd solveAcceleration_oneStep(const VecXd& q, const VecXd& v, double time) const;
    void solve_oneStep(double time, StepCallback onStep = nullptr);

    void setTimeStep(double dt);
    void solve(double endTime = -1, StepCallback onStep = nullptr);

    const KinematicsLog& getLog();
    void clearLog();

private:
    VecXd assembleEval(const VecXd& q, double time) const;
    MatXd assembleJacobian(const VecXd& q, double time) const;
    VecXd assembleVelRHS(const VecXd& q, double time) const;
    VecXd assembleAccRHS(const VecXd& q, const VecXd& v, double time) const;
};