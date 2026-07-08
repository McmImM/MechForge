#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <functional>

// Type Alias

// vector types
using Vec2 = Eigen::Vector2d;
using VecXd = Eigen::VectorXd;

// matrix types
using MatXd = Eigen::MatrixXd;

// function types
using R1toR1Fn = std::function<double(double)>;
using R1toR2Fn = std::function<Vec2(double)>;

// Joint
enum class JointType { None, Fixed, Revolute, Prismatic };

struct JointState {
    Vec2 position = {0, 0};
    Vec2 velocity = {0, 0};
    Vec2 acceleration = {0, 0};
};

struct Joint {
    int id = -1;
    JointType type = JointType::None;
    JointState initialState;
};

// Link
struct LinkState {
    double angle = 0;
    double angularVel = 0;
    double angularAccel = 0;
};

struct Link {
    int id = -1;
    int jointA_id = -1;
    int jointB_id = -1;
    double length = 0;
    LinkState initialState;
};

// Constraint base
struct Constraint {
    virtual ~Constraint() = default;

    // num of the equations
    virtual constexpr int eqNum() const = 0;

    // F(q), evaluate the error of the constraint equation.
    virtual VecXd eval(const VecXd& q, double time) const = 0;

    // J = ∂F/∂q, jacobian matrix
    virtual MatXd jacobian(const VecXd& q, double time) const = 0;

    // -∂F/∂t, velocity RHS
    virtual VecXd velRHS(const VecXd& q, double time) const = 0;

    // -∂²F/∂t²-J'q', acceleration RHS
    virtual VecXd accRHS(const VecXd& q, const VecXd& v, double time) const = 0;
};

// Mechanism: a complete mechanism description
struct Mechanism {
    std::vector<Joint> joints;
    std::vector<Link> links;
    std::vector<std::unique_ptr<Constraint>> constraints;
};

// Log
struct KinematicsLog {
    std::vector<double> times = {};
    std::vector<VecXd> q = {};
    std::vector<VecXd> v = {};
    std::vector<VecXd> a = {};
};