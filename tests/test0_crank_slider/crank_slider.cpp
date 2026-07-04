#include "core/solver.h"
#include <cmath>
#include <cstdio>

/*
 * Crank-slider mechanism test (position analysis only)
 *
 * Joint 0: fixed pivot at (0, 0)
 * Joint 1: crank pin, driven relative to Joint 0
 * Joint 2: slider on horizontal rail
 *
 * Constraints:
 *   FixedConstraint(Joint 0, (0,0))
 *   PositionDriving(Joint 1, relative=Joint 0)   -- crank length 100
 *   DistanceConstraint(Joint 1, Joint 2, 200)     -- connecting rod
 *   PrismaticConstraint(Joint 2, horizontal)
 */

int main() {
    // Build mechanism
    Mechanism mech;

    mech.joints.resize(3);
    mech.joints[0].id = 0;
    mech.joints[0].initialState.position = Vec2{0, 0};
    mech.joints[1].id = 1;
    mech.joints[1].initialState.position = Vec2{100, 0};
    mech.joints[2].id = 2;
    mech.joints[2].initialState.position = Vec2{300, 0};

    double crankLen = 100.0;
    double omega = 10.0;

    // Fixed: Joint 0 at (0,0)
    {
        auto c = std::make_unique<FixedConstraint>();
        c->jointId = 0;
        c->position = Vec2{0, 0};
        mech.constraints.push_back(std::move(c));
    }

    // PositionDriving: Joint 1 relative to Joint 0
    {
        auto c = std::make_unique<PositionDriving>();
        c->jointId = 1;
        c->relativeId = 0;
        c->posFn = [=](double t) -> Vec2 {
            return {crankLen * cos(omega * t), crankLen * sin(omega * t)};
        };
        c->velFn = [=](double t) -> Vec2 {
            return {-crankLen * omega * sin(omega * t),
                    crankLen * omega * cos(omega * t)};
        };
        c->accFn = [=](double t) -> Vec2 {
            return {-crankLen * omega * omega * cos(omega * t),
                    -crankLen * omega * omega * sin(omega * t)};
        };
        mech.constraints.push_back(std::move(c));
    }

    // Distance: connecting rod Joint 1 ↔ Joint 2
    {
        auto c = std::make_unique<DistanceConstraint>();
        c->jointAId = 1;
        c->jointBId = 2;
        c->d = 200.0;
        mech.constraints.push_back(std::move(c));
    }

    // Prismatic: Joint 2 on horizontal rail
    {
        auto c = std::make_unique<PrismaticConstraint>();
        c->jointId = 2;
        c->origin = Vec2{0, 0};
        c->direction = Vec2{1, 0};
        mech.constraints.push_back(std::move(c));
    }

    Solver solver(mech);

    // Solve at t = 0 to 2π, step 0.05
    // solvePosition_oneStep uses m_q (from mech.initialState) as initial guess
    int nSteps = int(2 * M_PI / 0.05) + 1;
    printf("t,x_slider,y_slider\n");
    for (int i = 0; i < nSteps; ++i) {
        double t = i * 0.05;
        VecXd q = solver.solvePosition_oneStep(t);
        printf("%.6f,%.10f,%.10f\n", t, q[4], q[5]);
    }

    return 0;
}