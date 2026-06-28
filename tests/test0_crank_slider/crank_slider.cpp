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
    // Build constraints
    std::vector<std::unique_ptr<Constraint>> constraints;

    {
        auto c = std::make_unique<FixedConstraint>();
        c->jointId = 0;
        c->position = Vec2{0, 0};
        constraints.push_back(std::move(c));
    }

    double crankLen = 100.0;
    double omega = 10.0;

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
        constraints.push_back(std::move(c));
    }

    {
        auto c = std::make_unique<DistanceConstraint>();
        c->jointAId = 1;
        c->jointBId = 2;
        c->d = 200.0;
        constraints.push_back(std::move(c));
    }

    {
        auto c = std::make_unique<PrismaticConstraint>();
        c->jointId = 2;
        c->origin = Vec2{0, 0};
        c->direction = Vec2{1, 0};
        constraints.push_back(std::move(c));
    }

    Solver solver(std::move(constraints));

    // Initial guess: q = [x0, y0, x1, y1, x2, y2]
    VecXd q0(6);
    q0 << 0, 0, 100, 0, 300, 0;

    // Solve at t = 0 to 2π, step 0.05
    int nSteps = int(2 * M_PI / 0.05) + 1;
    printf("t,x_slider,y_slider\n");
    VecXd q = q0;
    for (int i = 0; i < nSteps; ++i) {
        double t = i * 0.05;
        q = solver.solvePosition(q, t);
        printf("%.6f,%.10f,%.10f\n", t, q[4], q[5]);
    }

    return 0;
}