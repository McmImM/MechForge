#include <string>
#include <tuple>
#include <iostream>

#include <tinyexpr.h>
#include <json.hpp>

#include "core/types.h"
#include "core/solver.h"

namespace {

struct ExprContext {
    double t = 0;
    te_variable var[1] = {{"t", &t}};
    te_expr* expr = nullptr;

    ~ExprContext() {
        te_free(expr); // te_free is safe to call on nullptr
    }
};

std::tuple<R1toR1Fn, int> parseR1toR1Fn(const std::string& exprStr) {
    auto exprCtx = std::make_shared<ExprContext>();

    int err = 0;
    R1toR1Fn f;
    exprCtx->expr = te_compile(exprStr.c_str(), exprCtx->var, 1, &err);
    if (err != 0)
        f = nullptr;
    else
        f = [exprCtx](double t) -> double {
            exprCtx->t = t;
            return te_eval(exprCtx->expr);
        };

    return {f, err};
}

std::tuple<R1toR2Fn, int, int> parseR1toR2Fn(const std::string& exprStr1,
                                             const std::string& exprStr2) {
    auto [f1, err1] = parseR1toR1Fn(exprStr1);
    auto [f2, err2] = parseR1toR1Fn(exprStr2);

    R1toR2Fn f;
    if (err1 == 0 && err2 == 0)
        f = [f1, f2](double t) -> Vec2 { return {f1(t), f2(t)}; };
    else
        f = nullptr;

    return {f, err1, err2};
}

int errOutputJson(const std::string& message) {
    nlohmann::json output;
    output["status"] = "error";
    output["message"] = message;
    std::cout << output << std::endl;
    return 1;
}

std::optional<R1toR1Fn> parseR1(const std::string& name, const std::string& expr) {
    auto [fn, err] = parseR1toR1Fn(expr);
    if (err != 0) {
        errOutputJson("Failed to parse " + name + ": error occurs at "
                      + std::to_string(err));
        return std::nullopt;
    }
    return fn;
}

std::optional<R1toR2Fn> parseR2(const std::string& nameX, const std::string& exprX,
                                const std::string& nameY, const std::string& exprY) {
    auto [fn, err1, err2] = parseR1toR2Fn(exprX, exprY);
    if (err1 != 0) {
        errOutputJson("Failed to parse " + nameX + ": error occurs at "
                      + std::to_string(err1));
        return std::nullopt;
    }
    if (err2 != 0) {
        errOutputJson("Failed to parse " + nameY + ": error occurs at "
                      + std::to_string(err2));
        return std::nullopt;
    }
    return fn;
}
} // namespace

using json = nlohmann::json;

int main() {
    json input;

    try {
        std::cin >> input;
    } catch (const json::parse_error& e) {
        return errOutputJson(std::string("Failed to parse input JSON")
                             + std::to_string(e.byte) + ": " + e.what());
    }

    Mechanism mech;

    // extract Joints
    if (!input.contains("joints"))
        return errOutputJson("Input JSON must contain 'joints' field.");

    const auto& jointsJson = input["joints"];
    mech.joints.resize(jointsJson.size());
    for (const auto& jointJson : jointsJson) {
        int id = jointJson["id"];
        Joint& joint = mech.joints[id];

        joint.id = id;

        joint.type = static_cast<JointType>(jointJson["type"].get<int>());
        switch (joint.type) {
            using enum JointType;
        case Grounded:
            if (jointJson.contains("groundPos")) {
                mech.constraints.push_back(std::make_unique<GroundedConstraint>(
                    id, Vec2{jointJson["groundPos"][0].get<double>(),
                             jointJson["groundPos"][1].get<double>()}));
            } else
                return errOutputJson("Grounded joint must have groundPos field.");
            break;
        case Fixed: break;
        case Revolute: break;
        case Prismatic:
            if (jointJson.contains("slide")) {
                const auto& slideJson = jointJson["slide"];
                Vec2 origin{slideJson["origin"][0].get<double>(),
                            slideJson["origin"][1].get<double>()};
                Vec2 dir{slideJson["direction"][0].get<double>(),
                         slideJson["direction"][1].get<double>()};
                mech.constraints.push_back(
                    std::make_unique<PrismaticConstraint>(id, origin, dir));
            } else
                return errOutputJson("Prismatic joint must have slide field.");
            break;
        case Free: break;
        default:
            return errOutputJson("Unknown joint type: "
                                 + std::to_string(static_cast<int>(joint.type)));
            break;
        }

        joint.initialState.position =
            Vec2{jointJson["pos"][0].get<double>(), jointJson["pos"][1].get<double>()};

        if (jointJson.contains("vel"))
            joint.initialState.velocity = Vec2{jointJson["vel"][0].get<double>(),
                                               jointJson["vel"][1].get<double>()};

        if (jointJson.contains("acc"))
            joint.initialState.acceleration = Vec2{jointJson["acc"][0].get<double>(),
                                                   jointJson["acc"][1].get<double>()};
    }

    // extract Links
    if (input.contains("links")) {
        const auto& linksJson = input["links"];
        mech.links.resize(linksJson.size());
        for (const auto& linkJson : linksJson) {
            int id = linkJson["id"];
            Link& link = mech.links[id];

            link.id = id;

            link.jointA_id = linkJson["jointA"].get<int>();
            link.jointB_id = linkJson["jointB"].get<int>();

            link.length = linkJson["length"].get<double>();
            mech.constraints.push_back(std::make_unique<DistanceConstraint>(
                link.jointA_id, link.jointB_id, link.length));
        }
    }

    // extract Driving
    if (input.contains("drivings")) {
        const auto& drivingsJson = input["drivings"];
        for (const auto& drivingJson : drivingsJson) {
            if (!drivingJson.contains("type"))
                return errOutputJson("Driving must have 'type' field.");

            DrivingType drivingType =
                static_cast<DrivingType>(drivingJson["type"].get<int>());
            switch (drivingType) {
                using enum DrivingType;
            case Position: {
                if (!drivingJson.contains("jointId")
                    || !drivingJson.contains("relativeId")
                    || !drivingJson.contains("posX") || !drivingJson.contains("posY")
                    || !drivingJson.contains("velX") || !drivingJson.contains("velY")
                    || !drivingJson.contains("accX") || !drivingJson.contains("accY"))
                    return errOutputJson(
                        "PositionDriving must have 'jointId', 'relativeId', 'posX', "
                        "'posY', 'velX', 'velY', 'accX', 'accY' fields.");

                int jointId = drivingJson["jointId"].get<int>();
                int relativeId = drivingJson["relativeId"].get<int>();

                auto posFn =
                    parseR2("posX", drivingJson["posX"], "posY", drivingJson["posY"]);
                if (!posFn) return 1;
                auto velFn =
                    parseR2("velX", drivingJson["velX"], "velY", drivingJson["velY"]);
                if (!velFn) return 1;
                auto accFn =
                    parseR2("accX", drivingJson["accX"], "accY", drivingJson["accY"]);
                if (!accFn) return 1;

                mech.constraints.push_back(std::make_unique<PositionDriving>(
                    jointId, relativeId, *posFn, *velFn, *accFn));
                break;
            }

            case Angle: {
                if (!drivingJson.contains("jointId")
                    || !drivingJson.contains("relativeId")
                    || !drivingJson.contains("theta") || !drivingJson.contains("omega")
                    || !drivingJson.contains("alpha"))
                    return errOutputJson(
                        "AngleDriving must have 'jointId', 'relativeId', 'theta', "
                        "'omega', 'alpha', fields.");

                int jointId = drivingJson["jointId"].get<int>();
                int relativeId = drivingJson["relativeId"].get<int>();

                auto theta = parseR1("theta", drivingJson["theta"]);
                if (!theta) return 1;
                auto omega = parseR1("omega", drivingJson["omega"]);
                if (!omega) return 1;
                auto alpha = parseR1("alpha", drivingJson["alpha"]);
                if (!alpha) return 1;

                mech.constraints.push_back(std::make_unique<AngleDriving>(
                    jointId, relativeId, *theta, *omega, *alpha));
                break;
            }

            case Distance: {
                if (!drivingJson.contains("jointAId") || !drivingJson.contains("jointBId")
                    || !drivingJson.contains("distance") || !drivingJson.contains("vel")
                    || !drivingJson.contains("acc"))
                    return errOutputJson("DistanceDriving must have 'jointAId', "
                                         "'jointBId', 'distance', 'vel', 'acc' fields.");

                int jointAId = drivingJson["jointAId"].get<int>();
                int jointBId = drivingJson["jointBId"].get<int>();

                auto distance = parseR1("distance", drivingJson["distance"]);
                if (!distance) return 1;
                auto vel = parseR1("vel", drivingJson["vel"]);
                if (!vel) return 1;
                auto acc = parseR1("acc", drivingJson["acc"]);
                if (!acc) return 1;

                mech.constraints.push_back(std::make_unique<DistanceDriving>(
                    jointAId, jointBId, *distance, *vel, *acc));
                break;
            }

            default:
                return errOutputJson("Unknown driving type: "
                                     + std::to_string(static_cast<int>(drivingType)));
                break;
            }
        }
    }

    // solve
    Solver solver(mech);

    double endTime = -1;
    SolveLevel solveLevel = SolveLevel::Position;

    // extract SolverConfig
    if (input.contains("solverConfig")) {
        const auto& solverConfigJson = input["solverConfig"];

        if (solverConfigJson.contains("endTime"))
            endTime = solverConfigJson["endTime"].get<double>();

        solver.setTimeStep(solverConfigJson.value("timeStep", 0.01));

        solveLevel = static_cast<SolveLevel>(
            solverConfigJson.value("solveLevel", static_cast<int>(SolveLevel::Position)));
        solver.setSolveLevel(solveLevel);

        solver.setAccuracy(solverConfigJson.value("maxIter", 100),
                           solverConfigJson.value("tol", 1e-9));
    }

    StepCallback onStep = [&](double time, const VecXd& q, const VecXd& v,
                              const VecXd& a) {
        json output;
        output["status"] = "ok";
        output["time"] = time;
        output["joints"] = json::array();

        using enum SolveLevel;
        for (const auto& joint : mech.joints) {
            int i = 2 * joint.id;
            json jointJson;
            jointJson["id"] = joint.id;

            jointJson["x"] = q[i];
            jointJson["y"] = q[i + 1];

            if (solveLevel >= Velocity) {
                jointJson["vx"] = v[i];
                jointJson["vy"] = v[i + 1];
                if (solveLevel >= Acceleration) {
                    jointJson["ax"] = a[i];
                    jointJson["ay"] = a[i + 1];
                }
            }

            output["joints"].push_back(jointJson);
        }
        std::cout << output << std::endl;
    };

    solver.solve(endTime, onStep);
}