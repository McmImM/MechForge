#include <string>
#include <tuple>
#include <iostream>

#include <tinyexpr.h>
#include <json.hpp>

#include "core/types.h"
#include "core/solver.h"

using json = nlohmann::json;

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

void errOutputJson(const std::string& message) {
    json output;
    output["status"] = "error";
    output["message"] = message;
    std::cout << output << std::endl;
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

int main() {
    Mechanism mech;

    std::string jsonline;
    json input;
    // The parent process should give the input JSON line by line.
    while (std::getline(std::cin, jsonline)) {
        try {
            input = json::parse(jsonline);
        } catch (const std::exception& e) {
            errOutputJson("Failed to parse input JSON: " + std::string(e.what()));
            continue;
        }

        // The input JSON should be gauranteed to be right in the parent process,
        // so we don't need to check the schema here.
        std::string cmd = input["cmd"];
        if (cmd == "build") {
            const json& mechJson = input["mech"];

            // parse joints
            const json& jointsJson = mechJson["joints"];
            mech.joints.resize(jointsJson.size());
            for (const json& jointJson : jointsJson) {
                // parse id
                int id = jointJson["id"];
                Joint& joint = mech.joints[id];
                joint.id = id;

                // parse type
                using enum JointType;
                std::string type = jointJson["type"];
                if (type == "Grounded")
                    joint.type = Grounded;
                else if (type == "Fixed")
                    joint.type = Fixed;
                else if (type == "Revolute")
                    joint.type = Revolute;
                else if (type == "Prismatic")
                    joint.type = Prismatic;
                else if (type == "Free")
                    joint.type = Free;

                switch (joint.type) {
                case Grounded:
                    mech.constraints.push_back(std::make_unique<GroundedConstraint>(
                        id, Vec2{jointJson["groundPos"][0].get<double>(),
                                 jointJson["groundPos"][1].get<double>()}));
                    break;
                case Fixed: break;
                case Revolute: break;
                case Prismatic: {
                    const json& slideJson = jointJson["slide"];
                    Vec2 origin{slideJson["pos"][0].get<double>(),
                                slideJson["pos"][1].get<double>()};
                    Vec2 dir{slideJson["axis"][0].get<double>(),
                             slideJson["axis"][1].get<double>()};
                    mech.constraints.push_back(
                        std::make_unique<PrismaticConstraint>(id, origin, dir));
                    break;
                }
                case Free: break;
                }

                // parse initial state
                // parse position
                joint.initialState.position = Vec2{jointJson["pos"][0].get<double>(),
                                                   jointJson["pos"][1].get<double>()};
                // parse velocity
                if (jointJson.contains("vel"))
                    joint.initialState.velocity = Vec2{jointJson["vel"][0].get<double>(),
                                                       jointJson["vel"][1].get<double>()};
                // parse acceleration
                if (jointJson.contains("acc"))
                    joint.initialState.acceleration =
                        Vec2{jointJson["acc"][0].get<double>(),
                             jointJson["acc"][1].get<double>()};
            }

            // parse links
            if (mechJson.contains("links")) {
                const json& linksJson = mechJson["links"];
                mech.links.resize(linksJson.size());
                for (const json& linkJson : linksJson) {
                    // parse id
                    int id = linkJson["id"];
                    Link& link = mech.links[id];
                    link.id = id;

                    // parse jointA and jointB
                    link.jointA_id = linkJson["jointA"].get<int>();
                    link.jointB_id = linkJson["jointB"].get<int>();

                    // parse length
                    link.length = linkJson["length"].get<double>();
                    mech.constraints.push_back(std::make_unique<DistanceConstraint>(
                        link.jointA_id, link.jointB_id, link.length));
                }
            }

            // parse drivings
            if (mechJson.contains("drivings")) {
                const json& drivingsJson = mechJson["drivings"];
                bool drivingFailed = false;
                for (const json& drivingJson : drivingsJson) {
                    // parse ids
                    int jointAId = drivingJson["jointA"].get<int>();
                    int jointBId = drivingJson["jointB"].get<int>();

                    // parse type
                    using enum DrivingType;
                    std::string drivingTypeStr = drivingJson["type"];
                    DrivingType drivingType;
                    if (drivingTypeStr == "position")
                        drivingType = Position;
                    else if (drivingTypeStr == "angle")
                        drivingType = Angle;
                    else if (drivingTypeStr == "distance")
                        drivingType = Distance;

                    switch (drivingType) {
                    case Position: {
                        auto pos = parseR2("posX", drivingJson["posX"], "posY",
                                           drivingJson["posY"]);
                        if (!pos) {
                            drivingFailed = true;
                            break;
                        }
                        auto vel = parseR2("velX", drivingJson["velX"], "velY",
                                           drivingJson["velY"]);
                        if (!vel) {
                            drivingFailed = true;
                            break;
                        }
                        auto acc = parseR2("accX", drivingJson["accX"], "accY",
                                           drivingJson["accY"]);
                        if (!acc) {
                            drivingFailed = true;
                            break;
                        }

                        mech.constraints.push_back(std::make_unique<PositionDriving>(
                            jointBId, jointAId, *pos, *vel, *acc));
                        break;
                    }
                    case Angle: {
                        auto angle = parseR1("angle", drivingJson["theta"]);
                        if (!angle) {
                            drivingFailed = true;
                            break;
                        }
                        auto omega = parseR1("omega", drivingJson["omega"]);
                        if (!omega) {
                            drivingFailed = true;
                            break;
                        }
                        auto alpha = parseR1("alpha", drivingJson["alpha"]);
                        if (!alpha) {
                            drivingFailed = true;
                            break;
                        }

                        mech.constraints.push_back(std::make_unique<AngleDriving>(
                            jointBId, jointAId, *angle, *omega, *alpha));
                        break;
                    }
                    case Distance: {
                        auto distance = parseR1("distance", drivingJson["distance"]);
                        if (!distance) {
                            drivingFailed = true;
                            break;
                        }
                        auto vel = parseR1("vel", drivingJson["vel"]);
                        if (!vel) {
                            drivingFailed = true;
                            break;
                        }
                        auto acc = parseR1("acc", drivingJson["acc"]);
                        if (!acc) {
                            drivingFailed = true;
                            break;
                        }

                        mech.constraints.push_back(std::make_unique<DistanceDriving>(
                            jointAId, jointBId, *distance, *vel, *acc));
                        break;
                    }
                    }
                    if (drivingFailed) break;
                }
                if (drivingFailed) {
                    mech = Mechanism{};
                    errOutputJson("Failed to build the mechanism due to "
                                  "invalid driving functions.");
                    continue;
                }
            }
        } else if (cmd == "solve") {
            if (mech.constraints.empty()) {
                errOutputJson(
                    "No constraints in the mechanism. Please build the mechanism first.");
                continue;
            }

            const json& solveJson = input["solveConfig"];
            double endTime = solveJson["endTime"].get<double>();
            double timeStep = solveJson["timeStep"].get<double>();
            int maxIter = solveJson["maxIterations"].get<int>();
            double tol = solveJson["tolerance"].get<double>();
            std::string solveLevelStr = solveJson["solveLevel"].get<std::string>();
            using enum SolveLevel;
            SolveLevel solveLevel;
            if (solveLevelStr == "pos")
                solveLevel = Position;
            else if (solveLevelStr == "vel")
                solveLevel = Velocity;
            else if (solveLevelStr == "acc")
                solveLevel = Acceleration;
            Solver solver(mech);
            solver.setTimeStep(timeStep);
            solver.setAccuracy(maxIter, tol);
            solver.setSolveLevel(solveLevel);
            StepCallback onStep = [&](double time, const VecXd& q, const VecXd& v,
                                      const VecXd& a) {
                json output;
                output["status"] = "ok";
                output["time"] = time;

                using enum SolveLevel;
                output["q"] = json::array();
                for (int i = 0; i < q.size(); i++)
                    output["q"].push_back(q[i]);

                if (solveLevel >= Velocity) {
                    output["v"] = json::array();
                    for (int i = 0; i < v.size(); i++)
                        output["v"].push_back(v[i]);
                    if (solveLevel >= Acceleration) {
                        output["a"] = json::array();
                        for (int i = 0; i < a.size(); i++)
                            output["a"].push_back(a[i]);
                    }
                }
                std::cout << output << std::endl;
            };
            solver.solve(endTime, onStep);

        } else if (cmd == "exit") {
            json output;
            output["status"] = "ok";
            output["message"] = "Exiting.";
            std::cout << output << std::endl;
            break;
        }
    }
}