#include <string>
#include <tuple>
#include <iostream>

#include <tinyexpr.h>
#include <json.hpp>

#include "core/types.h"

namespace {

struct ExprContext {
    double t = 0;
    te_variable var[1] = {{"t", &t}};
    te_expr* expr = nullptr;

    ~ExprContext() {
        te_free(expr); // te_free is safe to call on NULL ptr
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
} // namespace

using json = nlohmann::json;

int main() {
    json input;
    json output;

    try {
        std::cin >> input;
    } catch (...) { return 1; }

    Mechanism mech;
}