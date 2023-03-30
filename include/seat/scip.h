#pragma once

#include <assert>
#include <string>
#include <vector>

#include "scip/cons_indicator.h"
#include "scip/scip.h"
#include "scip/scip_copy.h"
#include "scip/scip_param.h"
#include "scip/scip_prob.h"
#include "scip/scipdefplugins.h"

#include "utl/verify.h"

#define SCIP_CHECKED(x) utl::verify(SCIP_OKAY == (x), "SCIP fail")

namespace seat {

struct scip {
  scip() {
    SCIP_CHECKED(SCIPcreate(&scip_));
    SCIP_CHECKED(SCIPincludeDefaultPlugins(scip_));
    SCIP_CHECKED(SCIPsetEmphasis(scip_, SCIP_PARAMEMPHASIS_FEASIBILITY,
                                 /*quiet=*/true));
    SCIP_CHECKED(
        SCIPsetIntParam(scip_, "timing/clocktype", SCIP_CLOCKTYPE_WALL));
    SCIP_CHECKED(SCIPcreateProb(scip_, "SCIP", nullptr, nullptr, nullptr,
                                nullptr, nullptr, nullptr, nullptr));
    SCIP_CHECKED(SCIPsetObjsense(scip_, SCIP_OBJSENSE_MINIMIZE));
  }

  SCIP_VAR* add_var(std::string const& name, double const lb, double const ub) {
    auto& scip_var = variables_.emplace_back();
    double tmp_obj_coef = 0.0;
    SCIP_CHECKED(SCIPcreateVar(scip_, &scip_var, name.c_str(), lb, ub,
                               tmp_obj_coef, SCIP_VARTYPE_INTEGER, true, false,
                               nullptr, nullptr, nullptr, nullptr, nullptr));
    SCIP_CHECKED(SCIPaddVar(scip_, scip_var));
    return scip_var;
  }

  void set_bounds(SCIP_VAR* var, double const lb, double const ub) {
    SCIP_CHECKED(SCIPchgVarLb(scip_, var, lb));
    SCIP_CHECKED(SCIPchgVarUb(scip_, var, ub));
  }

  void set_bounds(SCIP_CONS* constraint, double const lb, double const ub) {
    SCIP_CHECKED(SCIPchgCons(scip_, constraint, lb));
    SCIP_CHECKED(SCIPchgVarUb(scip_, constraint, ub));
  }

  SCIP_CONS* add_constraint(std::string const& name, double const lb,
                            double const ub) {
    auto& cons = constraints_.emplace_back();
    SCIP_CHECKED(SCIPcreateConsLinear(scip_, &cons, name.c_str(), 0, 0, nullptr,
                                      lb, ub,
                                      /*initial=*/true,
                                      /*separate=*/true,
                                      /*enforce=*/true,
                                      /*check=*/true,
                                      /*propagate=*/true,
                                      /*local=*/false,
                                      /*modifiable=*/false,
                                      /*dynamic=*/false,
                                      /*removable=*/false,
                                      /*stickingatnode=*/false));
    return cons;
  }

  void set_coefficient(SCIP_CONS* constraint, SCIP_VAR* var,
                       double coefficient) {
    SCIPaddCoefLinear(scip_, constraint, var, coefficient);
  }

  ~scip() {
    if (scip_ == nullptr) {
      return;
    }
    for (auto& var : variables_) {
      SCIP_CHECKED(SCIPreleaseVar(scip_, &var));
    }
    for (auto& c : constraints_) {
      SCIP_CHECKED(SCIPreleaseCons(scip_, &c));
    }
    SCIP_CHECKED(SCIPfree(&scip_));
    scip_ = nullptr;
  }

  std::vector<SCIP_VAR*> variables_;
  std::vector<SCIP_CONS*> constraints_;
  SCIP* scip_;
};

}  // namespace seat