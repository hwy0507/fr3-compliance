#pragma once
#include "../fr3_contact/active_vmc.hpp"
namespace fr3_esn128 {
using namespace fr3_contact;
namespace weights {
#include "model_generated.inc"
}
// Fixed-size, no heap allocation or file I/O in step. One call per 40 ms.
// Exact supported frozen architecture, not a generic NPZ model interpreter.
class Esn128 {
 public:
  Eigen::Matrix<double,128,1> state=Eigen::Matrix<double,128,1>::Zero();
  void reset() {state.setZero();}
  V7 step(const Obs& obs,double dt=.04) {
    if(!obs.allFinite()||!std::isfinite(dt)||std::abs(dt-.04)>1e-12)
      throw std::invalid_argument("ESN requires finite canonical48 at 25Hz");
    Eigen::Matrix<double,6,1> x;
    for(int i=0;i<6;++i)x[i]=clip(obs[39+i]/weights::k_scale[i],-12,12);
    Eigen::Matrix<double,12,1> context,drive;
    for(int i=0;i<3;++i) {
      context[i]=clip(obs[14+i]/.06,-4,4);
      context[3+i]=clip(obs[26+i]/.06,-4,4);
      context[6+i]=clip(obs[45+i],-4,4);
      context[9+i]=clip(obs[20+i]/.08,-4,4);
    }
    drive.head<6>()=x;drive.tail<6>()=x.cwiseAbs();
    Eigen::Matrix<double,144,1> cross;
    for(int i=0;i<12;++i)for(int j=0;j<12;++j)cross[12*i+j]=drive[i]*context[j];
    Eigen::Matrix<double,156,1> input;input<<drive,cross;
    const Eigen::Map<const Eigen::Matrix<double,128,1>> leak(weights::k_leak);
    Eigen::Matrix<double,128,1> response;
    for (int i=0;i<128;++i) {
      const Eigen::Map<const Eigen::Matrix<double,156,1>> input_row(weights::k_win+156*i);
      const Eigen::Map<const Eigen::Matrix<double,128,1>> recurrent_row(weights::k_w+128*i);
      response[i]=std::tanh(input_row.dot(input)+recurrent_row.dot(state));
    }
    state+=leak.cwiseProduct(response-state);
    Eigen::Matrix<double,134,1> signed_features;signed_features<<x,state;
    const Eigen::Map<const Eigen::Matrix<double,128,12,Eigen::RowMajor>> projection(weights::k_state_context_readout_w);
    Eigen::Matrix<double,552,1> features;
    features<<signed_features,signed_features.cwiseAbs(),cross,
      state.cwiseProduct((projection*context).array().tanh().matrix()),context;
    const Eigen::Map<const Eigen::Matrix<double,552,7,Eigen::RowMajor>> readout(weights::k_weights);
    const V7 raw=readout.transpose()*features;
    if(!raw.allFinite()||!state.allFinite())throw std::runtime_error("nonfinite ESN state");
    V7 result=raw.cwiseMax(-1).cwiseMin(1);result[0]=clip(raw[0],0,1);return result;
  }
};
// The frozen filter runs at 25Hz, holds outputs between ticks. Not torque output.
struct ActionFilter {
  double scale=1;V6 yield=V6::Zero();
  void update(const V7& action) {
    if(!action.allFinite())throw std::invalid_argument("invalid action");
    const V7 a=action.cwiseMax(-1).cwiseMin(1);
    scale+=clip(1-.8*std::max(0.,a[0])-scale,-4*.04,4*.04);
    for(int i=0;i<6;++i)yield[i]+=clip(a[i+1]*(i<3?.32:.6)-yield[i],-(i<3?1.2:4)*.04,(i<3?1.2:4)*.04);
  }
};
} // namespace fr3_esn128
