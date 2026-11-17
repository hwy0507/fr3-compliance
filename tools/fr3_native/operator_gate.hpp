#pragma once
#include "native_hold.hpp"
#include <string>

namespace fr3_native {
enum class Trial {HoldOnly,LightPush,FreeGuiding};
// Non-realtime supervisor. STOP is latched and cannot be reset in this process.
// A Ready observation never authorizes contact by itself.
class OperatorGate {
 public:
  OperatorGate(Trial trial,Session& session,std::atomic<bool>& cancel,bool operator_ended=false)
    :trial_(trial),session_(session),cancel_(cancel),operator_ended_(operator_ended) {}
  void stop() noexcept {cancel_.store(true);session_.contact_enabled.store(false);}
  bool permit_contact(bool heartbeat_fresh) noexcept {
    if(!heartbeat_fresh||trial_==Trial::HoldOnly||cancel_.load()||!session_.active.load()||
       session_.phase.load()!=int(Phase::Ready))return false;
    session_.contact_enabled.store(true);
    if(cancel_.load()){session_.contact_enabled.store(false);return false;}
    return true;
  }
  std::string status(bool heartbeat_fresh) const {
    if(cancel_.load())return "STOP_REQUESTED: contact forbidden; awaiting control exit";
    if(!session_.active.load())return "NOT_ACTIVE: contact forbidden";
    if(!heartbeat_fresh)return "NOT_READY: state update stale; contact forbidden";
    switch(static_cast<Phase>(session_.phase.load())) {
      case Phase::Settling:return "SETTLING: do not touch";
      case Phase::Ready:
        if(trial_==Trial::HoldOnly)return "READY_HOLD: stable; contact is NOT authorized";
        if(trial_==Trial::FreeGuiding)return session_.contact_enabled.load()?
          (operator_ended_?"READY_DRAG: gentle horizontal guiding, max 15mm from start; operator-ended":
                          "READY_DRAG: gentle horizontal guiding, max 15mm from start; release within 2s"):
          "READY_TO_DRAG: do not touch; type DRAG to enable guiding";
        return session_.contact_enabled.load()?"READY_PUSH: gentle reviewed horizontal contact permitted":
          "READY_TO_ARM: do not touch; type PUSH to enable reviewed contact";
      case Phase::Contact:return operator_ended_?"CONTACT: slow guiding within the reviewed bounds; STOP to end":"CONTACT: release promptly";
      case Phase::Recovering:return "RECOVERING: release and keep clear";
      case Phase::Finished:return "FINISHED: contact forbidden";
      case Phase::Stopped:return "STOPPED: contact forbidden";
    }
    return "NOT_READY";
  }
 private:
  Trial trial_;Session& session_;std::atomic<bool>& cancel_;
  bool operator_ended_=false;
};
} // namespace fr3_native
