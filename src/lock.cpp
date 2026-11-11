#include "kitezh/lock.hpp"
#include "kitezh/geometry.hpp"
#include "kitezh/vessel.hpp"
#include <stdexcept>
#include <algorithm>
#include <vector>

namespace kitezh {

Lock::Lock() 
    : water_level_(Level::Low)
    , lower_gate_(GateState::Closed)
    , upper_gate_(GateState::Closed)
    , pump_cycles_(0)
{}

Level Lock::level() const {
    return water_level_;
}

GateState Lock::gate(Gate g) const {
    return (g == Gate::Lower) ? lower_gate_ : upper_gate_;
}

int Lock::pumpCycles() const {
    return pump_cycles_;
}

std::vector<Vessel> Lock::inside() const {
    return vessels_inside_;
}

void Lock::open(Gate g) {
    if (g == Gate::Lower) {
        if (water_level_ == Level::High) {
            throw LockError("Cannot open lower gate when water is high");
        }
        lower_gate_ = GateState::Open;
    } else {
        if (water_level_ == Level::Low) {
            throw LockError("Cannot open upper gate when water is low");
        }
        upper_gate_ = GateState::Open;
    }
}

void Lock::close(Gate g) {
    if (g == Gate::Lower) {
        lower_gate_ = GateState::Closed;
    } else {
        upper_gate_ = GateState::Closed;
    }
}

void Lock::pump() {
    if (lower_gate_ == GateState::Open || upper_gate_ == GateState::Open) {
        throw LockError("Cannot pump water while gates are open");
    }

    if (water_level_ == Level::High) {
        for (const auto& vessel : vessels_inside_) {
            if (vessel.draft_dm > kLowDepthDm) {
                throw LockError("Vessel will ground if water level is lowered");
            }
        }
        water_level_ = Level::Low;
    } else {
        water_level_ = Level::High;
    }

    pump_cycles_++;
}

void Lock::enter(const Vessel& vessel) {
    auto validation_error = validateVessel(vessel);
    if (validation_error.has_value()) {
        throw LockError("Invalid vessel data");
    }

    if (vessel.direction == Direction::Upstream) {
        if (lower_gate_ != GateState::Open) {
            throw LockError("Lower gate must be open to enter");
        }
    } else {
        if (upper_gate_ != GateState::Open) {
            throw LockError("Upper gate must be open to enter");
        }
    }

    int current_depth = (water_level_ == Level::Low) ? kLowDepthDm : kHighDepthDm;
    if (vessel.draft_dm > current_depth) {
        throw LockError("Vessel draft is too deep for current water level");
    }

    if (!fitsInChamber(vessels_inside_, vessel)) {
        throw LockError("Not enough space in the chamber");
    }

    bool is_oversized = (vessel.kind == VesselKind::Oversized);
    if (is_oversized && !vessels_inside_.empty()) {
        throw LockError("Oversized vessel requires an empty chamber");
    }

    for (const auto& v : vessels_inside_) {
        if (v.kind == VesselKind::Oversized) {
            throw LockError("Cannot enter: an oversized vessel is already inside");
        }
    }

    vessels_inside_.push_back(vessel);
}

void Lock::exit(const std::string& call_sign) {
    if (vessels_inside_.empty()) {
        throw LockError("Chamber is empty");
    }

    const auto& front_vessel = vessels_inside_.front();
    if (front_vessel.call_sign != call_sign) {
        throw LockError("Only the front vessel can exit first");
    }

    if (front_vessel.direction == Direction::Upstream) {
        if (upper_gate_ != GateState::Open) {
            throw LockError("Upper gate must be open to exit upstream");
        }
    } else {
        if (lower_gate_ != GateState::Open) {
            throw LockError("Lower gate must be open to exit downstream");
        }
    }

    vessels_inside_.erase(vessels_inside_.begin());
}

} // namespace kitezh
