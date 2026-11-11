#include "kitezh/shift.hpp"
#include "kitezh/lock.hpp"
#include "kitezh/geometry.hpp"
#include "kitezh/vessel.hpp"
#include <algorithm>
#include <set>

namespace kitezh {

ShiftReport runNightShift(const std::vector<Vessel>& queue) {
    ShiftReport report;
    std::vector<Vessel> valid_queue;
    std::set<std::string> seen_call_signs;

    for (const auto& vessel : queue) {
        if (seen_call_signs.count(vessel.call_sign)) {
            report.refused.push_back({vessel.call_sign, "duplicate"});
            continue;
        }
        seen_call_signs.insert(vessel.call_sign);

        auto error = validateVessel(vessel);
        if (error.has_value()) {
            report.refused.push_back({vessel.call_sign, "invalid"});
            continue;
        }

        if (vessel.draft_dm > kLowDepthDm) {
            report.refused.push_back({vessel.call_sign, "draft"});
            continue;
        }

        valid_queue.push_back(vessel);
    }

    Lock lock;

    while (!valid_queue.empty()) {
        std::vector<Vessel> batch;
        Direction batch_dir = valid_queue.front().direction;
        bool has_normal_passed = false;

        for (size_t i = 0; i < valid_queue.size(); ++i) {
            const auto& vessel = valid_queue[i];
            
            if (vessel.direction != batch_dir) {
                continue;
            }

            bool is_oversized = (vessel.kind == VesselKind::Oversized);
            bool is_ambulance = (vessel.kind == VesselKind::Ambulance);
            
            if (is_oversized) {
                if (batch.empty()) {
                    batch.push_back(vessel);
                    valid_queue.erase(valid_queue.begin() + i);
                    break;
                } else {
                    continue;
                }
            }

            if (has_normal_passed && !is_ambulance) {
                continue;
            }

            if (fitsInChamber(batch, vessel)) {
                batch.push_back(vessel);
                valid_queue.erase(valid_queue.begin() + i);
                --i;
                if (!is_ambulance) {
                    has_normal_passed = true;
                }
            }
        }

        if (batch.empty()) {
            valid_queue.erase(valid_queue.begin());
            continue;
        }

        Level required_level = (batch_dir == Direction::Upstream) ? Level::Low : Level::High;
        Gate entry_gate = (batch_dir == Direction::Upstream) ? Gate::Lower : Gate::Upper;
        Gate exit_gate = (batch_dir == Direction::Upstream) ? Gate::Upper : Gate::Lower;

        // Качаем воду перед заходом судов ТОЛЬКО если текущий уровень не совпадает!
        if (lock.level() != required_level) {
            if (lock.gate(Gate::Lower) == GateState::Open) lock.close(Gate::Lower);
            if (lock.gate(Gate::Upper) == GateState::Open) lock.close(Gate::Upper);
            lock.pump();
        }

        lock.open(entry_gate);

        for (const auto& vessel : batch) {
            lock.enter(vessel);
        }

        lock.close(entry_gate);
        lock.pump();
        lock.open(exit_gate);

        for (const auto& vessel : batch) {
            lock.exit(vessel.call_sign);
            report.passed.push_back(vessel.call_sign);
        }
        lock.close(exit_gate);
    }

    report.pump_cycles = lock.pumpCycles();
    return report;
}

} // namespace kitezh
