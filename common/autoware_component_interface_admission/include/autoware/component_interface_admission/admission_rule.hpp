// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__COMPONENT_INTERFACE_ADMISSION__ADMISSION_RULE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_ADMISSION__ADMISSION_RULE_HPP_

#include "autoware/component_interface_admission/records.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace autoware::component_interface_admission
{

// Admission verdict codes. An enum-backed uint16_t constant set whose numeric values are chosen to
// match the eventual rosidl message binding, so that the future message mapping is mechanical.
//
// This code is designed so it can later be raised as a distinct error code in Autoware's
// system-wide diagnostics, under a reserved namespace for interface-compatibility errors, reusing
// the same code space, off-wire name derivation, and minimal-risk-maneuver (MRM) wiring as any
// other Autoware error. accepted == (code == ACCEPTED); the human-readable reason is derivable
// from the code.
enum Verdict : std::uint16_t {
  ACCEPTED = 0,
  MAJOR_MISMATCH = 1,
  MINOR_MISMATCH = 2,
  // version-compatible, but a remap left provider / consumer on disjoint wire topics
  TOPIC_MISMATCH = 3,
  // deploy-time only: a required interface has no provider in the composed set. The runtime
  // observe-mode evaluate() never emits this — a provider may simply not have started yet.
  NO_PROVIDER = 4,
  // The three codes below are deploy-time only, layered on top of an otherwise-ACCEPTED version
  // verdict by evaluate_deploy(manifests, spec_pivots) when both sides of a pairing carry QoS
  // (ProvidedInterface::has_qos / RequiredInterface::has_qos). See apply_qos_verdict().
  //
  // A pivot is registered for this interface (autoware_component_interface_specs'
  // interface_manifest.json) and the provider's offered QoS ranks below it.
  QOS_PIVOT_PROVIDER = 5,
  // A pivot is registered for this interface and the consumer's requested QoS ranks above it.
  QOS_PIVOT_CONSUMER = 6,
  // No pivot is registered for this interface (e.g. a vendor / unversioned interface); the
  // provider's offered QoS and the consumer's requested QoS are directly incompatible.
  QOS_PAIR_INCOMPATIBLE = 7,
};

// One consumer <- provider interface pairing verdict.
struct AdmissionResult
{
  std::string consumer_node;
  std::string interface_name;
  std::string provider_node;
  std::uint16_t code{ACCEPTED};
};

// Whether the provider's MAJOR falls inside the consumer's accepted MAJOR window
// [accept_major_min, accept_major_max]. This window is the primary compatibility gate; a MAJOR
// outside it is a hard MAJOR_MISMATCH, and it is what separates a MINOR_MISMATCH (MAJOR in range,
// version bound unmet) from a MAJOR_MISMATCH in the blame path.
inline bool major_in_range(const RequiredInterface & r, const ProvidedInterface & p)
{
  return r.accept_major_min <= p.major && p.major <= r.accept_major_max;
}

// Whether the provider satisfies the consumer's full version contract: its MAJOR is in the accepted
// window AND the optional min_minor lower bound is met. Per semver, MINOR resets to 0 on every
// MAJOR bump, so min_minor binds ONLY at the MAJOR it was declared against (accept_major_min); at
// any higher accepted MAJOR the bound is already satisfied. min_minor == 0 means unconstrained; the
// bound is inclusive (provider MINOR >= min_minor).
inline bool version_compatible(const RequiredInterface & r, const ProvidedInterface & p)
{
  if (!major_in_range(r, p)) {
    return false;
  }
  return r.min_minor == 0 || p.major > r.accept_major_min || p.minor >= r.min_minor;
}

// Strength rank of a reliability policy string. RELIABLE delivers everything BEST_EFFORT does and
// more, so it ranks higher. A policy string outside the two the QosRecord vocabulary allows ranks
// -1: incomparable, so every comparison involving it fails (fail closed).
//
// autoware_component_interface_specs' qos_compatibility.hpp expresses this exact rank over RMW
// enums (rmw_qos_reliability_policy_t / rmw_qos_durability_policy_t). The admission package
// restates these ranks on the JSON string encoding (it is a no-dependency leaf that cannot include
// this header). Change one and the other must follow: autoware_component_interface_admission,
// admission_rule.hpp.
inline int reliability_rank(const std::string & policy)
{
  if (policy == "best_effort") {
    return 0;
  }
  if (policy == "reliable") {
    return 1;
  }
  return -1;
}

// Strength rank of a durability policy string. TRANSIENT_LOCAL delivers everything VOLATILE does
// and more, so it ranks higher. See reliability_rank() for the fail-closed / dual-maintenance note
// (it applies identically here).
inline int durability_rank(const std::string & policy)
{
  if (policy == "volatile") {
    return 0;
  }
  if (policy == "transient_local") {
    return 1;
  }
  return -1;
}

inline bool reliability_at_least(const std::string & a, const std::string & b)
{
  return reliability_rank(a) >= 0 && reliability_rank(b) >= 0 &&
         reliability_rank(a) >= reliability_rank(b);
}

inline bool durability_at_least(const std::string & a, const std::string & b)
{
  return durability_rank(a) >= 0 && durability_rank(b) >= 0 &&
         durability_rank(a) >= durability_rank(b);
}

// DDS request-vs-offered compatibility: a connection forms iff the offered QoS is at least as
// strong as the requested QoS on every axis. depth is endpoint-local and presentational only (see
// QosRecord in records.hpp); it is deliberately never inspected here or anywhere in this file.
inline bool qos_is_at_least(const QosRecord & offered, const QosRecord & requested)
{
  return reliability_at_least(offered.reliability, requested.reliability) &&
         durability_at_least(offered.durability, requested.durability);
}

// The pivot rule: a spec's declared QoS is a pivot, not an exact-match requirement. A provider must
// offer at least the pivot and a consumer must request at most the pivot; transitivity of the QoS
// partial order then guarantees every conforming pair connects (offered >= pivot >= requested).
//
// Layers a QoS verdict on top of `res`, which must already hold an ACCEPTED (or, for an unversioned
// entry, a provisionally-ACCEPTED) version verdict for the given provider/required pairing. Leaves
// `res` untouched (still ACCEPTED) when either side of the pairing does not carry QoS at all — a
// provider or a required entry without `qos` in its manifest is not something this package can
// evaluate, so no QOS_* verdict is produced for it.
inline void apply_qos_verdict(
  AdmissionResult & res, const ProvidedInterface & provider, const RequiredInterface & required,
  const std::map<std::string, QosRecord> & spec_pivots)
{
  if (!provider.has_qos || !required.has_qos) {
    return;
  }

  const auto pivot_it = spec_pivots.find(required.interface_name);
  if (pivot_it != spec_pivots.end()) {
    const auto & pivot = pivot_it->second;
    if (!qos_is_at_least(provider.qos, pivot)) {
      res.code = QOS_PIVOT_PROVIDER;
      return;
    }
    if (!qos_is_at_least(pivot, required.qos)) {
      res.code = QOS_PIVOT_CONSUMER;
    }
    return;
  }

  // No pivot registered for this interface (e.g. a vendor / unversioned interface): fall back to a
  // direct offered-vs-requested compatibility check.
  if (!qos_is_at_least(provider.qos, required.qos)) {
    res.code = QOS_PAIR_INCOMPATIBLE;
  }
}

// Runtime admission (the shared rule at its runtime trigger). For each required interface, find a
// provider of the same interface_name and apply the two-layer match:
//   - version-ok AND resolved_name coincide      -> ACCEPTED (the actually-wired provider)
//   - version-ok but resolved_name disjoint       -> TOPIC_MISMATCH (remap false-accept caught)
//   - MAJOR in range but min_minor unmet           -> MINOR_MISMATCH
//   - otherwise                                    -> MAJOR_MISMATCH
// A required interface with no provider is SKIPPED (not reported): under the runtime trigger the
// provider may simply not have started yet, so absence is not yet a failure.
inline std::vector<AdmissionResult> evaluate(const std::vector<InterfaceManifest> & manifests)
{
  struct ProviderEntry
  {
    std::string node;
    ProvidedInterface p;
  };
  std::unordered_map<std::string, std::vector<ProviderEntry>> providers;
  for (const auto & m : manifests) {
    for (const auto & p : m.provided) {
      providers[p.interface_name].push_back({m.node_name, p});
    }
  }

  std::vector<AdmissionResult> results;
  for (const auto & m : manifests) {
    for (const auto & r : m.required) {
      const auto it = providers.find(r.interface_name);
      if (it == providers.end() || it->second.empty()) {
        continue;  // no provider yet — nothing to admit
      }

      const ProviderEntry * wired = nullptr;             // version-ok AND same resolved wire topic
      const ProviderEntry * version_ok_other = nullptr;  // version-ok but disjoint wire topic
      for (const auto & entry : it->second) {
        if (version_compatible(r, entry.p)) {
          if (entry.p.resolved_name == r.resolved_name) {
            wired = &entry;
            break;
          }
          // Lowest node name wins, so the reported provider does not depend on the order the
          // manifests were passed to manifest_admit.
          if (version_ok_other == nullptr || entry.node < version_ok_other->node) {
            version_ok_other = &entry;
          }
        }
      }

      AdmissionResult res;
      res.consumer_node = m.node_name;
      res.interface_name = r.interface_name;
      if (wired != nullptr) {
        res.code = ACCEPTED;
        res.provider_node = wired->node;
      } else if (version_ok_other != nullptr) {
        // version-compatible, but a remap left the wire topics disjoint — the false-accept that
        // logical-name-only admission (matching on Spec::name alone) would have missed.
        res.code = TOPIC_MISMATCH;
        res.provider_node = version_ok_other->node;
      } else {
        // No provider satisfied both bounds. Blame the one the operator can act on, by a stable
        // total order rather than registration order: the provider on the consumer's wire topic
        // first, then one whose MAJOR is already in range (the actionable MINOR_MISMATCH), then
        // the lowest node name. Manifest/argv order must not change the verdict.
        const auto rank = [&r](const ProviderEntry & e) {
          const bool on_wire = e.p.resolved_name == r.resolved_name;
          return std::make_tuple(!on_wire, !major_in_range(r, e.p), e.node);
        };
        const ProviderEntry * blame = &it->second.front();
        for (const auto & entry : it->second) {
          if (rank(entry) < rank(*blame)) {
            blame = &entry;
          }
        }
        res.provider_node = blame->node;
        res.code = major_in_range(r, blame->p) ? MINOR_MISMATCH : MAJOR_MISMATCH;
      }
      results.push_back(res);
    }
  }
  return results;
}

// Deploy-time admission (the same shared rule at its deploy-time trigger, restricted to stage 1).
// The deploy gate reads each component's manifest from static image metadata, where remaps — which
// live in the launch / compose layer — are NOT visible, so the resolved_name match (stage 2 of the
// rule) is not statically decidable and is deferred to the runtime trigger. Deploy
// therefore pairs a consumer with a provider on interface_name + version compatibility ONLY and
// never emits TOPIC_MISMATCH — that residual remap false-accept is what the runtime trigger
// backstops. Because the deploy image set is complete (unlike the runtime observe mode, where a
// provider may simply not have started yet), a required interface with NO provider anywhere in the
// set is reported as NO_PROVIDER.
//
// `spec_pivots` is the interface_name -> QoS map parsed by spec_pivots_from_json() from
// autoware_component_interface_specs' interface_manifest.json. When a pairing that is otherwise
// version-ACCEPTED has QoS on both sides, apply_qos_verdict() layers QOS_PIVOT_PROVIDER /
// QOS_PIVOT_CONSUMER / QOS_PAIR_INCOMPATIBLE on top of it; see that function's comment.
//
// A required entry with has_version == false (a v2 manifest declaring no version at all) never
// produces a version verdict (MAJOR_MISMATCH / MINOR_MISMATCH / NO_PROVIDER): version bounds simply
// do not apply to it. A provider is still resolved by interface_name alone, purely so the QoS layer
// above has something to check (lowest node name wins, for determinism); if no provider exists at
// all, the entry has nothing to admit and is silently skipped (no row is reported for it, mirroring
// how the runtime trigger skips a not-yet-started provider rather than rejecting it).
inline std::vector<AdmissionResult> evaluate_deploy(
  const std::vector<InterfaceManifest> & manifests,
  const std::map<std::string, QosRecord> & spec_pivots)
{
  struct ProviderEntry
  {
    std::string node;
    ProvidedInterface p;
  };
  std::unordered_map<std::string, std::vector<ProviderEntry>> providers;
  for (const auto & m : manifests) {
    for (const auto & p : m.provided) {
      providers[p.interface_name].push_back({m.node_name, p});
    }
  }

  std::vector<AdmissionResult> results;
  for (const auto & m : manifests) {
    for (const auto & r : m.required) {
      const auto it = providers.find(r.interface_name);

      if (!r.has_version) {
        if (it == providers.end() || it->second.empty()) {
          continue;  // nothing to admit against; not a NO_PROVIDER, just skipped
        }
        const ProviderEntry * matched = &it->second.front();
        for (const auto & entry : it->second) {
          if (entry.node < matched->node) {
            matched = &entry;
          }
        }
        AdmissionResult res;
        res.consumer_node = m.node_name;
        res.interface_name = r.interface_name;
        res.provider_node = matched->node;
        res.code = ACCEPTED;
        apply_qos_verdict(res, matched->p, r, spec_pivots);
        results.push_back(res);
        continue;
      }

      AdmissionResult res;
      res.consumer_node = m.node_name;
      res.interface_name = r.interface_name;

      if (it == providers.end() || it->second.empty()) {
        // Complete-set semantics: a required interface with no provider anywhere is a hard reject.
        res.code = NO_PROVIDER;
        results.push_back(res);
        continue;
      }

      // Stage 1 only: accept if ANY provider of this interface is version-compatible. resolved_name
      // (stage 2) is not statically visible at deploy time, so it is never inspected here.
      const ProviderEntry * accepted = nullptr;
      for (const auto & entry : it->second) {
        if (version_compatible(r, entry.p)) {
          // Lowest node name wins, so the reported provider is independent of manifest order.
          if (accepted == nullptr || entry.node < accepted->node) {
            accepted = &entry;
          }
        }
      }

      if (accepted != nullptr) {
        res.code = ACCEPTED;
        res.provider_node = accepted->node;
        apply_qos_verdict(res, accepted->p, r, spec_pivots);
      } else {
        // No provider satisfied the version bounds. Blame by a stable total order (not manifest
        // order): a provider whose MAJOR is already in range (the actionable MINOR_MISMATCH) first,
        // then the lowest node name.
        const auto rank = [&r](const ProviderEntry & e) {
          return std::make_tuple(!major_in_range(r, e.p), e.node);
        };
        const ProviderEntry * blame = &it->second.front();
        for (const auto & entry : it->second) {
          if (rank(entry) < rank(*blame)) {
            blame = &entry;
          }
        }
        res.provider_node = blame->node;
        res.code = major_in_range(r, blame->p) ? MINOR_MISMATCH : MAJOR_MISMATCH;
      }
      results.push_back(res);
    }
  }
  return results;
}

// Version-only overload, kept for existing callers: equivalent to evaluate_deploy(manifests, {}),
// i.e. no spec pivot data available, so any otherwise-ACCEPTED pairing that carries QoS on both
// sides falls back to the no-pivot-row direct compatibility check in apply_qos_verdict().
inline std::vector<AdmissionResult> evaluate_deploy(
  const std::vector<InterfaceManifest> & manifests)
{
  return evaluate_deploy(manifests, {});
}

// The human-readable reason is derived from the verdict code off-wire — it is not carried on
// AdmissionResult (the code is the single source of identity, matching the future error-code
// reuse described above, not an existing Autoware-wide mechanism today). Covers both the runtime
// and the deploy-only (NO_PROVIDER) codes.
inline const char * verdict_text(std::uint16_t code)
{
  switch (code) {
    case ACCEPTED:
      return "accepted";
    case MAJOR_MISMATCH:
      return "MAJOR mismatch";
    case MINOR_MISMATCH:
      return "MINOR mismatch";
    case TOPIC_MISMATCH:
      return "resolved-topic mismatch (remap)";
    case NO_PROVIDER:
      return "required interface has no provider in the set";
    case QOS_PIVOT_PROVIDER:
      return "QOS pivot violation: provider offers below the registered pivot";
    case QOS_PIVOT_CONSUMER:
      return "QOS pivot violation: consumer requests above the registered pivot";
    case QOS_PAIR_INCOMPATIBLE:
      return "QOS incompatible: no pivot registered and offered/requested QoS do not match";
    default:
      return "unknown";
  }
}

inline bool any_rejected(const std::vector<AdmissionResult> & results)
{
  for (const auto & r : results) {
    if (r.code != ACCEPTED) {
      return true;
    }
  }
  return false;
}

}  // namespace autoware::component_interface_admission

#endif  // AUTOWARE__COMPONENT_INTERFACE_ADMISSION__ADMISSION_RULE_HPP_
