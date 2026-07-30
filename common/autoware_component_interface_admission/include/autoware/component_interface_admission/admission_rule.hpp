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
  // The three codes below are deploy-time only.
  //
  // QOS_PIVOT_PROVIDER and QOS_PIVOT_CONSUMER are per-ENDPOINT verdicts, reported independently of
  // stage-1 pairing by collect_pivot_endpoint_verdicts(): whenever a pivot is registered for an
  // interface (autoware_component_interface_specs' interface_manifest.json) every `provided` entry
  // with qos for it must offer at least the pivot, and every `required` entry with qos for it must
  // request at most the pivot -- regardless of whether a counterparty for that specific pairing is
  // present in this deploy set at all. A publisher-only image is exactly as checkable as a full
  // pair; that is the whole point of a pivot. Such a row names only the one side it is about (see
  // AdmissionResult below).
  //
  // The provider's offered QoS ranks below the registered pivot (empty consumer_node).
  QOS_PIVOT_PROVIDER = 5,
  // The consumer's requested QoS ranks above the registered pivot (empty provider_node).
  QOS_PIVOT_CONSUMER = 6,
  // QOS_PAIR_INCOMPATIBLE is a per-PAIR verdict instead, reported by apply_no_pivot_qos_verdict()
  // only when NO pivot is registered for the interface (e.g. a vendor / unversioned interface) AND
  // both sides of a stage-1-matched pairing carry qos: the provider's offered QoS and the
  // consumer's requested QoS are then checked directly against each other.
  QOS_PAIR_INCOMPATIBLE = 7,
};

// One verdict row. For a version verdict (ACCEPTED / MAJOR_MISMATCH / MINOR_MISMATCH /
// TOPIC_MISMATCH / NO_PROVIDER) and for QOS_PAIR_INCOMPATIBLE, both consumer_node and provider_node
// are set -- it is a resolved consumer <- provider pairing. A per-endpoint pivot verdict
// (QOS_PIVOT_PROVIDER / QOS_PIVOT_CONSUMER) is about exactly one side and leaves the other node
// field empty; see collect_pivot_endpoint_verdicts().
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
// this header). Change one and the other must follow: autoware_component_interface_specs,
// qos_compatibility.hpp.
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
// This is a PER-ENDPOINT contract, not a per-pair one: it is checked independently for every
// `provided` entry and every `required` entry that carries qos, by
// collect_pivot_endpoint_verdicts() below, regardless of whether that specific endpoint has a
// matched counterpart in this deploy set at all.
//
// DDS-style pairwise QoS fallback for interfaces with NO registered pivot (e.g. a vendor /
// unversioned interface): there is nothing to check each endpoint against in isolation, so the gate
// instead falls back to a direct offered-vs-requested compatibility check on the ONE
// stage-1-matched pair. Layers that verdict on top of `res`, which must already hold an ACCEPTED
// (or, for an unversioned entry, a provisionally-ACCEPTED) version verdict for the given
// provider/required pairing. When a pivot IS registered for this interface, this function is a
// no-op: the independent per-endpoint checks below are what runs instead. Also a no-op when either
// side of the pairing does not carry QoS at all — a provider or a required entry without `qos` in
// its manifest is not something this package can evaluate, so no QOS_* verdict is produced for it.
inline void apply_no_pivot_qos_verdict(
  AdmissionResult & res, const ProvidedInterface & provider, const RequiredInterface & required,
  const std::map<std::string, QosRecord> & spec_pivots)
{
  if (!provider.has_qos || !required.has_qos) {
    return;
  }
  if (spec_pivots.find(required.interface_name) != spec_pivots.end()) {
    return;  // a pivot is registered for this interface; see collect_pivot_endpoint_verdicts().
  }
  if (!qos_is_at_least(provider.qos, required.qos)) {
    res.code = QOS_PAIR_INCOMPATIBLE;
  }
}

// The per-endpoint half of the pivot rule (see apply_no_pivot_qos_verdict() above for the no-pivot
// fallback). Runs entirely independently of stage-1 pairing: it walks every `provided` and every
// `required` entry across the whole deploy set directly, not the provider/consumer matches computed
// elsewhere in evaluate_deploy(). A provider with no consumer in the set (a publisher-only image)
// and a consumer with no provider in the set are each exactly as checkable as a matched pair — that
// is the whole point of a pivot being a per-endpoint contract. A provider-side violation and a
// consumer-side violation for the same interface are reported as two separate rows, each naming
// only the one side it is about (empty consumer_node / empty provider_node respectively). `depth`
// is never inspected here, matching qos_is_at_least().
inline void collect_pivot_endpoint_verdicts(
  const std::vector<InterfaceManifest> & manifests,
  const std::map<std::string, QosRecord> & spec_pivots, std::vector<AdmissionResult> & results)
{
  for (const auto & m : manifests) {
    for (const auto & p : m.provided) {
      if (!p.has_qos) {
        continue;
      }
      const auto pivot_it = spec_pivots.find(p.interface_name);
      if (pivot_it == spec_pivots.end()) {
        continue;  // no pivot registered for this interface: see apply_no_pivot_qos_verdict().
      }
      if (!qos_is_at_least(p.qos, pivot_it->second)) {
        AdmissionResult res;
        res.provider_node = m.node_name;
        res.interface_name = p.interface_name;
        res.code = QOS_PIVOT_PROVIDER;
        results.push_back(res);
      }
    }
    for (const auto & r : m.required) {
      if (!r.has_qos) {
        continue;
      }
      const auto pivot_it = spec_pivots.find(r.interface_name);
      if (pivot_it == spec_pivots.end()) {
        continue;
      }
      if (!qos_is_at_least(pivot_it->second, r.qos)) {
        AdmissionResult res;
        res.consumer_node = m.node_name;
        res.interface_name = r.interface_name;
        res.code = QOS_PIVOT_CONSUMER;
        results.push_back(res);
      }
    }
  }
}

// Runtime admission (the shared rule at its runtime trigger). For each required interface, find a
// provider of the same interface_name and apply the two-layer match:
//   - version-ok AND resolved_name coincide      -> ACCEPTED (the actually-wired provider)
//   - version-ok but resolved_name disjoint       -> TOPIC_MISMATCH (remap false-accept caught)
//   - MAJOR in range but min_minor unmet           -> MINOR_MISMATCH
//   - otherwise                                    -> MAJOR_MISMATCH
// A required interface with no version-checkable provider is SKIPPED (not reported): under the
// runtime trigger a provider may simply not have started yet, so absence is not yet a failure. This
// applies whether there is literally no provider of the interface_name at all, or only unversioned
// ones (has_version == false) -- see the has_version handling below.
//
// has_version == false, on either side, means "makes no version claim" and must never enter a
// version verdict (I1 / the ported package's stated contract): a required entry with no version at
// all is resolved by interface_name alone, ignoring every provider's has_version and version fields
// entirely, and always ACCEPTED once any provider is found (there is nothing left to check). A
// required entry that DOES declare a version treats an unversioned provider as invisible for this
// purpose -- it is excluded from both the "accepted" and the "blamed" candidate pools, exactly like
// it does not exist, rather than being silently compared against its all-zero default version.
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

      if (!r.has_version) {
        // No version claim: resolve any provider by interface_name alone, ignoring its own
        // has_version and resolved_name, for consistency with evaluate_deploy()'s treatment of the
        // same v2 document. Version bounds simply do not apply, so there is nothing left to reject.
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
        results.push_back(res);
        continue;
      }

      // Only version-checkable providers are candidates: an unversioned provider makes no version
      // claim and must never enter a version verdict, on either the accepted or the blamed side.
      std::vector<const ProviderEntry *> candidates;
      for (const auto & entry : it->second) {
        if (entry.p.has_version) {
          candidates.push_back(&entry);
        }
      }
      if (candidates.empty()) {
        continue;  // only unversioned providers observed; treat like "not started yet"
      }

      const ProviderEntry * wired = nullptr;             // version-ok AND same resolved wire topic
      const ProviderEntry * version_ok_other = nullptr;  // version-ok but disjoint wire topic
      for (const auto * entry : candidates) {
        if (version_compatible(r, entry->p)) {
          if (entry->p.resolved_name == r.resolved_name) {
            wired = entry;
            break;
          }
          // Lowest node name wins, so the reported provider does not depend on the order the
          // manifests were passed to manifest_admit.
          if (version_ok_other == nullptr || entry->node < version_ok_other->node) {
            version_ok_other = entry;
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
        const ProviderEntry * blame = candidates.front();
        for (const auto * entry : candidates) {
          if (rank(*entry) < rank(*blame)) {
            blame = entry;
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
// autoware_component_interface_specs' interface_manifest.json. The QoS gate is layered on in two
// independent parts, run once each over the whole deploy set: collect_pivot_endpoint_verdicts()
// checks every provided/required entry that carries qos against a registered pivot, per endpoint,
// regardless of pairing (QOS_PIVOT_PROVIDER / QOS_PIVOT_CONSUMER); apply_no_pivot_qos_verdict()
// checks the one stage-1-matched pair directly, but only for an interface with NO registered pivot
// (QOS_PAIR_INCOMPATIBLE). See both functions' comments.
//
// has_version == false, on either side, means "makes no version claim" and must never enter a
// version verdict (I1). A required entry with no version at all never produces MAJOR_MISMATCH /
// MINOR_MISMATCH: a provider is still resolved by interface_name alone, ignoring every candidate's
// own has_version, purely so the no-pivot QoS fallback above has a pairing to check (lowest node
// name wins, for determinism). A required entry that DOES declare a version excludes unversioned
// providers from candidacy entirely -- they are invisible for version-verdict purposes, neither
// accepted nor blamed, rather than being silently compared against their all-zero default version.
// NO_PROVIDER, by contrast, is a completeness verdict, not a version verdict: because the deploy
// image set is complete, it fires whenever no version-checkable provider exists for a required
// entry ANYWHERE in the set, whether that entry declares a version or not, and whether the reason
// is that literally no provider of the interface_name exists or that every provider of it is itself
// unversioned. This mirrors the pre-v2 behavior, where every required entry got this check.
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
          // Completeness verdict: NO_PROVIDER fires regardless of has_version (I3).
          AdmissionResult res;
          res.consumer_node = m.node_name;
          res.interface_name = r.interface_name;
          res.code = NO_PROVIDER;
          results.push_back(res);
          continue;
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
        apply_no_pivot_qos_verdict(res, matched->p, r, spec_pivots);
        results.push_back(res);
        continue;
      }

      AdmissionResult res;
      res.consumer_node = m.node_name;
      res.interface_name = r.interface_name;

      // Only version-checkable providers (has_version == true) are candidates; an unversioned
      // provider makes no version claim and must never enter a version verdict (I1).
      std::vector<const ProviderEntry *> candidates;
      if (it != providers.end()) {
        for (const auto & entry : it->second) {
          if (entry.p.has_version) {
            candidates.push_back(&entry);
          }
        }
      }

      if (candidates.empty()) {
        // Complete-set semantics: no version-checkable provider anywhere is a hard reject, whether
        // because no provider of the interface_name exists at all or because every provider of it
        // declined to state a version.
        res.code = NO_PROVIDER;
        results.push_back(res);
        continue;
      }

      // Stage 1 only: accept if ANY candidate is version-compatible. resolved_name (stage 2) is not
      // statically visible at deploy time, so it is never inspected here.
      const ProviderEntry * accepted = nullptr;
      for (const auto * entry : candidates) {
        if (version_compatible(r, entry->p)) {
          // Lowest node name wins, so the reported provider is independent of manifest order.
          if (accepted == nullptr || entry->node < accepted->node) {
            accepted = entry;
          }
        }
      }

      if (accepted != nullptr) {
        res.code = ACCEPTED;
        res.provider_node = accepted->node;
        apply_no_pivot_qos_verdict(res, accepted->p, r, spec_pivots);
      } else {
        // No candidate satisfied the version bounds. Blame by a stable total order (not manifest
        // order): a provider whose MAJOR is already in range (the actionable MINOR_MISMATCH) first,
        // then the lowest node name.
        const auto rank = [&r](const ProviderEntry & e) {
          return std::make_tuple(!major_in_range(r, e.p), e.node);
        };
        const ProviderEntry * blame = candidates.front();
        for (const auto * entry : candidates) {
          if (rank(*entry) < rank(*blame)) {
            blame = entry;
          }
        }
        res.provider_node = blame->node;
        res.code = major_in_range(r, blame->p) ? MINOR_MISMATCH : MAJOR_MISMATCH;
      }
      results.push_back(res);
    }
  }

  // Per-endpoint pivot conformance: independent of the pairing loop above (see
  // collect_pivot_endpoint_verdicts()'s comment).
  collect_pivot_endpoint_verdicts(manifests, spec_pivots, results);

  return results;
}

// Version-only overload, kept for existing callers: equivalent to evaluate_deploy(manifests, {}),
// i.e. no spec pivot data available, so every interface falls into the no-pivot-row path above.
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
