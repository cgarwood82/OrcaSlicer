#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"

namespace Slic3r {

// Returns the lowest 0-based logical filament index L such that pem[L] == physical.
// Returns -1 if no filament routes to `physical`.
// Degenerate case: an empty pem returns 0 when `physical == 0` (identity-on-head-0
// fallback for printers that never configured a pem) and -1 otherwise.
// With identity pem, the caller sees logical == physical behavior because
// position L holds value L.
int first_filament_for_physical_head(const ConfigOptionInts& pem, int physical);

// Returns true if the printer has at least one physical head fed by >= 2 logical
// filaments (i.e., an MMU/AFC is present on some head). Drives the plater's
// click-handler branch: true -> open popover; false -> retain cycle-through.
bool has_mmu(const ConfigOptionInts& pem);

// Returns true if any physical head OTHER than `primary_physical` is fed by
// >= 2 logical filaments. Primary is the head whose filament is already
// controlled by the left sidebar's object->filament assignment, so an MMU
// there needs no popover row; only non-primary MMU heads do.
// Returns false when pem is empty or size < 2.
bool has_non_primary_mmu(const ConfigOptionInts& pem, int primary_physical);

// Parses one mode's entry from `imex_mode_active_tools` and returns the
// 0-based physical T-index carrying the `:P` (Primary) role marker.
// Accepts two forms:
//   "0:P,1:C,2:C"   (explicit role suffix)
//   "0"             (backwards-compat: a bare index == Primary)
// Returns -1 on empty/malformed input or if no primary is found.
// The caller is responsible for indexing into imex_mode_active_tools by mode.
int imex_primary_tool_for_mode(const std::string& active_tools_for_mode);

// Parses the "phys:slot,phys:slot" serialization of imex_head_filament_map.
// Keys are physical T-indices (0-based); values are 1-based filament slots.
// Returns empty map on empty/malformed input.
std::map<int,int> parse_imex_head_filament_map(const std::string& s);

// Resolves which 0-based logical filament to use for a physical head.
// 1. If the plate map overrides `physical`, returns (slot - 1) — 1-based → 0-based.
// 2. Else falls back to first_filament_for_physical_head(pem, physical).
// Returns -1 if neither source yields a valid filament.
int resolve_filament_for_head(const std::map<int,int>& plate_map,
                              const ConfigOptionInts&  pem,
                              int                       physical);

enum class ImexRole { Primary, Copy, Mirror };

// Parses `imex_mode_active_tools[mode]` into a list of (physical_head, role) pairs.
// Accepted token forms (comma-separated, whitespace-tolerant):
//   "phys"      — bare index, role defaults to Copy
//   "phys:P"    — Primary
//   "phys:C"    — Copy
//   "phys:M"    — Mirror
//   "phys:???"  — unknown role suffix, treated as Copy
// Malformed tokens (unparseable int, negative phys) are skipped.
// NOTE: the bare-token → Copy default differs from `imex_primary_tool_for_mode`,
// which separately scans for a Primary; callers needing the primary index should
// use that helper. This parser is for call sites that already have the primary
// in hand and need the full head/role list (e.g. ghost factory/updater).
std::vector<std::pair<int, ImexRole>> parse_imex_active_tools(const std::string& active_tools_for_mode);

// World-space transform composed as `head_xf * primary_instance_world` to place a ghost
// copy of the primary into `target`'s frame under `role`.
// `gantry_offset` = center_for(target) - center_for(primary) (XY, in mm).
// `primary_origin` = world-space translation of the primary instance (use its matrix's
//   translation column). Only consulted for Mirror; Copy/Primary ignore it.
// Copy:    pure translation by gantry_offset. Ghost tracks primary 1:1 during drag.
// Mirror:  translate by gantry_offset (Copy-style placement AND motion), then flip the
//          ghost's geometry about a plane through primary_origin whose normal is the
//          primary-row gantry axis (X for all current IMEX printers), NOT the gantry_offset
//          direction. This keeps off-row Mirror ghosts (e.g. T3 on a 2x2) reflected across
//          the same plane as on-row mirrors (e.g. T1), just placed at the off-row position.
//          Zero-length gantry_offset degenerates to identity.
// Primary: identity.
Transform3d imex_head_transform(int primary, int target, ImexRole role,
                                const Vec2d& gantry_offset,
                                const Vec3d& primary_origin = Vec3d::Zero());

} // namespace Slic3r
