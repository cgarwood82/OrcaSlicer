#include <catch2/catch_all.hpp>

#include "libslic3r/IMEXHelpers.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

static ConfigOptionInts make_pem(std::vector<int> v) {
    ConfigOptionInts o;
    o.values = std::move(v);
    return o;
}

TEST_CASE("effective_physical_extruder_map — explicit wins", "[IMEX]") {
    auto explicit_pem = make_pem({0, 0, 0, 0, 1, 2, 3});
    auto pei          = make_pem({1, 2, 3, 4}); // would derive to {0,1,2,3}
    auto out = effective_physical_extruder_map(&explicit_pem, &pei);
    REQUIRE(out.values == std::vector<int>{0, 0, 0, 0, 1, 2, 3});
}

TEST_CASE("effective_physical_extruder_map — default pem falls back to pei derive", "[IMEX]") {
    auto default_pem = make_pem({0});           // size 1, the PrintConfig default
    auto pei         = make_pem({1, 2});        // 1-indexed IDEX
    auto out = effective_physical_extruder_map(&default_pem, &pei);
    REQUIRE(out.values == std::vector<int>{0, 1}); // 1-indexed → 0-indexed
}

TEST_CASE("effective_physical_extruder_map — null explicit, pei present", "[IMEX]") {
    auto pei = make_pem({1, 2, 3});
    auto out = effective_physical_extruder_map(nullptr, &pei);
    REQUIRE(out.values == std::vector<int>{0, 1, 2});
}

TEST_CASE("effective_physical_extruder_map — both absent yields empty", "[IMEX]") {
    auto out = effective_physical_extruder_map(nullptr, nullptr);
    REQUIRE(out.values.empty());
}

TEST_CASE("effective_physical_extruder_map — IDEX ghost-color regression guard", "[IMEX]") {
    // Printer with printer_extruder_id = [1, 2] and no explicit pem (default {0}).
    // Before the GUI fix, this scenario produced a black ghost on T1 because
    // first_filament_for_physical_head({0}, 1) == -1.
    auto default_pem = make_pem({0});
    auto pei         = make_pem({1, 2});
    auto pem         = effective_physical_extruder_map(&default_pem, &pei);
    REQUIRE(first_filament_for_physical_head(pem, 0) == 0);
    REQUIRE(first_filament_for_physical_head(pem, 1) == 1); // no longer -1
}

TEST_CASE("first_filament_for_physical_head — identity pem", "[IMEX]") {
    auto pem = make_pem({0, 1, 2, 3});
    REQUIRE(first_filament_for_physical_head(pem, 0) == 0);
    REQUIRE(first_filament_for_physical_head(pem, 1) == 1);
    REQUIRE(first_filament_for_physical_head(pem, 2) == 2);
    REQUIRE(first_filament_for_physical_head(pem, 3) == 3);
    REQUIRE(first_filament_for_physical_head(pem, 4) == -1);
}

TEST_CASE("first_filament_for_physical_head — AFC routing", "[IMEX]") {
    // User's IQEX: 4 AFC lanes on T0, direct extruders on T1, T2, T3
    auto pem = make_pem({0, 0, 0, 0, 1, 2, 3});
    REQUIRE(first_filament_for_physical_head(pem, 0) == 0); // first of T0's lanes
    REQUIRE(first_filament_for_physical_head(pem, 1) == 4);
    REQUIRE(first_filament_for_physical_head(pem, 2) == 5);
    REQUIRE(first_filament_for_physical_head(pem, 3) == 6);
    REQUIRE(first_filament_for_physical_head(pem, 5) == -1); // no head 5
}

TEST_CASE("first_filament_for_physical_head — empty pem", "[IMEX]") {
    ConfigOptionInts pem;
    REQUIRE(first_filament_for_physical_head(pem, 0) == 0);
    REQUIRE(first_filament_for_physical_head(pem, 1) == -1);
}

TEST_CASE("has_mmu — pure IDEX", "[IMEX]") {
    REQUIRE_FALSE(has_mmu(make_pem({0, 1})));
    REQUIRE_FALSE(has_mmu(make_pem({0, 1, 2, 3})));
}

TEST_CASE("has_mmu — MMU on one head", "[IMEX]") {
    REQUIRE(has_mmu(make_pem({0, 0, 0, 0, 1, 2, 3}))); // user's IQEX
    REQUIRE(has_mmu(make_pem({0, 0}))); // tiny MMU
}

TEST_CASE("has_mmu — MMU on second head", "[IMEX]") {
    // Hypothetical future: direct extruder on T0, MMU on T1
    REQUIRE(has_mmu(make_pem({0, 1, 1, 1})));
}

TEST_CASE("has_mmu — empty / single-entry pem", "[IMEX]") {
    REQUIRE_FALSE(has_mmu(make_pem({})));
    REQUIRE_FALSE(has_mmu(make_pem({0})));
}

TEST_CASE("parse_imex_head_filament_map — round-trip", "[IMEX]") {
    auto m = parse_imex_head_filament_map("0:3,4:5");
    REQUIRE(m.size() == 2);
    REQUIRE(m[0] == 3);
    REQUIRE(m[4] == 5);
}

TEST_CASE("parse_imex_head_filament_map — whitespace + empty tokens", "[IMEX]") {
    auto m = parse_imex_head_filament_map("  0 : 3 , , 4:5  ");
    REQUIRE(m.size() == 2);
    REQUIRE(m[0] == 3);
    REQUIRE(m[4] == 5);
}

TEST_CASE("parse_imex_head_filament_map — empty string", "[IMEX]") {
    REQUIRE(parse_imex_head_filament_map("").empty());
}

TEST_CASE("resolve_filament_for_head — override wins", "[IMEX]") {
    auto pem = make_pem({0, 0, 0, 0, 1, 2, 3});
    std::map<int,int> plate_map{{0, 3}}; // 1-based slot 3 = 0-based logical 2
    REQUIRE(resolve_filament_for_head(plate_map, pem, 0) == 2);
}

TEST_CASE("resolve_filament_for_head — fallback when unset", "[IMEX]") {
    auto pem = make_pem({0, 0, 0, 0, 1, 2, 3});
    std::map<int,int> plate_map{}; // empty
    REQUIRE(resolve_filament_for_head(plate_map, pem, 0) == 0); // first pem-routed
    REQUIRE(resolve_filament_for_head(plate_map, pem, 1) == 4);
}

TEST_CASE("resolve_filament_for_head — no routing for head", "[IMEX]") {
    auto pem = make_pem({0, 1}); // no head 2
    std::map<int,int> plate_map{};
    REQUIRE(resolve_filament_for_head(plate_map, pem, 2) == -1);
}

TEST_CASE("imex_primary_tool_for_mode — role marker authoritative", "[IMEX]") {
    // Tab.cpp enforces one Primary per mode; position is not semantically meaningful.
    REQUIRE(imex_primary_tool_for_mode("0:P,1:C,2:C") == 0);
    REQUIRE(imex_primary_tool_for_mode("1:C,0:P,2:M") == 0);
    REQUIRE(imex_primary_tool_for_mode("1:C,2:M,3:P") == 3);
}

TEST_CASE("imex_primary_tool_for_mode — backwards-compat plain index", "[IMEX]") {
    REQUIRE(imex_primary_tool_for_mode("0") == 0);
    REQUIRE(imex_primary_tool_for_mode("2") == 2);
    // Bare index wins when no :P marker is present; first bare index takes primary.
    REQUIRE(imex_primary_tool_for_mode("1,2,3") == 1);
}

TEST_CASE("imex_primary_tool_for_mode — explicit :P beats bare index", "[IMEX]") {
    // Mixed serialization: role marker must dominate over bare-index fallback.
    REQUIRE(imex_primary_tool_for_mode("1,2:P,3") == 2);
}

TEST_CASE("imex_primary_tool_for_mode — empty + malformed", "[IMEX]") {
    REQUIRE(imex_primary_tool_for_mode("") == -1);
    REQUIRE(imex_primary_tool_for_mode(",,") == -1);
    REQUIRE(imex_primary_tool_for_mode("abc:P") == -1); // idx not parseable
    REQUIRE(imex_primary_tool_for_mode("-1:P") == -1);   // negative idx rejected
}

TEST_CASE("imex_primary_tool_for_mode — whitespace tolerant", "[IMEX]") {
    REQUIRE(imex_primary_tool_for_mode("  0 : P , 1 : C ") == 0);
}

TEST_CASE("has_non_primary_mmu — MMU on primary only", "[IMEX]") {
    // User's Neo XP 0.6: 4 AFC lanes on T0, singles on T4/T5/T6.
    // Primary = 0 → no other head has MMU.
    auto pem = make_pem({0, 0, 0, 0, 4, 5, 6});
    REQUIRE_FALSE(has_non_primary_mmu(pem, 0));
}

TEST_CASE("has_non_primary_mmu — MMU on secondary", "[IMEX]") {
    // Direct extruder on T0, MMU on T1 → non-primary MMU present.
    auto pem = make_pem({0, 1, 1, 1});
    REQUIRE(has_non_primary_mmu(pem, 0));
}

TEST_CASE("has_non_primary_mmu — pure IDEX", "[IMEX]") {
    REQUIRE_FALSE(has_non_primary_mmu(make_pem({0, 1}), 0));
    REQUIRE_FALSE(has_non_primary_mmu(make_pem({0, 1, 2, 3}), 0));
}

TEST_CASE("has_non_primary_mmu — secondary single-lane", "[IMEX]") {
    // Primary is T4 (single filament); T0 has MMU.
    auto pem = make_pem({0, 0, 0, 0, 4});
    REQUIRE(has_non_primary_mmu(pem, 4));
}

TEST_CASE("has_non_primary_mmu — empty / single-entry pem", "[IMEX]") {
    REQUIRE_FALSE(has_non_primary_mmu(make_pem({}), 0));
    REQUIRE_FALSE(has_non_primary_mmu(make_pem({0}), 0));
}

TEST_CASE("imex_head_transform — copy mode is pure translation", "[IMEX]") {
    const Vec2d offset{120.0, 0.0};
    Transform3d xf = imex_head_transform(0, 1, ImexRole::Copy, offset);
    const Vec3d in{10.0, 20.0, 30.0};
    const Vec3d out = xf * in;
    REQUIRE_THAT(out.x(), WithinAbs(130.0, 1e-9));
    REQUIRE_THAT(out.y(), WithinAbs(20.0,  1e-9));
    REQUIRE_THAT(out.z(), WithinAbs(30.0,  1e-9));
}

TEST_CASE("imex_head_transform — mirror at origin places ghost at gantry offset", "[IMEX]") {
    // Primary instance at origin: ghost origin lands at the gantry offset (Copy-style),
    // and applying mirror flips geometry about origin (= primary's translation).
    const Vec2d offset{120.0, 0.0};
    Transform3d xf = imex_head_transform(0, 1, ImexRole::Mirror, offset);
    const Vec3d mapped = xf * Vec3d::Zero();
    REQUIRE_THAT(mapped.x(), WithinAbs(120.0, 1e-9));
    REQUIRE_THAT(mapped.y(), WithinAbs(0.0,   1e-9));
    REQUIRE_THAT(mapped.z(), WithinAbs(0.0,   1e-9));
}

TEST_CASE("imex_head_transform — mirror flips geometry in-place about primary origin", "[IMEX]") {
    // Primary sits at (50, 10, 0). Gantry +120 in X → ghost origin at (170, 10, 0),
    // NOT reflected about a midplane. A model point at +5 X from primary origin ends
    // up at -5 X from the ghost origin (geometric flip preserved).
    const Vec2d offset{120.0, 0.0};
    const Vec3d primary_origin{50.0, 10.0, 0.0};
    Transform3d xf = imex_head_transform(0, 1, ImexRole::Mirror, offset, primary_origin);

    const Vec3d ghost_origin = xf * primary_origin;
    REQUIRE_THAT(ghost_origin.x(), WithinAbs(170.0, 1e-9)); // 50 + 120 (Copy-style)
    REQUIRE_THAT(ghost_origin.y(), WithinAbs(10.0,  1e-9));

    const Vec3d plus5 = primary_origin + Vec3d(5.0, 0.0, 0.0);
    const Vec3d mapped = xf * plus5;
    REQUIRE_THAT(mapped.x(), WithinAbs(165.0, 1e-9)); // ghost_origin - 5 (flipped)
    REQUIRE_THAT(mapped.y(), WithinAbs(10.0,  1e-9));
}

TEST_CASE("imex_head_transform — mirror tracks primary motion (same delta as Copy)", "[IMEX]") {
    // When the primary moves by some delta, the ghost origin must move by the SAME delta
    // (not the reflected delta). This is the core user requirement: mirrored geometry,
    // un-mirrored motion.
    const Vec2d offset{120.0, 0.0};
    const Vec3d p0{0.0, 0.0, 0.0};
    const Vec3d p1{30.0, -5.0, 0.0};

    Transform3d xf0 = imex_head_transform(0, 1, ImexRole::Mirror, offset, p0);
    Transform3d xf1 = imex_head_transform(0, 1, ImexRole::Mirror, offset, p1);

    const Vec3d ghost0 = xf0 * p0;
    const Vec3d ghost1 = xf1 * p1;
    const Vec3d ghost_delta = ghost1 - ghost0;
    const Vec3d primary_delta = p1 - p0;

    REQUIRE_THAT(ghost_delta.x(), WithinAbs(primary_delta.x(), 1e-9));
    REQUIRE_THAT(ghost_delta.y(), WithinAbs(primary_delta.y(), 1e-9));
    REQUIRE_THAT(ghost_delta.z(), WithinAbs(primary_delta.z(), 1e-9));
}

TEST_CASE("imex_head_transform — mirror reflects model point across primary origin", "[IMEX]") {
    // Primary at origin, offset +X. Model point at +5 X lands 5 left of ghost origin.
    // Matches the pre-refactor semantics for the special case primary_origin = 0.
    const Vec2d offset{120.0, 0.0};
    Transform3d xf = imex_head_transform(0, 1, ImexRole::Mirror, offset);
    const Vec3d in{5.0, 7.0, 0.0};
    const Vec3d out = xf * in;
    REQUIRE_THAT(out.x(), WithinAbs(115.0, 1e-9));
    REQUIRE_THAT(out.y(), WithinAbs(7.0,   1e-9));
}

TEST_CASE("imex_head_transform — mirror reflection is X-axis regardless of offset direction", "[IMEX]") {
    // Mirror's reflection plane normal is the primary-row gantry axis (X), not
    // gantry_offset.normalized(). A pure-Y offset (off-row target) must still flip
    // X, not Y — otherwise off-row Mirror ghosts end up rotated vs their on-row peers.
    const Vec2d offset{0.0, 80.0};
    Transform3d xf = imex_head_transform(0, 1, ImexRole::Mirror, offset);
    const Vec3d in{3.0, 10.0, 0.0};
    const Vec3d out = xf * in;
    REQUIRE_THAT(out.x(), WithinAbs(-3.0, 1e-9));  // X flipped about origin
    REQUIRE_THAT(out.y(), WithinAbs(90.0, 1e-9));  // Y translated by gantry, unflipped
}

TEST_CASE("imex_head_transform — primary is identity", "[IMEX]") {
    const Vec2d offset{120.0, 30.0};
    Transform3d xf = imex_head_transform(0, 0, ImexRole::Primary, offset);
    REQUIRE(xf.isApprox(Transform3d::Identity()));
}

TEST_CASE("imex_head_transform — mirror with zero offset is identity", "[IMEX]") {
    const Vec2d offset{0.0, 0.0};
    Transform3d xf = imex_head_transform(0, 1, ImexRole::Mirror, offset);
    REQUIRE(xf.isApprox(Transform3d::Identity()));
}

TEST_CASE("imex_head_transform — mirror on 2x2 off-row target (diagonal offset) flips X only", "[IMEX]") {
    // 2x2 IMEX layout: primary T0 at rear-left, target T3 at front-right → diagonal
    // gantry_offset. Reflection plane must still be X-axis (same as on-row T1 mirror),
    // not the diagonal direction — otherwise T3's ghost reads as rotated ~45° in plan
    // view (the bug this test guards against). Primary origin (0,0,0) maps to target
    // origin (100,100,0) regardless.
    const Vec2d offset{100.0, 100.0};
    Transform3d xf = imex_head_transform(0, 3, ImexRole::Mirror, offset);

    const Vec3d mapped = xf * Vec3d::Zero();
    REQUIRE_THAT(mapped.x(), WithinAbs(100.0, 1e-9));
    REQUIRE_THAT(mapped.y(), WithinAbs(100.0, 1e-9));
    REQUIRE_THAT(mapped.z(), WithinAbs(0.0,   1e-9));

    // A model point offset +5 X from primary ends up 5 LEFT of the ghost origin
    // (X flipped), while Y translates 1:1 (Y unflipped).
    const Vec3d in{5.0, 7.0, 0.0};
    const Vec3d out = xf * in;
    REQUIRE_THAT(out.x(), WithinAbs(95.0,  1e-9));   // 100 - 5
    REQUIRE_THAT(out.y(), WithinAbs(107.0, 1e-9));   // 100 + 7
}

TEST_CASE("resolve_filament_for_head — no routing returns -1 (ghost color fallback)", "[IMEX]") {
    // User's IQEX pem: T0 has 4 AFC lanes, T1/T2/T3 direct. T5 is unrouted.
    auto pem = make_pem({0, 0, 0, 0, 1, 2, 3});
    std::map<int,int> no_override;
    REQUIRE(resolve_filament_for_head(no_override, pem, 5) == -1);

    // Plate override for an unrouted head still resolves (user's explicit choice wins).
    std::map<int,int> override_on_5 = {{5, 7}};
    REQUIRE(resolve_filament_for_head(override_on_5, pem, 5) == 6);
}
