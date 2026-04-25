#include "libslic3r/IMEXHelpers.hpp"

#include <algorithm>
#include <cassert>
#include <cctype>
#include <sstream>
#include <unordered_set>

#include "libslic3r/PresetBundle.hpp"

namespace Slic3r {

ConfigOptionInts effective_physical_extruder_map(const ConfigOptionInts* explicit_pem,
                                                  const ConfigOptionInts* printer_extruder_id)
{
    if (explicit_pem && explicit_pem->values.size() >= 2)
        return *explicit_pem;
    ConfigOptionInts derived;
    if (printer_extruder_id) {
        derived.values.reserve(printer_extruder_id->values.size());
        for (int v : printer_extruder_id->values)
            derived.values.push_back(v - 1);
    }
    return derived;
}

ConfigOptionInts effective_physical_extruder_map(const PresetBundle& pb)
{
    const ConfigOptionInts* explicit_pem = pb.project_config.option<ConfigOptionInts>("physical_extruder_map");
    if (!explicit_pem || explicit_pem->values.size() < 2)
        explicit_pem = pb.printers.get_edited_preset().config.option<ConfigOptionInts>("physical_extruder_map");
    const ConfigOptionInts* pei = pb.printers.get_edited_preset().config.option<ConfigOptionInts>("printer_extruder_id");
    return effective_physical_extruder_map(explicit_pem, pei);
}

int first_filament_for_physical_head(const ConfigOptionInts& pem, int physical)
{
    const auto& v = pem.values;
    if (v.empty())
        return (physical == 0) ? 0 : -1; // degenerate: treat as identity on head 0
    for (size_t i = 0; i < v.size(); ++i) {
        if (v[i] == physical)
            return static_cast<int>(i);
    }
    return -1;
}

bool has_mmu(const ConfigOptionInts& pem)
{
    if (pem.values.size() < 2)
        return false;
    std::unordered_set<int> seen;
    for (int pv : pem.values) {
        if (!seen.insert(pv).second)
            return true;
    }
    return false;
}

bool has_non_primary_mmu(const ConfigOptionInts& pem, int primary_physical)
{
    if (pem.values.size() < 2)
        return false;
    std::unordered_set<int> seen;
    for (int pv : pem.values) {
        if (pv == primary_physical)
            continue;
        if (!seen.insert(pv).second)
            return true;
    }
    return false;
}

std::vector<std::pair<int, ImexRole>> parse_imex_active_tools(const std::string& active_tools_for_mode)
{
    std::vector<std::pair<int, ImexRole>> out;
    if (active_tools_for_mode.empty()) return out;
    std::istringstream ss(active_tools_for_mode);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        tok.erase(std::remove_if(tok.begin(), tok.end(), ::isspace), tok.end());
        if (tok.empty()) continue;
        const auto colon = tok.find(':');
        const std::string idx_str = (colon == std::string::npos) ? tok : tok.substr(0, colon);
        int phys = -1;
        try { phys = std::stoi(idx_str); } catch (...) { continue; }
        if (phys < 0 || phys >= (int)MAXIMUM_EXTRUDER_NUMBER) continue;
        ImexRole role = ImexRole::Copy;
        if (colon != std::string::npos) {
            const std::string r = tok.substr(colon + 1);
            if      (r == "P") role = ImexRole::Primary;
            else if (r == "M") role = ImexRole::Mirror;
            // "C" and anything else → Copy.
        }
        out.emplace_back(phys, role);
    }
    return out;
}

int imex_primary_tool_for_mode(const std::string& active_tools_for_mode)
{
    if (active_tools_for_mode.empty())
        return -1;
    std::istringstream ss(active_tools_for_mode);
    std::string token;
    int first_bare = -1;
    while (std::getline(ss, token, ',')) {
        token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        if (token.empty())
            continue;
        const auto colon = token.find(':');
        const std::string idx_str = (colon == std::string::npos) ? token : token.substr(0, colon);
        int idx = -1;
        try {
            idx = std::stoi(idx_str);
        } catch (...) {
            continue;
        }
        if (idx < 0)
            continue;
        if (colon == std::string::npos) {
            // Backwards-compat: a bare index is Primary.
            if (first_bare < 0)
                first_bare = idx;
            continue;
        }
        const std::string role = token.substr(colon + 1);
        if (role == "P")
            return idx;
    }
    return first_bare;
}

std::map<int,int> parse_imex_head_filament_map(const std::string& s)
{
    std::map<int,int> result;
    if (s.empty()) return result;
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        if (token.empty()) continue;
        auto colon = token.find(':');
        if (colon == std::string::npos) continue;
        try {
            int phys = std::stoi(token.substr(0, colon));
            int slot = std::stoi(token.substr(colon + 1));
            if (phys >= 0 && slot >= 1)
                result[phys] = slot;
        } catch (...) {}
    }
    return result;
}

int resolve_filament_for_head(const std::map<int,int>& plate_map,
                              const ConfigOptionInts&  pem,
                              int                       physical)
{
    auto it = plate_map.find(physical);
    if (it != plate_map.end()) {
        const int zero_based = it->second - 1;
        if (zero_based >= 0)
            return zero_based;
    }
    return first_filament_for_physical_head(pem, physical);
}

Transform3d imex_head_transform(int /*primary*/, int /*target*/, ImexRole role,
                                const Vec2d& gantry_offset,
                                const Vec2d& primary_zone_center)
{
    switch (role) {
    case ImexRole::Primary:
        return Transform3d::Identity();
    case ImexRole::Copy:
        return Eigen::Translation3d(gantry_offset.x(), gantry_offset.y(), 0.0)
               * Transform3d::Identity();
    case ImexRole::Mirror: {
        const double len2 = gantry_offset.squaredNorm();
        if (len2 < 1e-12)
            return Transform3d::Identity();
        // True reflection about the zone-boundary plane (perpendicular to X, passing
        // through primary_zone_center.x + gantry_offset.x/2). This makes the ghost
        // land at the mirrored position within the target zone — matching where the
        // mirror tool will actually print — and reflects drag motion so X is inverted
        // while Y translates 1:1 by gantry_offset.y. Off-row Mirror targets (e.g. T3
        // on a 2x2) still reflect across this X-plane rather than the diagonal, so
        // they visually match their on-row counterparts.
        //   .linear()      = Reflect(X) = diag(-1, 1, 1)
        //   .translation() = (2*primary_zone_center.x + gantry_offset.x, gantry_offset.y, 0)
        // TODO: if a future IMEX printer has Y-oriented gantries, lift this to a
        // caller-supplied axis.
        Transform3d out = Transform3d::Identity();
        out.linear()      = Eigen::DiagonalMatrix<double, 3>(-1.0, 1.0, 1.0);
        out.translation() = Vec3d(2.0 * primary_zone_center.x() + gantry_offset.x(),
                                  gantry_offset.y(),
                                  0.0);
        return out;
    }
    }
    return Transform3d::Identity();
}

} // namespace Slic3r
