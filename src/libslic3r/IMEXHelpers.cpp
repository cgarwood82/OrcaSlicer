#include "libslic3r/IMEXHelpers.hpp"

#include <algorithm>
#include <cassert>
#include <cctype>
#include <sstream>
#include <unordered_set>

namespace Slic3r {

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
        if (phys < 0) continue;
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
                                const Vec3d& primary_origin)
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
        // Reflection plane normal is the primary-row gantry axis (X for all
        // current IMEX printers), not gantry_offset.normalized(). An off-row
        // Mirror target (e.g. T3 on a 2x2 where primary is T0) has a diagonal
        // gantry_offset; reflecting across that diagonal plane rotates the
        // ghost ~45° in plan view, which visually reads as the object laying
        // on its side. All Mirror ghosts must flip across the same plane
        // (the one the primary-row Primary↔Mirror pair defines), so that
        // off-row mirrors look like their on-row counterparts, just placed
        // at the off-row position.
        // Formula: head_xf = T(gantry) * T(p) * Reflect(n) * T(-p)
        //   .linear()      = Reflect(n) = I - 2 n n^T
        //   .translation() = gantry + (I - Reflect) * p = gantry + 2 n n^T p
        // TODO: if a future IMEX printer has Y-oriented gantries, lift this
        // to a caller-supplied axis.
        const Vec3d n(1.0, 0.0, 0.0);
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d nnT = n * n.transpose();
        Eigen::Matrix3d L = I3 - 2.0 * nnT;
        Vec3d t = Vec3d(gantry_offset.x(), gantry_offset.y(), 0.0)
                  + (I3 - L) * primary_origin;
        Transform3d out = Transform3d::Identity();
        out.linear() = L;
        out.translation() = t;
        return out;
    }
    }
    return Transform3d::Identity();
}

} // namespace Slic3r
