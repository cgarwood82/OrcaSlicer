#include <catch2/catch_all.hpp>

#include <memory>

#include "libslic3r/GCodeWriter.hpp"

using namespace Slic3r;

SCENARIO("lift() is not ignored after unlift() at normal values of Z", "[GCodeWriter][.]") {
    GIVEN("A config from a file and a single extruder.") {
        GCodeWriter writer;
        GCodeConfig &config = writer.config;
        config.load(std::string(TEST_DATA_DIR) + "/fff_print_tests/test_gcodewriter/config_lift_unlift.ini", ForwardCompatibilitySubstitutionRule::Disable);

        std::vector<unsigned int> extruder_ids {0};
        writer.set_extruders(extruder_ids);
        writer.set_extruder(0);

        WHEN("Z is set to 203") {
            double trouble_Z = 203;
            writer.travel_to_z(trouble_Z);
            AND_WHEN("GcodeWriter::Lift() is called") {
                REQUIRE(writer.lazy_lift().size() > 0);
                AND_WHEN("Z is moved post-lift to the same delta as the config Z lift") {
                    REQUIRE(writer.travel_to_z(trouble_Z + config.z_hop.values[0]).size() == 0);
                    AND_WHEN("GCodeWriter::Unlift() is called") {
                        REQUIRE(writer.unlift().size() == 0); // we're the same height so no additional move happens.
                        THEN("GCodeWriter::Lift() emits gcode.") {
                            REQUIRE(writer.lazy_lift().size() > 0);
                        }
                    }
                }
            }
        }
        WHEN("Z is set to 500003") {
            double trouble_Z = 500003;
            writer.travel_to_z(trouble_Z);
            AND_WHEN("GcodeWriter::Lift() is called") {
                REQUIRE(writer.lazy_lift().size() > 0);
                AND_WHEN("Z is moved post-lift to the same delta as the config Z lift") {
                    REQUIRE(writer.travel_to_z(trouble_Z + config.z_hop.values[0]).size() == 0);
                    AND_WHEN("GCodeWriter::Unlift() is called") {
                        REQUIRE(writer.unlift().size() == 0); // we're the same height so no additional move happens.
                        THEN("GCodeWriter::Lift() emits gcode.") {
                            REQUIRE(writer.lazy_lift().size() > 0);
                        }
                    }
                }
            }
        }
        WHEN("Z is set to 10.3") {
            double trouble_Z = 10.3;
            writer.travel_to_z(trouble_Z);
            AND_WHEN("GcodeWriter::Lift() is called") {
                REQUIRE(writer.lazy_lift().size() > 0);
                AND_WHEN("Z is moved post-lift to the same delta as the config Z lift") {
                    REQUIRE(writer.travel_to_z(trouble_Z + config.z_hop.values[0]).size() == 0);
                    AND_WHEN("GCodeWriter::Unlift() is called") {
                        REQUIRE(writer.unlift().size() == 0); // we're the same height so no additional move happens.
                        THEN("GCodeWriter::Lift() emits gcode.") {
                            REQUIRE(writer.lazy_lift().size() > 0);
                        }
                    }
                }
            }
        }
		// The test above will fail for trouble_Z == 9007199254740992, where trouble_Z + 1.5 will be rounded to trouble_Z + 2.0 due to double mantisa overflow.
    }
}

SCENARIO("set_pressure_advance emits nothing for negative PA", "[GCodeWriter][PressureAdvance]") {
    GIVEN("A default GCodeWriter") {
        GCodeWriter writer;
        THEN("Negative PA returns empty regardless of firmware flavor") {
            writer.config.gcode_flavor.value = gcfKlipper;
            REQUIRE(writer.set_pressure_advance(-1.0).empty());
            writer.config.gcode_flavor.value = gcfRepRapFirmware;
            REQUIRE(writer.set_pressure_advance(-0.001).empty());
            writer.config.gcode_flavor.value = gcfMarlinFirmware;
            REQUIRE(writer.set_pressure_advance(-100.0).empty());
        }
    }
}

SCENARIO("set_pressure_advance emits Klipper form with optional EXTRUDER=extruder<N>", "[GCodeWriter][PressureAdvance]") {
    GIVEN("A Klipper-flavored GCodeWriter") {
        GCodeWriter writer;
        writer.config.gcode_flavor.value = gcfKlipper;

        WHEN("set_pressure_advance is called without a tool index") {
            std::string out = writer.set_pressure_advance(0.05);
            THEN("Output contains SET_PRESSURE_ADVANCE ADVANCE=0.05 with no EXTRUDER qualifier") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("SET_PRESSURE_ADVANCE"));
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("ADVANCE=0.05"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring("EXTRUDER="));
            }
        }
        WHEN("set_pressure_advance is called with tool=0") {
            std::string out = writer.set_pressure_advance(0.04, 0);
            THEN("Output targets EXTRUDER=extruder (no trailing index)") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("EXTRUDER=extruder "));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring("EXTRUDER=extruder0"));
            }
        }
        WHEN("set_pressure_advance is called with tool=3") {
            std::string out = writer.set_pressure_advance(0.06, 3);
            THEN("Output targets EXTRUDER=extruder3") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("EXTRUDER=extruder3"));
            }
        }
    }
}

SCENARIO("set_pressure_advance emits RepRapFirmware form with optional D<N>", "[GCodeWriter][PressureAdvance]") {
    GIVEN("An RRF-flavored GCodeWriter") {
        GCodeWriter writer;
        writer.config.gcode_flavor.value = gcfRepRapFirmware;

        WHEN("set_pressure_advance is called without a tool index") {
            std::string out = writer.set_pressure_advance(0.07);
            THEN("Output is bare M572 S... with no D qualifier") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M572 S0.07"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring(" D"));
            }
        }
        WHEN("set_pressure_advance is called with tool=0") {
            std::string out = writer.set_pressure_advance(0.08, 0);
            THEN("Output contains D0 (explicit tool 0, not the current-tool fallback)") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M572 D0 S0.08"));
            }
        }
        WHEN("set_pressure_advance is called with tool=2") {
            std::string out = writer.set_pressure_advance(0.09, 2);
            THEN("Output contains D2") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M572 D2 S0.09"));
            }
        }
    }
}

SCENARIO("set_pressure_advance emits Marlin 2.x form with optional T<N>", "[GCodeWriter][PressureAdvance]") {
    GIVEN("A Marlin 2-flavored GCodeWriter") {
        GCodeWriter writer;
        writer.config.gcode_flavor.value = gcfMarlinFirmware;

        WHEN("set_pressure_advance is called without a tool index") {
            std::string out = writer.set_pressure_advance(0.10);
            THEN("Output is bare M900 K... with no T qualifier") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M900 K0.1 "));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring(" T"));
            }
        }
        WHEN("set_pressure_advance is called with tool=0") {
            std::string out = writer.set_pressure_advance(0.11, 0);
            THEN("Output contains T0") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M900 K0.11 T0"));
            }
        }
        WHEN("set_pressure_advance is called with tool=1") {
            std::string out = writer.set_pressure_advance(0.12, 1);
            THEN("Output contains T1") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M900 K0.12 T1"));
            }
        }
    }
}

SCENARIO("set_pressure_advance emits Marlin Legacy form without tool qualifier even when tool index is supplied",
         "[GCodeWriter][PressureAdvance]") {
    GIVEN("A Marlin Legacy-flavored GCodeWriter") {
        GCodeWriter writer;
        writer.config.gcode_flavor.value = gcfMarlinLegacy;

        WHEN("set_pressure_advance is called without a tool index") {
            std::string out = writer.set_pressure_advance(0.05);
            THEN("Output is bare M900 K...") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M900 K0.05"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring(" T"));
            }
        }
        WHEN("set_pressure_advance is called with tool=2 (a hypothetical IMEX secondary)") {
            std::string out = writer.set_pressure_advance(0.06, 2);
            THEN("Output is still bare M900 — Marlin Legacy has no per-tool LA") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M900 K0.06"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring(" T2"));
            }
        }
    }
}

SCENARIO("set_temperature per-flavor command routing", "[GCodeWriter][Temperature]") {
    GIVEN("temperature=210, no tool index, no wait") {
        WHEN("flavor is Marlin 2") {
            std::string out = GCodeWriter::set_temperature(210, gcfMarlinFirmware, false, -1, std::string());
            THEN("output is M104 S210 with no tool qualifier") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M104 S210"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring(" T"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring("M109"));
            }
        }
        WHEN("flavor is RepRapFirmware") {
            std::string out = GCodeWriter::set_temperature(210, gcfRepRapFirmware, false, -1, std::string());
            THEN("output is G10 S210 (M104 is deprecated on RRF)") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("G10 S210"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring("M104"));
            }
        }
        WHEN("flavor is Mach3 or Machinekit") {
            std::string mach3      = GCodeWriter::set_temperature(210, gcfMach3,      false, -1, std::string());
            std::string machinekit = GCodeWriter::set_temperature(210, gcfMachinekit, false, -1, std::string());
            THEN("output uses P-prefix for the value instead of S") {
                REQUIRE_THAT(mach3,      Catch::Matchers::ContainsSubstring("M104 P210"));
                REQUIRE_THAT(machinekit, Catch::Matchers::ContainsSubstring("M104 P210"));
                REQUIRE_THAT(mach3,      !Catch::Matchers::ContainsSubstring("S210"));
            }
        }
    }
}

SCENARIO("set_temperature wait=true handling per firmware", "[GCodeWriter][Temperature]") {
    WHEN("flavor is Marlin 2 with wait") {
        std::string out = GCodeWriter::set_temperature(210, gcfMarlinFirmware, true, -1, std::string());
        THEN("output is M109 S210 (blocking wait)") {
            REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M109 S210"));
            REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring("M104"));
        }
    }
    WHEN("flavor is MakerWare or Sailfish with wait") {
        std::string mw = GCodeWriter::set_temperature(210, gcfMakerWare, true, -1, std::string());
        std::string sf = GCodeWriter::set_temperature(210, gcfSailfish,  true, -1, std::string());
        THEN("output is empty — these flavors don't support blocking waits") {
            REQUIRE(mw.empty());
            REQUIRE(sf.empty());
        }
    }
    WHEN("flavor is Teacup with wait") {
        std::string out = GCodeWriter::set_temperature(210, gcfTeacup, true, -1, std::string());
        THEN("output emits M104 + a separate M116 poll (Teacup doesn't support M109)") {
            REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M104 S210"));
            REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M116"));
            REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring("M109"));
        }
    }
    WHEN("flavor is RepRapFirmware with wait") {
        std::string out = GCodeWriter::set_temperature(210, gcfRepRapFirmware, true, -1, std::string());
        THEN("output emits G10 + M116 (same poll pattern as Teacup)") {
            REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("G10 S210"));
            REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M116"));
        }
    }
}

SCENARIO("set_temperature per-tool qualifier routing for IMEX secondary carriages",
         "[GCodeWriter][Temperature]") {
    // IMEX secondary tools never go through a tool-change, so layer-change temperature
    // for them is emitted via the tool-qualified static set_temperature overload.
    GIVEN("temperature=220, tool=2, no wait") {
        WHEN("flavor is Marlin 2") {
            std::string out = GCodeWriter::set_temperature(220, gcfMarlinFirmware, false, 2, std::string());
            THEN("output contains T2 qualifier") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M104 S220 T2"));
            }
        }
        WHEN("flavor is RepRapFirmware") {
            std::string out = GCodeWriter::set_temperature(220, gcfRepRapFirmware, false, 2, std::string());
            THEN("output uses P-prefix for tool (RRF convention), not T") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("G10 S220 P2"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring(" T2"));
            }
        }
        WHEN("flavor is Klipper") {
            std::string out = GCodeWriter::set_temperature(220, gcfKlipper, false, 1, std::string());
            THEN("output contains T1 qualifier (Klipper layer-change temperature uses T)") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M104 S220 T1"));
            }
        }
    }
}

SCENARIO("set_temperature instance overload forces tool=-1 on single-extruder writers",
         "[GCodeWriter][Temperature]") {
    // Guards against spuriously emitting `T0` on printers that only have one extruder.
    // The instance overload discards the tool argument when !multiple_extruders.
    GIVEN("A default GCodeWriter (multiple_extruders=false)") {
        GCodeWriter writer;
        writer.config.gcode_flavor.value = gcfMarlinFirmware;

        WHEN("set_temperature is called with tool=2") {
            std::string out = writer.set_temperature(210, false, 2);
            THEN("output has no T qualifier despite the caller passing tool=2") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M104 S210"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring(" T"));
            }
        }
    }
    GIVEN("A GCodeWriter with multiple_extruders=true (not SEMM)") {
        GCodeWriter writer;
        writer.config.gcode_flavor.value = gcfMarlinFirmware;
        writer.multiple_extruders = true;

        WHEN("set_temperature is called with tool=2") {
            std::string out = writer.set_temperature(210, false, 2);
            THEN("tool argument passes through — output contains T2") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M104 S210 T2"));
            }
        }
    }
}

SCENARIO("set_pressure_advance emits BBL M900 L1000 M10 regardless of tool index",
         "[GCodeWriter][PressureAdvance]") {
    GIVEN("A BBL-flagged GCodeWriter (the flag overrides firmware flavor routing)") {
        GCodeWriter writer;
        writer.set_is_bbl_machine(true);
        // Flavor intentionally set to something other than the BBL branch to prove the flag wins.
        writer.config.gcode_flavor.value = gcfMarlinFirmware;

        WHEN("set_pressure_advance is called without a tool index") {
            std::string out = writer.set_pressure_advance(0.05);
            THEN("Output is the BBL-specific M900 Kx L1000 M10 form") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M900 K0.05 L1000 M10"));
            }
        }
        WHEN("set_pressure_advance is called with a tool index") {
            std::string out = writer.set_pressure_advance(0.05, 2);
            THEN("BBL output is unchanged — no per-tool qualifier is emitted on BBL printers") {
                REQUIRE_THAT(out, Catch::Matchers::ContainsSubstring("M900 K0.05 L1000 M10"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring(" T2"));
                REQUIRE_THAT(out, !Catch::Matchers::ContainsSubstring("EXTRUDER="));
            }
        }
    }
}

SCENARIO("set_speed emits values with fixed-point output.", "[GCodeWriter]") {

    GIVEN("GCodeWriter instance") {
        GCodeWriter writer;
        WHEN("set_speed is called to set speed to 99999.123") {
            THEN("Output string is G1 F99999.123") {
                REQUIRE_THAT(writer.set_speed(99999.123), Catch::Matchers::Equals("G1 F99999.123\n"));
            }
        }
        WHEN("set_speed is called to set speed to 1") {
            THEN("Output string is G1 F1") {
                REQUIRE_THAT(writer.set_speed(1.0), Catch::Matchers::Equals("G1 F1\n"));
            }
        }
        WHEN("set_speed is called to set speed to 203.200022") {
            THEN("Output string is G1 F203.2") {
                REQUIRE_THAT(writer.set_speed(203.200022), Catch::Matchers::Equals("G1 F203.2\n"));
            }
        }
        WHEN("set_speed is called to set speed to 203.200522") {
            THEN("Output string is G1 F203.201") {
                REQUIRE_THAT(writer.set_speed(203.200522), Catch::Matchers::Equals("G1 F203.201\n"));
            }
        }
    }
}
