
#include "libslic3r/Model.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/filesystem/operations.hpp>

#include <catch2/catch_tostring.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <type_traits> // for std::enable_if_t
#include <typeinfo>    // for typeid

namespace Catch {
    template <typename T>
    struct is_eigen_matrix : std::is_base_of<Eigen::MatrixBase<T>, T> {};

    template <typename T>
    struct StringMaker<T, std::enable_if_t<is_eigen_matrix<T>::value>> {
        static std::string convert(const T& eigen_obj) {
            // Newline at end of rows
            Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
            std::stringstream ss;
            ss << "Matrix<" << typeid(eigen_obj).name() << "> = \n";
            ss << eigen_obj.format(fmt);
            return ss.str();
        }
    };
    
    // We must manually specialize for Eigen::Transform as it doesn't derive from MatrixBase.
    // It's defined as: Eigen::Transform<Scalar, Dim, Mode, Options>
    template <typename Scalar, int Dim, int Mode, int Options>
    struct StringMaker<Eigen::Transform<Scalar, Dim, Mode, Options>> {
        static std::string convert(const Eigen::Transform<Scalar, Dim, Mode, Options>& trafo) {
            // We print the underlying matrix 
            const auto& matrix = trafo.matrix();

            // Newline at end of rows
            Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
            std::stringstream ss;
            
            ss << "Transform<Mode=" << Mode << ", Dim=" << Dim << "> = \n"; 
            ss << matrix.format(fmt);
            return ss.str();
        }
    };
    
    // Quaternions also need an explicit specialization
    template <typename Scalar, int Options>
    struct StringMaker<Eigen::Quaternion<Scalar, Options>> {
        static std::string convert(const Eigen::Quaternion<Scalar, Options>& quat) {
            std::stringstream ss;
            ss << "Quaternion(w=" << quat.w() << ", x=" << quat.x() << ", y=" << quat.y() << ", z=" << quat.z() << ")";
            return ss.str();
        }
    };
} // end namespace Catch

#include <catch2/catch_all.hpp>

using namespace Slic3r;


SCENARIO("Reading 3mf file", "[3mf]") {
    GIVEN("umlauts in the path of the file") {
        Model model;
        WHEN("3mf model is read") {
            std::string path = std::string(TEST_DATA_DIR) + "/test_3mf/Geräte/Büchse.3mf";
            DynamicPrintConfig config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Disable };
            bool ret = load_3mf(path.c_str(), config, ctxt, &model, false);
            THEN("load should succeed") {
                REQUIRE(ret);
            }
        }
    }
}

SCENARIO("Export+Import geometry to/from 3mf file cycle", "[3mf]") {
    GIVEN("world vertices coordinates before save") {
        // load a model from stl file
        Model src_model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        load_stl(src_file.c_str(), &src_model);
        src_model.add_default_instances();

        ModelObject* src_object = src_model.objects.front();

        // apply generic transformation to the 1st volume
        Geometry::Transformation src_volume_transform;
        src_volume_transform.set_offset({ 10.0, 20.0, 0.0 });
        src_volume_transform.set_rotation({ Geometry::deg2rad(25.0), Geometry::deg2rad(35.0), Geometry::deg2rad(45.0) });
        src_volume_transform.set_scaling_factor({ 1.1, 1.2, 1.3 });
        src_volume_transform.set_mirror({ -1.0, 1.0, -1.0 });
        src_object->volumes.front()->set_transformation(src_volume_transform);

        // apply generic transformation to the 1st instance
        Geometry::Transformation src_instance_transform;
        src_instance_transform.set_offset({ 5.0, 10.0, 0.0 });
        src_instance_transform.set_rotation({ Geometry::deg2rad(12.0), Geometry::deg2rad(13.0), Geometry::deg2rad(14.0) });
        src_instance_transform.set_scaling_factor({ 0.9, 0.8, 0.7 });
        src_instance_transform.set_mirror({ 1.0, -1.0, -1.0 });
        src_object->instances.front()->set_transformation(src_instance_transform);

        WHEN("model is saved+loaded to/from 3mf file") {
            // save the model to 3mf file
            std::string test_file = std::string(TEST_DATA_DIR) + "/test_3mf/prusa.3mf";
            store_3mf(test_file.c_str(), &src_model, nullptr, false);

            // load back the model from the 3mf file
            Model dst_model;
            DynamicPrintConfig dst_config;
            {
                ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Disable };
                load_3mf(test_file.c_str(), dst_config, ctxt, &dst_model, false);
            }
            boost::filesystem::remove(test_file);

            // compare meshes
            TriangleMesh src_mesh = src_model.mesh();
            TriangleMesh dst_mesh = dst_model.mesh();

            bool res = src_mesh.its.vertices.size() == dst_mesh.its.vertices.size();
            if (res) {
                for (size_t i = 0; i < dst_mesh.its.vertices.size(); ++i) {
                    res &= dst_mesh.its.vertices[i].isApprox(src_mesh.its.vertices[i]);
                }
            }
            THEN("world vertices coordinates after load match") {
                REQUIRE(res);
            }
        }
    }
}

SCENARIO("BBS 3MF round-trips per-plate IMEX state (parallel mode + head filament map)", "[3mf][IMEX]") {
    // Regression guard for the class of bug where per-plate state silently drops through
    // save/load (the variant-truncation bug was the precipitating example; IMEX plate state
    // rides the same XML metadata path and is equally vulnerable).
    // BBS exporter scaffolds a backup dir under temporary_dir() for the project config file;
    // point it at a writable location for the test process.
    set_temporary_dir(boost::filesystem::temp_directory_path().string());

    GIVEN("A Model with a single object on plate 0 and IMEX plate state set") {
        Model src_model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &src_model));
        src_model.add_default_instances();

        DynamicPrintConfig src_config;

        PlateDataPtrs src_plates;
        auto *plate0 = new PlateData();
        plate0->plate_index = 0;
        plate0->config.set_key_value("imex_parallel_mode",     new ConfigOptionString("copy_mode"));
        plate0->config.set_key_value("imex_head_filament_map", new ConfigOptionString("1:2,2:3"));
        src_plates.push_back(plate0);

        WHEN("the model is saved to BBS 3MF and loaded back") {
            std::string test_file = std::string(TEST_DATA_DIR) + "/test_3mf/imex_roundtrip.3mf";

            StoreParams store_params;
            store_params.path            = test_file.c_str();
            store_params.model           = &src_model;
            store_params.plate_data_list = src_plates;
            store_params.config          = &src_config;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            PlateDataPtrs dst_plates;
            std::vector<Preset*> dst_presets;
            bool is_bbl = false;
            bool is_orca = false;
            Semver file_version;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Disable };
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model,
                                       &dst_plates, &dst_presets, &is_bbl, &is_orca, &file_version);
            boost::filesystem::remove(test_file);

            THEN("load succeeds") {
                REQUIRE(loaded);
            }
            THEN("the loaded plate list has the same number of plates") {
                REQUIRE(dst_plates.size() == src_plates.size());
            }
            THEN("imex_parallel_mode round-trips with its original value") {
                REQUIRE(dst_plates.size() >= 1);
                auto *mode_opt = dst_plates[0]->config.option<ConfigOptionString>("imex_parallel_mode");
                REQUIRE(mode_opt != nullptr);
                REQUIRE(mode_opt->value == "copy_mode");
            }
            THEN("imex_head_filament_map round-trips with its original value") {
                REQUIRE(dst_plates.size() >= 1);
                auto *hfm_opt = dst_plates[0]->config.option<ConfigOptionString>("imex_head_filament_map");
                REQUIRE(hfm_opt != nullptr);
                REQUIRE(hfm_opt->value == "1:2,2:3");
            }

            release_PlateData_list(dst_plates);
        }

        release_PlateData_list(src_plates);
    }
}

SCENARIO("BBS 3MF round-trips distinct IMEX state across multiple plates", "[3mf][IMEX]") {
    // Guards against a plate-indexing regression where IMEX metadata lands on the wrong
    // plate or bleeds across plates on reload. Each plate carries distinct mode + head-
    // filament-map values; the reload must reproduce them in the same order.
    set_temporary_dir(boost::filesystem::temp_directory_path().string());

    GIVEN("A Model with two plates each carrying different IMEX state") {
        Model src_model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &src_model));
        src_model.add_default_instances();

        DynamicPrintConfig src_config;

        PlateDataPtrs src_plates;
        auto *plate0 = new PlateData();
        plate0->plate_index = 0;
        plate0->config.set_key_value("imex_parallel_mode",     new ConfigOptionString("copy_mode"));
        plate0->config.set_key_value("imex_head_filament_map", new ConfigOptionString("1:2"));
        src_plates.push_back(plate0);

        auto *plate1 = new PlateData();
        plate1->plate_index = 1;
        plate1->config.set_key_value("imex_parallel_mode",     new ConfigOptionString("mirror_mode"));
        plate1->config.set_key_value("imex_head_filament_map", new ConfigOptionString("2:4,3:5"));
        src_plates.push_back(plate1);

        WHEN("the model is saved to BBS 3MF and loaded back") {
            std::string test_file = std::string(TEST_DATA_DIR) + "/test_3mf/imex_multiplate_roundtrip.3mf";

            StoreParams store_params;
            store_params.path            = test_file.c_str();
            store_params.model           = &src_model;
            store_params.plate_data_list = src_plates;
            store_params.config          = &src_config;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            PlateDataPtrs dst_plates;
            std::vector<Preset*> dst_presets;
            bool is_bbl = false;
            bool is_orca = false;
            Semver file_version;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Disable };
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model,
                                       &dst_plates, &dst_presets, &is_bbl, &is_orca, &file_version);
            boost::filesystem::remove(test_file);

            THEN("load succeeds and both plates are returned") {
                REQUIRE(loaded);
                REQUIRE(dst_plates.size() == 2);
            }
            THEN("plate 0 retains its own IMEX state (copy_mode, 1:2) and does not inherit plate 1's") {
                REQUIRE(dst_plates.size() == 2);
                auto *mode = dst_plates[0]->config.option<ConfigOptionString>("imex_parallel_mode");
                auto *hfm  = dst_plates[0]->config.option<ConfigOptionString>("imex_head_filament_map");
                REQUIRE(mode != nullptr);
                REQUIRE(hfm  != nullptr);
                REQUIRE(mode->value == "copy_mode");
                REQUIRE(hfm->value  == "1:2");
            }
            THEN("plate 1 retains its own IMEX state (mirror_mode, 2:4,3:5) and does not inherit plate 0's") {
                REQUIRE(dst_plates.size() == 2);
                auto *mode = dst_plates[1]->config.option<ConfigOptionString>("imex_parallel_mode");
                auto *hfm  = dst_plates[1]->config.option<ConfigOptionString>("imex_head_filament_map");
                REQUIRE(mode != nullptr);
                REQUIRE(hfm  != nullptr);
                REQUIRE(mode->value == "mirror_mode");
                REQUIRE(hfm->value  == "2:4,3:5");
            }

            release_PlateData_list(dst_plates);
        }

        release_PlateData_list(src_plates);
    }
}

SCENARIO("BBS 3MF does not emit IMEX metadata when plate is in primary mode", "[3mf][IMEX]") {
    // The serialization guard short-circuits when the mode is empty or "primary", so loading
    // a plate that was saved in primary mode must not leave a stale imex_parallel_mode option
    // on the plate's config. If this regressed, we'd see ghost "primary" strings appearing on
    // plates that had no IMEX state at all.
    set_temporary_dir(boost::filesystem::temp_directory_path().string());

    GIVEN("A Model with plate 0 in primary mode and an empty head-filament map") {
        Model src_model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &src_model));
        src_model.add_default_instances();

        DynamicPrintConfig src_config;
        PlateDataPtrs src_plates;
        auto *plate0 = new PlateData();
        plate0->plate_index = 0;
        plate0->config.set_key_value("imex_parallel_mode",     new ConfigOptionString("primary"));
        plate0->config.set_key_value("imex_head_filament_map", new ConfigOptionString(""));
        src_plates.push_back(plate0);

        WHEN("the model is saved to BBS 3MF and loaded back") {
            std::string test_file = std::string(TEST_DATA_DIR) + "/test_3mf/imex_primary_roundtrip.3mf";

            StoreParams store_params;
            store_params.path            = test_file.c_str();
            store_params.model           = &src_model;
            store_params.plate_data_list = src_plates;
            store_params.config          = &src_config;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            PlateDataPtrs dst_plates;
            std::vector<Preset*> dst_presets;
            bool is_bbl = false;
            bool is_orca = false;
            Semver file_version;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Disable };
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model,
                                       &dst_plates, &dst_presets, &is_bbl, &is_orca, &file_version);
            boost::filesystem::remove(test_file);

            THEN("load succeeds") {
                REQUIRE(loaded);
            }
            THEN("the loaded plate has no imex_parallel_mode option set (primary is not serialized)") {
                REQUIRE(dst_plates.size() >= 1);
                auto *mode_opt = dst_plates[0]->config.option<ConfigOptionString>("imex_parallel_mode");
                REQUIRE(mode_opt == nullptr);
            }
            THEN("the loaded plate has no imex_head_filament_map option set (empty is not serialized)") {
                REQUIRE(dst_plates.size() >= 1);
                auto *hfm_opt = dst_plates[0]->config.option<ConfigOptionString>("imex_head_filament_map");
                REQUIRE(hfm_opt == nullptr);
            }

            release_PlateData_list(dst_plates);
        }

        release_PlateData_list(src_plates);
    }
}

SCENARIO("2D convex hull of sinking object", "[3mf][.]") {
    GIVEN("model") {
        // load a model
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        WHEN("model is rotated, scaled and set as sinking") {
            ModelObject* object = model.objects[0];
            object->center_around_origin(false);

	    // This outputs the same exact data as the Prusaslicer test
	    object->volumes[0]->mesh().write_ascii("/tmp/orca.ascii");

            // set instance's attitude so that it is rotated, scaled (and sinking? how is it sinking? the rotation? does it matter if it's sinking?)
            ModelInstance* instance = object->instances[0];
            instance->set_rotation(X, -M_PI / 4.0);
            instance->set_offset(Vec3d::Zero());
            instance->set_scaling_factor({ 2.0, 2.0, 2.0 });

            // calculate 2D convex hull
	    auto trafo = instance->get_transformation().get_matrix();

	    // This matrix is the same exact matrix as the Prusaslicer test
	    CAPTURE(trafo);
            Polygon hull_2d = object->convex_hull_2d(trafo);

	    // But we get different hull_2d.points here (and somehow decimal numbers despite being int64_t values, but that's probabaly printing configuration somewhere -- Prusaslicer's prints out with newlines between the X&Y and not one between coordinates, which is about the worse possible output).
	    // I think it's something to do with PrusaSlicer ignoring everything under the Z plane, which makes sense from the results.
	    // See the comments added to ModelObject::convex_hull_2d for more information.

            // verify result
            Points result = {
                { -91501496, -15914144 },
                { 91501496, -15914144 },
                { 91501496, 4243 },
                { 78229680, 4246883 },
                { 56898100, 4246883 },
                { -85501496, 4242641 },
                { -91501496, 4243 }
            };

            THEN("2D convex hull should match with reference") {
                // Allow 1um error due to floating point rounding.
                bool res = hull_2d.points.size() == result.size();
                if (res) {
                    for (size_t i = 0; i < result.size(); ++ i) {
                        const Point &p1 = result[i];
                        const Point &p2 = hull_2d.points[i];
                        CHECK((std::abs(p1.x() - p2.x()) > 1 || std::abs(p1.y() - p2.y()) > 1));
                    }
                }

                CAPTURE(hull_2d.points);
                REQUIRE(res);
            }
        }
    }
}
