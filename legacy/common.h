//
// Created by cryscan on 8/18/21.
//

#ifndef TO_IHC_2_COMMON_H
#define TO_IHC_2_COMMON_H

#include <memory>
#include <type_traits>

#include "../biped/declarations.h"
#include "../biped/transforms.h"
#include "../biped/jacobians.h"
#include "../biped/inertia_properties.h"
#include "../biped/jsim.h"
#include "../biped/inverse_dynamics.h"

#define ASSIGN_VECTOR(to, from, it, size) (to) = (from).segment<(size)>(it); (it) += (size);
#define ASSIGN_COLS(to, from, it, size) (to) = (from).middleCols<(size)>(it); (it) += (size);
#define FILL_VECTOR(to, from, it, size) (to).segment<(size)>(it) = (from); (it) += (size);
#define FILL_ROWS(to, from, it, size) (to).middleRows<(size)>(it) = (from); (it) += (size);

#define MATRIX_AS_VECTOR(mat) Eigen::Map<Eigen::VectorXd>((mat).data(), (mat).size())
#define MATRIX_AS_VECTOR_AD(mat) Eigen::Map<ADVector>((mat).data(), (mat).size())

// change this to switch robot model
namespace Robot = Biped;

static constexpr int joint_space_dims = Robot::rcg::JointSpaceDimension;
static constexpr int state_dims = joint_space_dims + joint_space_dims;
static constexpr int action_dims = 2;
static constexpr int num_contacts = 3;
static constexpr int contact_dims = 3 * num_contacts;

using JointState = Eigen::Matrix<double, joint_space_dims, 1>;
using State = Eigen::Matrix<double, state_dims, 1>;
using Action = Eigen::Matrix<double, action_dims, 1>;
using Contact = Eigen::Matrix<double, num_contacts, 1>;

enum class EvalOption {
    ZERO_ORDER,
    FIRST_ORDER,
};

template<typename T>
struct Parameter {
};

template<typename T, int InputDims, int ParamDims, int OutputDims>
struct ADBase {
    using Params = Parameter<T>;

    using Scalar = Robot::rcg::Scalar;
    using ScalarTraits = Robot::rcg::ScalarTraits;
    using JointState = Robot::rcg::JointState;

    using State = Robot::rcg::Matrix<state_dims, 1>;
    using Action = Robot::rcg::Matrix<action_dims, 1>;
    using Contact = Robot::rcg::Matrix<num_contacts, 1>;

    static constexpr int input_dims = InputDims;
    static constexpr int param_dims = ParamDims;
    static constexpr int output_dims = OutputDims;

    explicit ADBase(const std::string& name) :
            name(name),
            jacobian_name(name + "_jacobian"),
            library_name(name + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION) {}

    ADBase(const ADBase& other) :
            params(other.params),
            name(other.name),
            jacobian_name(other.jacobian_name),
            library_name(other.library_name) {}

    virtual void build_map() {
        this->build_map();

        using namespace CppAD::cg;
        if (!system::isFile(library_name)) {
            std::cout << "Compiling " << library_name << std::endl;
            compile_library();
        } else {
            std::cout << "Loading " << library_name << std::endl;
            load_library();
        }
    };

    void evaluate(EvalOption option = EvalOption::FIRST_ORDER) { this->evaluate(params, option); }
    virtual void evaluate(const Params&, EvalOption option) {};

    static constexpr std::remove_pointer<ADBase<T, input_dims, param_dims, output_dims>>
    base_type() {};

    Params params;

protected:
    const std::string name;
    const std::string jacobian_name;
    const std::string library_name;

    using ScalarVector = Robot::rcg::Matrix<Eigen::Dynamic, 1>;
    using ADVector = Eigen::Matrix<typename ScalarTraits::AD, Eigen::Dynamic, 1>;

    ScalarVector ad_x{input_dims + param_dims};
    ScalarVector ad_y{output_dims};

    CppAD::ADFun<ScalarTraits::ValueType> ad_fun, ad_fun_;

    std::unique_ptr<CppAD::cg::DynamicLib<double>> lib;
    std::unique_ptr<CppAD::cg::GenericModel<double>> models[2];

    void build_jacobian() {
        ADVector ad_x_ = ad_x.cast<typename ScalarTraits::AD>();
        ADVector ad_y_(output_dims * input_dims);

        CppAD::Independent(ad_x_);

        using FullJacobian = Eigen::Matrix<Scalar, output_dims, input_dims + param_dims, Eigen::RowMajor>;
        using Jacobian = Eigen::Matrix<Scalar, output_dims, input_dims, Eigen::RowMajor>;

        FullJacobian full_jacobian;
        MATRIX_AS_VECTOR_AD(full_jacobian) = ad_fun.base2ad().Jacobian(ad_x_);

        Jacobian jacobian = full_jacobian.template leftCols<input_dims>();
        ad_y_ << MATRIX_AS_VECTOR_AD(jacobian);

        ad_fun_.Dependent(ad_y_);
        ad_fun_.optimize("no_compare_op");
    }

    void compile_library(CppAD::cg::ModelLibraryCSourceGen<double>& library_c_source_gen) {
        using namespace CppAD::cg;

        DynamicModelLibraryProcessor<double> processor(library_c_source_gen, name);
        GccCompiler<double> compiler;

        SaveFilesModelLibraryProcessor<double> save_files(library_c_source_gen);
        save_files.saveSources();

        lib = processor.createDynamicLibrary(compiler);
    }

    void compile_library() {
        using namespace CppAD::cg;

        if constexpr (input_dims == 0) {
            ModelCSourceGen<double> c_source_gen(ad_fun, name);
            ModelLibraryCSourceGen<double> library_c_source_gen(c_source_gen);
            compile_library(library_c_source_gen);
        } else {
            ModelCSourceGen<double> c_source_gen[2] = {{ad_fun,  name},
                                                       {ad_fun_, jacobian_name}};
            ModelLibraryCSourceGen<double> library_c_source_gen(c_source_gen[0], c_source_gen[1]);
            compile_library(library_c_source_gen);
        }

        models[0] = lib->model(name);
        if constexpr(input_dims != 0) models[1] = lib->model(jacobian_name);
    }

    void load_library() {
        using namespace CppAD::cg;
        lib = std::make_unique<LinuxDynamicLib<double>>(library_name);
        models[0] = lib->model(name);
        if (input_dims != 0) models[1] = lib->model(jacobian_name);
    }
};

#endif //TO_IHC_2_COMMON_H
