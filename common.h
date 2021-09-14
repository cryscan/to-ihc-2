//
// Created by cryscan on 8/18/21.
//

#ifndef TO_IHC_2_COMMON_H
#define TO_IHC_2_COMMON_H

#include <memory>
#include <type_traits>

#include "hopper/declarations.h"
#include "hopper/transforms.h"
#include "hopper/jacobians.h"
#include "hopper/inertia_properties.h"
#include "hopper/jsim.h"
#include "hopper/inverse_dynamics.h"

// change this to switch robot model
namespace Robot = Hopper;

static constexpr int joint_space_dims = Hopper::rcg::JointSpaceDimension;
static constexpr int state_dims = joint_space_dims + joint_space_dims;
static constexpr int action_dims = 2;

using State = Eigen::Matrix<double, state_dims, 1>;
using Action = Eigen::Matrix<double, action_dims, 1>;

enum class EvalOption {
    ZERO_ORDER,
    FIRST_ORDER,
};

template<typename T>
struct Parameter {
};

template<typename T, int InputDims, int OutputDims>
struct ADBase {
    using Params = Parameter<T>;

    using Scalar = Robot::rcg::Scalar;
    using ScalarTraits = Robot::rcg::ScalarTraits;
    using JointState = Robot::rcg::JointState;
    using Action = Robot::rcg::Matrix<action_dims, 1>;

    static constexpr int input_dims = InputDims;
    static constexpr int output_dims = OutputDims;

    explicit ADBase(const std::string& name) :
            name(name),
            library_name(name + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION) {}

    ADBase(const ADBase& other) : params(other.params), name(other.name), library_name(other.library_name) {}

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

    static constexpr std::remove_pointer<ADBase<T, input_dims, output_dims>> base_type() {};

    Params params;

protected:
    const std::string name;
    const std::string library_name;

    Robot::rcg::Matrix<Eigen::Dynamic, 1> ad_x{input_dims};
    Robot::rcg::Matrix<Eigen::Dynamic, 1> ad_y{output_dims};
    CppAD::ADFun<ScalarTraits::ValueType> ad_fun;

    std::unique_ptr<CppAD::cg::DynamicLib<double>> lib;
    std::unique_ptr<CppAD::cg::GenericModel<double>> model;

    void compile_library() {
        using namespace CppAD::cg;

        ModelCSourceGen<double> c_source_gen(ad_fun, name);
        c_source_gen.setCreateForwardZero(true);
        c_source_gen.setCreateJacobian(true);

        ModelLibraryCSourceGen<double> library_c_source_gen(c_source_gen);
        DynamicModelLibraryProcessor<double> processor(library_c_source_gen, name);
        GccCompiler<double> compiler;

        lib = processor.createDynamicLibrary(compiler);
        model = lib->model(name);
    }

    void load_library() {
        using namespace CppAD::cg;
        lib = std::make_unique<LinuxDynamicLib<double>>(library_name);
        model = lib->model(name);
    }
};

#define ASSIGN_VECTOR(to, from, it, size) (to) = (from).segment<(size)>(it); (it) += (size);
#define ASSIGN_COLS(to, from, it, size) (to) = (from).middleCols<(size)>(it); (it) += (size);
#define FILL_VECTOR(to, from, it, size) (to).segment<(size)>(it) = (from); (it) += (size);

#endif //TO_IHC_2_COMMON_H
