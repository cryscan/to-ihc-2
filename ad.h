//
// Created by cryscan on 10/28/21.
//

#ifndef TO_IHC_2_AD_H
#define TO_IHC_2_AD_H

#include <memory>
#include <cppad/cg.hpp>

#include "biped/rbd_types.h"

#define DEF_PARAMETER_FILL(...) \
template <typename Vector> \
void fill(Eigen::MatrixBase<Vector>& vector) const { \
    vector << __VA_ARGS__;    \
}

#define ASSIGN_SEGMENT(to, from, it, size) (to) << (from).template segment<(size)>(it); (it) += (size);
#define FILL_SEGMENT(to, from, it, size) (to).template segment<(size)>(it) << (from); (it) += (size);

template<typename T, typename ValueType>
struct Parameter {
};

template<typename Derived, int InputDims, int ParamDims, int OutputDims, typename ValueType, bool ComputeJacobian = false>
class ADBase {
public:
    enum EvalOption {
        ZERO_ORDER,
        FIRST_ORDER,
    };

    using Params = Parameter<Derived, ValueType>;
    static constexpr int input_dims = InputDims;
    static constexpr int param_dims = ParamDims;
    static constexpr int output_dims = OutputDims;

    static constexpr bool compute_jacobian = ComputeJacobian && (input_dims != 0);

    using ValueVector = Eigen::Matrix<ValueType, Eigen::Dynamic, 1>;

    using CG = CppAD::cg::CG<ValueType>;
    using AD = CppAD::AD<CG>;

    using ADVector = Eigen::Matrix<AD, Eigen::Dynamic, 1>;
    using ADFun = CppAD::ADFun<CG>;

    explicit ADBase(const std::string& name) :
            name(name),
            jacobian_name(name + "_jacobian"),
            library_name(name + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION),
            ad_x(input_dims + param_dims) {}

    ADBase(const ADBase& other) :
            name(other.name),
            jacobian_name(other.jacobian_name),
            library_name(other.library_name),
            ad_x(other.ad_x) {}

    Params params;

    inline void build_map() {
        using namespace CppAD::cg;
        if (!system::isFile(library_name)) {
            build_zero();
            if constexpr(compute_jacobian) build_jacobian();

            std::cout << "Compiling " << library_name << std::endl;
            compile_library();
        } else {
            std::cout << "Loading " << library_name << std::endl;
            load_library();
        }

        models[0] = lib->model(name);
        if constexpr (compute_jacobian) models[1] = lib->model(jacobian_name);
    }

    inline void evaluate(EvalOption option = FIRST_ORDER) {
        Eigen::Matrix<ValueType, Eigen::Dynamic, 1> x(input_dims + param_dims);
        params.template fill(x);
        f << models[0]->ForwardZero(x);

        if (compute_jacobian && option == FIRST_ORDER) {
            Eigen::Map<ValueVector>(df.data(), df.size()) << models[1]->ForwardZero(x);
        }
    }

    Eigen::Matrix<ValueType, output_dims, 1> f;
    Eigen::Matrix<ValueType, output_dims, input_dims, Eigen::RowMajor> df;

protected:
    const std::string name;
    const std::string jacobian_name;
    const std::string library_name;

    ADVector ad_x;
    ADFun ad_fun[2];

    std::unique_ptr<CppAD::cg::DynamicLib<ValueType>> lib;
    std::unique_ptr<CppAD::cg::GenericModel<ValueType>> models[2];

    virtual void build_zero() = 0;

    inline void build_jacobian() {
        using FullJacobian = Eigen::Matrix<AD, output_dims, input_dims + param_dims, Eigen::RowMajor>;
        using Jacobian = Eigen::Matrix<AD, output_dims, input_dims, Eigen::RowMajor>;

        ADVector ad_y(output_dims * input_dims);
        CppAD::Independent(ad_x);

        FullJacobian full_jacobian;
        Eigen::Map<ADVector>(full_jacobian.data(), full_jacobian.size()) << ad_fun[0].base2ad().template Jacobian(ad_x);

        Jacobian jacobian = full_jacobian.template leftCols<input_dims>();
        ad_y << Eigen::Map<ADVector>(jacobian.data(), jacobian.size());

        ad_fun[1].template Dependent(ad_y);
        ad_fun[1].optimize("no_compare_op");
    }

    void compile_library() {
        using namespace CppAD::cg;

        ModelCSourceGen<ValueType> c_source_gen[2] = {{ad_fun[0], name},
                                                      {ad_fun[1], jacobian_name}};

        ModelLibraryCSourceGen<ValueType> library_c_source_gen(c_source_gen[0]);
        if constexpr (compute_jacobian) library_c_source_gen.addModel(c_source_gen[1]);

        DynamicModelLibraryProcessor<ValueType> processor(library_c_source_gen, name);
        GccCompiler<ValueType> compiler;

        SaveFilesModelLibraryProcessor<ValueType> save_files(library_c_source_gen);
        save_files.saveSources();

        lib = processor.createDynamicLibrary(compiler);
    }

    void load_library() {
        using namespace CppAD::cg;
        lib = std::make_unique<LinuxDynamicLib<ValueType>>(library_name);
    }
};

#endif //TO_IHC_2_AD_H
