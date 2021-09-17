//
// Created by cryscan on 9/16/21.
//

#ifndef TO_IHC_2_ID_H
#define TO_IHC_2_ID_H

#include "common.h"

struct IDController;

template<>
struct Parameter<IDController> {
    State x;
    Action u;
    double d[num_contacts];
};

#define INPUT_DIMS  (state_dims + action_dims + num_contacts)
#define OUTPUT_DIMS state_dims

struct IDController : public ADBase<IDController, INPUT_DIMS, OUTPUT_DIMS> {
};

#undef INPUT_DIMS
#undef OUTPUT_DIMS

#endif //TO_IHC_2_ID_H
