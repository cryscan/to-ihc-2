//
// Created by cryscan on 10/11/21.
//

#ifndef TO_IHC_2_SLIP_H
#define TO_IHC_2_SLIP_H

#include <eigen3/Eigen/Core>

class SLIP {
public:
    struct State {
        double x, z, dx, dz;
    };

    SLIP(double dt, double mass, double l0, double k);

    void set_theta(double theta);
    void set_state(const State& state);

    [[nodiscard]] State get_state() const;
    [[nodiscard]] bool get_stance() const { return stance; }

    [[nodiscard]] double contact_position() const { return x0; }

    [[nodiscard]] double apex_height() const;
    [[nodiscard]] double apex_velocity() const;

    void step();

    const double dt;

    const double mass;
    const double l0;
    const double k;

private:
    double theta;

    Eigen::Vector2d q, u;

    bool stance;
    double x0;
};

#endif //TO_IHC_2_SLIP_H
