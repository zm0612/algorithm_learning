#include <Eigen/Dense>
#include <iostream>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

//#define USE_OSQP
#ifdef USE_OSQP

#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

#endif

#include <chrono>

using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

VecXd RunMPC(unsigned int N, VecXd &init_x, MatXd &A, MatXd &B,
             MatXd &Q, MatXd &R, MatXd &F) {

    unsigned int num_state = init_x.rows();
    unsigned int num_control = B.cols();

    MatXd C, M;
    C.resize((N + 1) * num_state, num_control * N);
    C.setZero();
    M.resize((N + 1) * num_state, num_state);
    Eigen::MatrixXd temp;
    temp.resize(num_state, num_state);
    temp.setIdentity();
    M.block(0, 0, num_state, num_state).setIdentity();
    for (unsigned int i = 1; i <= N; ++i) {
        Eigen::MatrixXd temp_c;
        temp_c.resize(num_state, (N + 1) * num_control);
        temp_c << temp * B, C.block(num_state * (i - 1), 0, num_state, C.cols());

        C.block(num_state * i, 0, num_state, C.cols())
                = temp_c.block(0, 0, num_state, temp_c.cols() - num_control);

        temp = A * temp;
        M.block(num_state * i, 0, num_state, num_state) = temp;
    }

    Eigen::MatrixXd Q_bar, R_bar;

    Q_bar.resize(num_state * (N + 1), num_state * (N + 1));
    Q_bar.setZero();
    for (unsigned int i = 0; i < N; ++i) {
        Q_bar.block(num_state * i, num_state * i, num_state, num_state) = Q;
    }
    Q_bar.block(num_state * N, num_state * N, num_state, num_state) = F;

    R_bar.resize(N * num_control, N * num_control);
    R_bar.setZero();
    for (unsigned int i = 0; i < N; ++i) {
        R_bar.block(i * num_control, i * num_control, num_control, num_control) = R;
    }


    Eigen::MatrixXd G = M.transpose() * Q_bar * M;
    Eigen::MatrixXd E = C.transpose() * Q_bar * M;
    Eigen::MatrixXd H = C.transpose() * Q_bar * C + R_bar;

    return H.inverse() * (-E * init_x);

}

int main() {
    MatXd A, B;
    A.resize(2, 2);
    B.resize(2, 1);
    A << 1, 0.1, 0, 2;
    B << 0, 0.5;

    unsigned int num_state = 2;
    Eigen::MatrixXd Q, R, F;
    Q.resize(num_state, num_state);
    Q << 1, 0, 0, 1;

    R.resize(1, 1);
    R << 0.1;

    F.resize(num_state, num_state);
    F << 2, 0, 0, 2;

    const unsigned int N = 3;
    VecXd init_x;
    init_x.resize(2, 1);
    init_x << 5.0, 5.0;

    std::vector<double> state_0;
    std::vector<double> time;
    state_0.emplace_back(init_x.x());
    time.emplace_back(0.0);

    for (unsigned int i = 0; i < 200; ++i) {
        std::cout << "error: " << init_x.transpose() << std::endl;
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        VecXd control = RunMPC(N, init_x, A, B, Q, R, F);
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> used_time = (end_time - start_time);
        std::cout << "one mpc use time(ms): " << used_time.count() * 1000 << std::endl;
        init_x = A * init_x + B * control.x();
        state_0.emplace_back(init_x.x());
        time.emplace_back(0.1 * (i + 1));
    }

    plt::plot(time, state_0, "ro");
    plt::xlim(-0.0, 20.0);
    plt::ylim(-0.0, 7.0);
    plt::title("MPC");
    plt::show();

//    std::cout << "closed form u: " << control.transpose() << std::endl;

    return 0;
}
