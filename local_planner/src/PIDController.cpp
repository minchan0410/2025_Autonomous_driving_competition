#include "PIDController.h"

// 생성자 구현
PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {
    // 멤버 초기화 리스트를 사용하여 변수를 초기화합니다.
    // Python의 __init__과 동일한 역할을 합니다.
}

// compute 함수 구현
double PIDController::compute(double current_angle, double target_angle) {
    // 1. 오차(Error) 계산
    double error = target_angle - current_angle;

    // 2. 오차 누적(Integral)
    integral_ += error;

    // 3. 오차 변화율(Derivative) 계산
    double derivative = error - prev_error_;

    // 4. 다음 계산을 위해 현재 오차를 이전 오차로 저장
    prev_error_ = error;

    // 5. PID 제어 출력 계산 및 반환
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}