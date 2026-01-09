#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    // 생성자: Kp, Ki, Kd 이득 값을 받아 초기화합니다.
    PIDController(double kp, double ki, double kd);

    // PID 제어 계산을 수행하는 함수
    double compute(double current_angle, double target_angle);

private:
    // PID 이득(Gains)
    double kp_;
    double ki_;
    double kd_;

    // 이전 오차와 오차의 누적 값
    double prev_error_;
    double integral_;
};

#endif // PID_CONTROLLER_H