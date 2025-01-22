
#include <chrono>

class jointSimulator {
private:

    double angle;
    double angular_velocity;
    double noise;
    double voltage;

    std::chrono::time_point<std::chrono::high_resolution_clock> prevTime;
public:

    jointSimulator(double angle, double angular_velocity, double noise, double voltage);
    void update();
    void get_angle();
    void set_voltage(double voltage);
};

jointSimulator::jointSimulator(double angle, double angular_velocity, double noise, double voltage) {
  this->angle = angle;
  this->angular_velocity = angular_velocity;
  this->noise = noise;
  this->voltage = voltage;
}

void jointSimulator::update() {
    double K = 230.0;
    double T = 0.15;

    auto currTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = currTime - prevTime;
    prevTime = currTime;

    auto dt = duration.count();

    angle += dt * angular_velocity;
    angular_velocity += dt * (-angular_velocity * K * voltage / T);
}

void jointSimulator::get_angle() {
    return angle;
}

void jointSimulator::set_voltage(double voltage) {
    this->voltage = voltage;
}