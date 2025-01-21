
class jointSimulator {
private:

    double angle;
    double angular_velocity;
    double noise;
    double voltage;
public:

    jointSimulator(double angle, double angular_velocity, double noise, double voltage);
    void update();
    void get_angle();
};

jointSimulator::jointSimulator(double angle, double angular_velocity, double noise, double voltage) {
  this->angle = angle;
  this->angular_velocity = angular_velocity;
  this->noise = noise;
  this->voltage = voltage;
}

void jointSimulator::update() {

}