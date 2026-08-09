#include <Eigen/Dense>
#include <iostream>

int main() {
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
    std::cout << "Eigen linked. 4x4 identity:\n" << P << "\n";
    return 0;
}
