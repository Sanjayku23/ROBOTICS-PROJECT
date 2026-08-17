
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

// 1. DATA STRUCTURES
struct DH_Param {
    double theta; // Joint angle (radians)
    double d;     // Link offset (mm)
    double a;     // Link length (mm)
    double alpha; // Link twist (radians)
};

struct Matrix4 {
    double data[4][4];
    
    Matrix4() {
        for(int i = 0; i < 4; i++)
            for(int j = 0; j < 4; j++)
                data[i][j] = (i == j) ? 1.0 : 0.0; 
    }

    Matrix4 operator*(const Matrix4& other) const {
        Matrix4 result;
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                result.data[i][j] = 0;
                for(int k = 0; k < 4; ++k) {
                    result.data[i][j] += data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }
};

// 2. FORWARD KINEMATICS FUNCTIONS
Matrix4 computeDHTransform(const DH_Param& dh) {
    Matrix4 T;
    double ct = cos(dh.theta);
    double st = sin(dh.theta);
    double ca = cos(dh.alpha);
    double sa = sin(dh.alpha);

    T.data[0][0] = ct;  T.data[0][1] = -st * ca; T.data[0][2] = st * sa;  T.data[0][3] = dh.a * ct;
    T.data[1][0] = st;  T.data[1][1] = ct * ca;  T.data[1][2] = -ct * sa; T.data[1][3] = dh.a * st;
    T.data[2][0] = 0;   T.data[2][1] = sa;       T.data[2][2] = ca;       T.data[2][3] = dh.d;
    T.data[3][0] = 0;   T.data[3][1] = 0;        T.data[3][2] = 0;        T.data[3][3] = 1;

    return T;
}

Matrix4 computeArmFK(const std::vector<DH_Param>& arm) {
    Matrix4 pose;
    for(size_t i = 0; i < arm.size(); ++i) {
        pose = pose * computeDHTransform(arm[i]);
    }
    return pose;
}

// 3. INVERSE KINEMATICS SOLVER (UPDATED WITH STABILITY FIXES)
bool solveIK(std::vector<DH_Param>& arm, double targetX, double targetY, double targetZ, double tolerance = 0.5, int max_iter = 50000) {
    // Drastically reduced learning rate for stability in mm-scale environments
    double learning_rate = 0.00001; 
    double delta = 0.001;          

    // Give a non-zero initial seed to break geometric singularity
    for (size_t i = 0; i < arm.size(); ++i) {
        if (arm[i].theta == 0.0) {
            arm[i].theta = 0.1 * (i + 1); 
        }
    }

    for (int iter = 0; iter < max_iter; ++iter) {
        Matrix4 currentPose = computeArmFK(arm);
        double currentX = currentPose.data[0][3];
        double currentY = currentPose.data[1][3];
        double currentZ = currentPose.data[2][3];

        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        double errorZ = targetZ - currentZ;
        
        double distance = sqrt(errorX * errorX + errorY * errorY + errorZ * errorZ);

        if (distance < tolerance) {
            std::cout << "[IK Success] Converged in " << iter << " iterations.\n";
            return true; 
        }

        // Print progress every 5000 iterations to watch it stabilize
        if (iter % 5000 == 0) {
            std::cout << "Iter: " << iter << " | Distance to target: " << std::fixed << std::setprecision(2) << distance << " mm\n";
        }

        for (size_t i = 0; i < arm.size(); ++i) {
            double orig_theta = arm[i].theta;

            arm[i].theta += delta;
            Matrix4 nudgedPose = computeArmFK(arm);
            
            double gradX = (nudgedPose.data[0][3] - currentX) / delta;
            double gradY = (nudgedPose.data[1][3] - currentY) / delta;
            double gradZ = (nudgedPose.data[2][3] - currentZ) / delta;

            arm[i].theta = orig_theta;

            // Calculate raw update
            double directional_gradient = errorX * gradX + errorY * gradY + errorZ * gradZ;
            double update = learning_rate * directional_gradient;

            // Gradient Clamping: Prevent the update from exceeding 0.05 radians per iteration
            double max_step = 0.05; 
            if (update > max_step) update = max_step;
            if (update < -max_step) update = -max_step;

            arm[i].theta += update;
        }
    }
    
    Matrix4 finalPose = computeArmFK(arm);
    double finalDist = sqrt(pow(targetX - finalPose.data[0][3], 2) + pow(targetY - finalPose.data[1][3], 2) + pow(targetZ - finalPose.data[2][3], 2));
    std::cout << "[IK Failed] Iteration limit reached. Stopped at distance: " << finalDist << " mm\n";
    
    return false;
}

// 4. MAIN EXECUTION
int main() {
    // Initial arm state
    std::vector<DH_Param> robot_arm = {
        {0.0, 150.0, 0.0,   M_PI/2},  // Joint 1
        {0.0, 0.0,   431.8, 0.0},     // Joint 2
        {0.0, 0.0,   20.3,  -M_PI/2}, // Joint 3
        {0.0, 431.8, 0.0,   M_PI/2},  // Joint 4
        {0.0, 0.0,   0.0,   -M_PI/2}, // Joint 5
        {0.0, 50.0,  0.0,   0.0}      // Joint 6
    };

    // Target coordinates in 3D space (X, Y, Z in mm)
    double targetX = 150.0;
    double targetY = 100.0;
    double targetZ = 200.0;

    std::cout << "Target Coordinates -> X: " << targetX << " mm, Y: " << targetY << " mm, Z: " << targetZ << " mm\n\n";

    bool solved = solveIK(robot_arm, targetX, targetY, targetZ);

    if (solved) {
        std::cout << "\n--- Calculated Joint Angles (Radians) ---\n";
        for (size_t i = 0; i < robot_arm.size(); ++i) {
            std::cout << "Joint " << i + 1 << ": " << std::fixed << std::setprecision(4) << robot_arm[i].theta << " rad\n";
        }

        Matrix4 verifyPose = computeArmFK(robot_arm);
        std::cout << "\n--- Verification (FK on Calculated Angles) ---\n";
        std::cout << "Reached X: " << verifyPose.data[0][3] << " mm\n";
        std::cout << "Reached Y: " << verifyPose.data[1][3] << " mm\n";
        std::cout << "Reached Z: " << verifyPose.data[2][3] << " mm\n";
    }

    return 0;
}