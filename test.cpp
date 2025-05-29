#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <regex>


struct test_pose
{
    int x = 0;
    int y= 0;
    int z = 0;
    int w = 0;
};

std::vector<test_pose> parseGcodeFile(const std::string &filename) {
    std::vector<test_pose> poses;
    std::ifstream gcodeFile(filename);

    if (!gcodeFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return poses;
    }

    std::string line;
    std::regex coordinateRegex(R"(X([-\d\.]+)\s*Y([-\d\.]+)\s*Z([-\d\.]+)?)");

    while (std::getline(gcodeFile, line)) {
        // Ignore comments or empty lines
        if (line.empty() || line[0] == ';') {
            continue;
        }

        std::istringstream stream(line);
        std::string token;
        double x = 0.0, y = 0.0, z = 0.0;
        bool has_x = false, has_y = false, has_z = false;

        while (stream >> token) {
            // Parse tokens for X, Y, Z
            if (token[0] == 'X') {
                x = std::stod(token.substr(1));
                has_x = true;
            } else if (token[0] == 'Y') {
                y = std::stod(token.substr(1));
                has_y = true;
            } else if (token[0] == 'Z') {
                z = std::stod(token.substr(1));
                has_z = true;
            }
        }

        // Create a Pose if we have at least X and Y
        if (has_x || has_y || has_z) {
            test_pose pose;
            pose.x = x;
            pose.y = y;
            pose.z = z;
            pose.w = 1.0;

            poses.push_back(pose);
        }
    }


    gcodeFile.close();
    return poses;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <gcode_file>" << std::endl;
        return 1;
    }

    std::string gcodeFilename = argv[1];
    auto poses = parseGcodeFile(gcodeFilename);

    std::cout << "Extracted " << poses.size() << " poses from " << gcodeFilename << std::endl;
    for (const auto &pose : poses) {
        std::cout << "Pose: X=" << pose.x
                  << ", Y=" << pose.y
                  << ", Z=" << pose.z
                  << std::endl;
    }

    // In a ROS 2 node, you would now pass the poses vector to MoveIt for Cartesian path planning.
    return 0;
}
