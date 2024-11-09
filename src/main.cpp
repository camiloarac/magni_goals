#include "magni_goals/magni_goals.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "magni_goals");

    MagniGoals magni_goals{};

    magni_goals.run();
    return 0;
}