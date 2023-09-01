#include "reference_sdf_generator/reference_sdf_generator.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reference_sdf_generator");
    reference_sdf_generator::ReferenceSDFGenerator reference_sdf_generator;
    ros::spin();
    return 0;
};
