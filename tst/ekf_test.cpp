#include <gtest/gtest.h>
#include "../src/ekf.h"
// g++ -I /usr/local/Eigen -Wall -g -pthread ekf_test.cpp -lgtest_main  -lgtest -lpthread 

TEST(EKFHelpers, TestSkewSym) {
	Eigen::Matrix<float,3,1> v;
	Eigen::Matrix<float,3,3> M;

	v << 1, 2, 3;
	to_skew(v,M);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}