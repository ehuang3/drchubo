#include <iostream>
#include <gtest/gtest.h>

using namespace std;

/* ********************************************************************************************* */
TEST(EXAMPLE, FIRST_TEST) {
	cout << "Hello Test!" << endl;
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
