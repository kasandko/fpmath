#include "operators_tests.h"

#include "tests.h"
#include "../../include/fpml/fixed_point.h"

bool test_comparison()
{
    fpml::fixed_point<int, 16> fp0(0);
    fpml::fixed_point<int, 16> fp1(0);
	fpml::fixed_point<int, 16> fp2(-10);
	fpml::fixed_point<int, 16> fp3(10);
    fpml::fixed_point<int, 16> fp4(200);
    fpml::fixed_point<int, 16> fp5(10);
    fpml::fixed_point<int, 16> fp6(-200);
    fpml::fixed_point<int, 16> fp7(-10);
    fpml::fixed_point<int, 16> fp8(-34.15);
    fpml::fixed_point<int, 16> fp9(34.15);

    // Integral with 0.
    TH_ASSERT_TRUE(fp0 == fp1, "");
    TH_ASSERT_TRUE(fp0 >= fp1, "");
    TH_ASSERT_TRUE(fp0 <= fp1, "");
    TH_ASSERT_TRUE(fp0 > fp2, "");
    TH_ASSERT_TRUE(fp0 >= fp2, "");
    TH_ASSERT_TRUE(fp0 < fp3, "");
    TH_ASSERT_TRUE(fp0 <= fp3, "");

    // Two integral pos numbers.
    TH_ASSERT_TRUE(fp3 == fp5, "");
    TH_ASSERT_TRUE(fp3 >= fp5, "");
    TH_ASSERT_TRUE(fp3 <= fp5, "");
    TH_ASSERT_TRUE(fp3 > fp4, "");
    TH_ASSERT_TRUE(fp3 >= fp4, "");
    TH_ASSERT_TRUE(fp4 < fp3, "");
    TH_ASSERT_TRUE(fp4 <= fp3, "");

    // Two integral neg numbers.
    TH_ASSERT_TRUE(fp2 == fp7, "");
    TH_ASSERT_TRUE(fp2 <= fp7, "");
    TH_ASSERT_TRUE(fp2 >= fp7, "");
    TH_ASSERT_TRUE(fp2 > fp6, "");
    TH_ASSERT_TRUE(fp2 >= fp6, "");
    TH_ASSERT_TRUE(fp6 < fp2, "");
    TH_ASSERT_TRUE(fp6 <= fp2, "");

    // Integral neg and pos.
    TH_ASSERT_TRUE(fp3 > fp2, "");
    TH_ASSERT_TRUE(fp3 >= fp2, "");
    TH_ASSERT_TRUE(fp2 < fp3, "");
    TH_ASSERT_TRUE(fp2 <= fp3, "");

    // Float with 0.
    TH_ASSERT_TRUE(fp9 > fp0, "");
    TH_ASSERT_TRUE(fp9 >= fp0, "");
    TH_ASSERT_TRUE(fp8 < fp0, "");
    TH_ASSERT_TRUE(fp8 <= fp0, "");

    // Two float pos numbers.

    // Two float neg numbers.

    // Float pos and neg numbers.

    return true;
}

bool test_equality()
{
    return true;
}

bool test_unary_minus()
{
    fpml::fixed_point<int, 16> fp1((signed char)-127);
	fpml::fixed_point<int, 16> fp2((unsigned char)127);
	fpml::fixed_point<int, 16> fp3((signed short)-32767);
	fpml::fixed_point<int, 16> fp4((unsigned short)32767);
	fpml::fixed_point<int, 16> fp5((signed int)-32767);
	fpml::fixed_point<int, 16> fp6((unsigned int)32767);
	fpml::fixed_point<int, 16> fp7((float)-1.5);
	fpml::fixed_point<int, 16> fp8((float)1.5);
	fpml::fixed_point<int, 16> fp9((double)-1.5);
	fpml::fixed_point<int, 16> fp10((double)1.5);
	fpml::fixed_point<int, 16> fp11((long double)-1.5);
	fpml::fixed_point<int, 16> fp12((long double)1.5);

	TH_ASSERT(fp1, -fp2, "");
	TH_ASSERT(-fp1, fp2, "");
	TH_ASSERT(fp3, -fp4, "");
	TH_ASSERT(-fp3, fp4, "");
	TH_ASSERT(fp5, -fp6, "");
	TH_ASSERT(-fp5, fp6, "");
	TH_ASSERT(fp7, -fp8, "");
	TH_ASSERT(-fp7, fp8, "");
	TH_ASSERT(fp9, -fp10, "");
	TH_ASSERT(-fp9, fp10, "");
	TH_ASSERT(fp11, -fp12, "");
	TH_ASSERT(-fp11, fp12, "");

    return true;
}

bool test_prefix_increment()
{
    return true;
}

bool test_prefix_decrement()
{
    return true;
}

bool test_postfix_increment()
{
    return true;
}

bool test_postfix_decrement()
{
    return true;
}

bool test_add()
{
    return true;
}

bool test_subtract()
{
    return true;
}

bool test_add_assign()
{
    return true;
}

bool test_subtract_assign()
{
    return true;
}

bool test_multiply()
{
    return true;
}

bool test_divide()
{
    return true;
}

bool test_multiply_assign()
{
    return true;
}

bool test_divide_assign()
{
    return true;
}

bool test_lshift()
{
    return true;
}

bool test_rshift()
{
    return true;
}
