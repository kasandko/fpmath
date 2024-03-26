#include "operators_tests.h"

#include "tests.h"
#include "../../include/fpml/fixed_point.h"

bool test_comparison()
{
    fpml::fixed_point<int, 16> fp01(0);
    fpml::fixed_point<int, 16> fp02(0);

	fpml::fixed_point<int, 16> fp11(-10);
	fpml::fixed_point<int, 16> fp12(10);
    fpml::fixed_point<int, 16> fp13(200);
    fpml::fixed_point<int, 16> fp14(10);
    fpml::fixed_point<int, 16> fp15(-200);
    fpml::fixed_point<int, 16> fp16(-10);

    fpml::fixed_point<int, 16> fp21(-34.15);
    fpml::fixed_point<int, 16> fp22(34.15);
    fpml::fixed_point<int, 16> fp23(115.67);
    fpml::fixed_point<int, 16> fp24(34.15);
    fpml::fixed_point<int, 16> fp25(-115.67);
    fpml::fixed_point<int, 16> fp26(-34.15);

    // With 0.
    TH_ASSERT_TRUE(fp01 >= fp02, "");
    TH_ASSERT_TRUE(fp01 <= fp02, "");
    TH_ASSERT_TRUE(fp01 > fp11, "");
    TH_ASSERT_TRUE(fp01 >= fp11, "");
    TH_ASSERT_TRUE(fp01 < fp12, "");
    TH_ASSERT_TRUE(fp01 <= fp12, "");
    TH_ASSERT_TRUE(fp01 > fp21, "");
    TH_ASSERT_TRUE(fp01 >= fp21, "");
    TH_ASSERT_TRUE(fp01 < fp22, "");
    TH_ASSERT_TRUE(fp01 <= fp22, "");

    TH_ASSERT_FALSE(fp01 <= fp02, "");
    TH_ASSERT_FALSE(fp01 >= fp02, "");
    TH_ASSERT_FALSE(fp01 < fp11, "");
    TH_ASSERT_FALSE(fp01 <= fp11, "");
    TH_ASSERT_FALSE(fp01 > fp12, "");
    TH_ASSERT_FALSE(fp01 >= fp12, "");
    TH_ASSERT_FALSE(fp01 < fp21, "");
    TH_ASSERT_FALSE(fp01 <= fp21, "");
    TH_ASSERT_FALSE(fp01 > fp22, "");
    TH_ASSERT_FALSE(fp01 >= fp22, "");

    // Two integral pos numbers.
    TH_ASSERT_TRUE(fp12 >= fp14, "");
    TH_ASSERT_TRUE(fp12 <= fp14, "");
    TH_ASSERT_TRUE(fp12 > fp13, "");
    TH_ASSERT_TRUE(fp12 >= fp13, "");
    TH_ASSERT_TRUE(fp13 < fp12, "");
    TH_ASSERT_TRUE(fp13 <= fp12, "");

    TH_ASSERT_FALSE(fp12 <= fp14, "");
    TH_ASSERT_FALSE(fp12 >= fp14, "");
    TH_ASSERT_FALSE(fp12 < fp13, "");
    TH_ASSERT_FALSE(fp12 <= fp13, "");
    TH_ASSERT_FALSE(fp13 > fp12, "");
    TH_ASSERT_FALSE(fp13 >= fp12, "");

    // Two integral neg numbers.
    TH_ASSERT_TRUE(fp11 <= fp16, "");
    TH_ASSERT_TRUE(fp11 >= fp16, "");
    TH_ASSERT_TRUE(fp11 > fp15, "");
    TH_ASSERT_TRUE(fp11 >= fp15, "");
    TH_ASSERT_TRUE(fp15 < fp11, "");
    TH_ASSERT_TRUE(fp15 <= fp11, "");

    TH_ASSERT_FALSE(fp11 >= fp16, "");
    TH_ASSERT_FALSE(fp11 <= fp16, "");
    TH_ASSERT_FALSE(fp11 < fp15, "");
    TH_ASSERT_FALSE(fp11 <= fp15, "");
    TH_ASSERT_FALSE(fp15 > fp11, "");
    TH_ASSERT_FALSE(fp15 >= fp11, "");

    // Integral neg and pos.
    TH_ASSERT_TRUE(fp12 > fp11, "");
    TH_ASSERT_TRUE(fp12 >= fp11, "");
    TH_ASSERT_TRUE(fp11 < fp12, "");
    TH_ASSERT_TRUE(fp11 <= fp12, "");

    TH_ASSERT_FALSE(fp12 < fp11, "");
    TH_ASSERT_FALSE(fp12 <= fp11, "");
    TH_ASSERT_FALSE(fp11 > fp12, "");
    TH_ASSERT_FALSE(fp11 >= fp12, "");

    // Two float pos numbers.
    TH_ASSERT_TRUE(fp22 >= fp24, "");
    TH_ASSERT_TRUE(fp22 <= fp24, "");
    TH_ASSERT_TRUE(fp22 > fp23, "");
    TH_ASSERT_TRUE(fp22 >= fp23, "");
    TH_ASSERT_TRUE(fp23 < fp22, "");
    TH_ASSERT_TRUE(fp23 <= fp22, "");

    TH_ASSERT_FALSE(fp22 <= fp24, "");
    TH_ASSERT_FALSE(fp22 >= fp24, "");
    TH_ASSERT_FALSE(fp22 < fp23, "");
    TH_ASSERT_FALSE(fp22 <= fp23, "");
    TH_ASSERT_FALSE(fp23 > fp22, "");
    TH_ASSERT_FALSE(fp23 >= fp22, "");

    // Two float neg numbers.
    TH_ASSERT_TRUE(fp21 <= fp26, "");
    TH_ASSERT_TRUE(fp21 >= fp26, "");
    TH_ASSERT_TRUE(fp21 > fp25, "");
    TH_ASSERT_TRUE(fp21 >= fp25, "");
    TH_ASSERT_TRUE(fp25 < fp21, "");
    TH_ASSERT_TRUE(fp25 <= fp21, "");

    TH_ASSERT_FALSE(fp21 >= fp26, "");
    TH_ASSERT_FALSE(fp21 <= fp26, "");
    TH_ASSERT_FALSE(fp21 < fp25, "");
    TH_ASSERT_FALSE(fp21 <= fp25, "");
    TH_ASSERT_FALSE(fp25 > fp21, "");
    TH_ASSERT_FALSE(fp25 >= fp21, "");

    // Float pos and neg numbers.
    TH_ASSERT_TRUE(fp22 > fp21, "");
    TH_ASSERT_TRUE(fp22 >= fp21, "");
    TH_ASSERT_TRUE(fp21 < fp22, "");
    TH_ASSERT_TRUE(fp21 <= fp22, "");

    TH_ASSERT_FALSE(fp22 < fp21, "");
    TH_ASSERT_FALSE(fp22 <= fp21, "");
    TH_ASSERT_FALSE(fp21 > fp22, "");
    TH_ASSERT_FALSE(fp21 >= fp22, "");

    // Authors tests.
    fpml::fixed_point<int, 16> fp0(0);
	fpml::fixed_point<int, 16> fp1((signed char)-128);
	fpml::fixed_point<int, 16> fp2((unsigned char)127);
	fpml::fixed_point<int, 16> fp3((signed short)-32768);
	fpml::fixed_point<int, 16> fp4((unsigned short)32767);
	fpml::fixed_point<int, 16> fp5((signed int)-32768);
	fpml::fixed_point<int, 16> fp6((unsigned int)32767);
	fpml::fixed_point<int, 16> fp7((float)-1.5);
	fpml::fixed_point<int, 16> fp8((float)1.5);
	fpml::fixed_point<int, 16> fp9((double)-1.5);
	fpml::fixed_point<int, 16> fp10((double)1.5);
	fpml::fixed_point<int, 16> fp11((long double)-1.5);
	fpml::fixed_point<int, 16> fp12((long double)1.5);

	TH_ASSERT_TRUE(fp1 < fp2, "");
	TH_ASSERT_TRUE(fp2 > fp1, "");
	TH_ASSERT_FALSE(fp4 < fp3, "");
	TH_ASSERT_FALSE(fp3 > fp4, "");

    return true;
}

bool test_equality()
{
    fpml::fixed_point<int, 16> fp01(0);
    fpml::fixed_point<int, 16> fp02(0);

	fpml::fixed_point<int, 16> fp11(-10);
	fpml::fixed_point<int, 16> fp12(10);
    fpml::fixed_point<int, 16> fp13(200);
    fpml::fixed_point<int, 16> fp14(10);
    fpml::fixed_point<int, 16> fp15(-200);
    fpml::fixed_point<int, 16> fp16(-10);

    fpml::fixed_point<int, 16> fp21(-34.15);
    fpml::fixed_point<int, 16> fp22(34.15);
    fpml::fixed_point<int, 16> fp23(115.67);
    fpml::fixed_point<int, 16> fp24(34.15);
    fpml::fixed_point<int, 16> fp25(-115.67);
    fpml::fixed_point<int, 16> fp26(-34.15);

    // With 0.
    TH_ASSERT_TRUE(fp01 == fp02, "");
    TH_ASSERT_TRUE(fp01 != fp11, "");
    TH_ASSERT_TRUE(fp01 != fp12, "");
    TH_ASSERT_TRUE(fp01 != fp21, "");
    TH_ASSERT_TRUE(fp01 != fp22, "");

    TH_ASSERT_FALSE(fp01 != fp02, "");
    TH_ASSERT_FALSE(fp01 == fp11, "");
    TH_ASSERT_FALSE(fp01 == fp12, "");
    TH_ASSERT_FALSE(fp01 == fp21, "");
    TH_ASSERT_FALSE(fp01 == fp22, "");

    // Two integral pos numbers.
    TH_ASSERT_TRUE(fp12 == fp14, "");

    // Two integral neg numbers.
    TH_ASSERT_TRUE(fp11 == fp16, "");

    // Two float pos numbers.
    TH_ASSERT_TRUE(fp22 == fp24, "");

    // Two float neg numbers.
    TH_ASSERT_TRUE(fp21 == fp26, "");

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
