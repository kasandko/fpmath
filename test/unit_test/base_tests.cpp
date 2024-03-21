#include "base_tests.h"

#include "tests.h"
#include "../../include/fpml/fixed_point.h"

bool test_construction()
{
    fpml::fixed_point<int, 16> fp0(0);
	TH_ASSERT((int)fp0, 0, "");

	fpml::fixed_point<int, 16> fp1((signed char)-128);
	TH_ASSERT((signed char)fp1, -128, "");

	fpml::fixed_point<int, 16> fp2((unsigned char)127);
	TH_ASSERT((unsigned char)fp2, 127, "");

	fpml::fixed_point<int, 16> fp3((signed short)-32768);
	TH_ASSERT((signed short)fp3, -32768, "");

	fpml::fixed_point<int, 16> fp4((unsigned short)32767);
	TH_ASSERT((unsigned short)fp4, 32767, "");

	fpml::fixed_point<int, 16> fp5((signed int)-32768);
	TH_ASSERT((signed int)fp5, -32768, "");

	fpml::fixed_point<int, 16> fp6((unsigned int)32767);
	TH_ASSERT((unsigned int)fp6, 32767, "");

	fpml::fixed_point<int, 16> fp7((float)-1.5);
	TH_ASSERT((float)fp7, -1.5, "");

	fpml::fixed_point<int, 16> fp8((float)1.5);
	TH_ASSERT((float)fp8, 1.5, "");

	fpml::fixed_point<int, 16> fp9((double)-1.5);
	TH_ASSERT((double)fp9, -1.5, "");

	fpml::fixed_point<int, 16> fp10((double)1.5);
	TH_ASSERT((double)fp10, 1.5, "");

	fpml::fixed_point<int, 16> fp11((long double)-1.5);
	TH_ASSERT((long double)fp11, -1.5, "");

	fpml::fixed_point<int, 16> fp12((long double)1.5);
	TH_ASSERT((long double)fp12, 1.5, "");

    return true;
}

bool test_copy_construction()
{
    fpml::fixed_point<int, 16> fp0(0);
	fpml::fixed_point<int, 16> fp0c(fp0);
	TH_ASSERT((int)fp0c, 0, "");

	fpml::fixed_point<int, 16> fp1((signed char)-128);
	fpml::fixed_point<int, 16> fp1c(fp1);
	TH_ASSERT((signed char)fp1c, -128, "");

	fpml::fixed_point<int, 16> fp2((unsigned char)127);
	fpml::fixed_point<int, 16> fp2c(fp2);
	TH_ASSERT((unsigned char)fp2c, 127, "");

	fpml::fixed_point<int, 16> fp3((signed short)-32768);
	fpml::fixed_point<int, 16> fp3c(fp3);
	TH_ASSERT((signed short)fp3c, -32768, "");

	fpml::fixed_point<int, 16> fp4((unsigned short)32767);
	fpml::fixed_point<int, 16> fp4c(fp4);
	TH_ASSERT((unsigned short)fp4c, 32767, "");

	fpml::fixed_point<int, 16> fp5((signed int)-32768);
	fpml::fixed_point<int, 16> fp5c(fp5);
	TH_ASSERT((signed int)fp5c, -32768, "");

	fpml::fixed_point<int, 16> fp6((unsigned int)32767);
	fpml::fixed_point<int, 16> fp6c(fp6);
	TH_ASSERT((unsigned int)fp6c, 32767, "");

	fpml::fixed_point<int, 16> fp7((float)-1.5);
	fpml::fixed_point<int, 16> fp7c(fp7);
	TH_ASSERT((float)fp7c, -1.5, "");

	fpml::fixed_point<int, 16> fp8((float)1.5);
	fpml::fixed_point<int, 16> fp8c(fp8);
	TH_ASSERT((float)fp8c, 1.5, "");

	fpml::fixed_point<int, 16> fp9((double)-1.5);
	fpml::fixed_point<int, 16> fp9c(fp9);
	TH_ASSERT((double)fp9c, -1.5, "");

	fpml::fixed_point<int, 16> fp10((double)1.5);
	fpml::fixed_point<int, 16> fp10c(fp10);
	TH_ASSERT((double)fp10c, 1.5, "");

	fpml::fixed_point<int, 16> fp11((long double)-1.5);
	fpml::fixed_point<int, 16> fp11c(fp11);
	TH_ASSERT((long double)fp11c, -1.5, "");

	fpml::fixed_point<int, 16> fp12((long double)1.5);
	fpml::fixed_point<int, 16> fp12c(fp12);
	TH_ASSERT((long double)fp12c, 1.5, "");

    return true;
}

bool test_copy_assignment()
{
    fpml::fixed_point<int, 16> fp0(0);
	fpml::fixed_point<int, 16> fp0c;
	fp0c = fp0;
	TH_ASSERT((int)fp0c, 0, "");

	fpml::fixed_point<int, 16> fp1((signed char)-128);
	fpml::fixed_point<int, 16> fp1c;
	fp1c = fp1;
	TH_ASSERT((signed char)fp1c, -128, "");

	fpml::fixed_point<int, 16> fp2((unsigned char)127);
	fpml::fixed_point<int, 16> fp2c;
	fp2c = fp2;
	TH_ASSERT((unsigned char)fp2c, 127, "");

	fpml::fixed_point<int, 16> fp3((signed short)-32768);
	fpml::fixed_point<int, 16> fp3c;
	fp3c = fp3;
	TH_ASSERT((signed short)fp3c, -32768, "");

	fpml::fixed_point<int, 16> fp4((unsigned short)32767);
	fpml::fixed_point<int, 16> fp4c;
	fp4c = fp4;
	TH_ASSERT((unsigned short)fp4c, 32767, "");

	fpml::fixed_point<int, 16> fp5((signed int)-32768);
	fpml::fixed_point<int, 16> fp5c;
	fp5c = fp5;
	TH_ASSERT((signed int)fp5c, -32768, "");

	fpml::fixed_point<int, 16> fp6((unsigned int)32767);
	fpml::fixed_point<int, 16> fp6c;
	fp6c = fp6;
	TH_ASSERT((unsigned int)fp6c, 32767, "");

	fpml::fixed_point<int, 16> fp7((float)-1.5);
	fpml::fixed_point<int, 16> fp7c;
	fp7c = fp7;
	TH_ASSERT((float)fp7c, -1.5, "");

	fpml::fixed_point<int, 16> fp8((float)1.5);
	fpml::fixed_point<int, 16> fp8c;
	fp8c = fp8;
	TH_ASSERT((float)fp8c, 1.5, "");

	fpml::fixed_point<int, 16> fp9((double)-1.5);
	fpml::fixed_point<int, 16> fp9c;
	fp9c = fp9;
	TH_ASSERT((double)fp9c, -1.5, "");

	fpml::fixed_point<int, 16> fp10((double)1.5);
	fpml::fixed_point<int, 16> fp10c;
	fp10c = fp10;
	TH_ASSERT((double)fp10c, 1.5, "");

	fpml::fixed_point<int, 16> fp11((long double)-1.5);
	fpml::fixed_point<int, 16> fp11c;
	fp11c = fp11;
	TH_ASSERT((long double)fp11c, -1.5, "");

	fpml::fixed_point<int, 16> fp12((long double)1.5);
	fpml::fixed_point<int, 16> fp12c;
	fp12c = fp12;
	TH_ASSERT((long double)fp12c, 1.5, "");

    return true;
}
