#include <unordered_set>
#include <unordered_map>
#include <string>
#include <functional>
#include <iostream>

#include "base_tests.h"
#include "operators_tests.h"
#include "math_functions_tests.h"
#include "casts_tests.h"
#include "other_tests.h"

using TestFunc = std::function<bool()>;
const std::unordered_map<std::string, TestFunc> ALL_TESTS = {
    {"test_construction", &test_construction},
    {"test_copy_construction", &test_copy_construction},
    {"test_copy_assignment", &test_copy_assignment},
    {"test_sqrt", &test_sqrt},
    {"test_sin", &test_sin},
    {"test_cos", &test_cos},
    {"test_exp", &test_exp},
    {"test_modf", &test_modf},
    {"test_fmod", &test_fmod},
    {"test_floor", &test_floor},
    {"test_ceil", &test_ceil},
    {"test_fabs", &test_fabs},
    {"test_ftos", &test_ftos},
    {"test_stof", &test_stof},
    {"test_comparison", &test_comparison},
    {"test_equality", &test_equality},
    {"test_unary_minus", &test_unary_minus},
    {"test_prefix_increment", &test_prefix_increment},
    {"test_prefix_decrement", &test_prefix_decrement},
    {"test_postfix_increment", &test_postfix_increment},
    {"test_postfix_decrement", &test_postfix_decrement},
    {"test_add", &test_add},
    {"test_subtract", &test_subtract},
    {"test_add_assign", &test_add_assign},
    {"test_subtract_assign", &test_subtract_assign},
    {"test_multiply", &test_multiply},
    {"test_divide", &test_divide},
    {"test_multiply_assign", &test_multiply_assign},
    {"test_divide_assign", &test_divide_assign},
    {"test_lshift", &test_lshift},
    {"test_rshift", &test_rshift},
    {"test_to_char", &test_to_char},
    {"test_to_signed_char", &test_to_signed_char},
    {"test_to_unsigned_char", &test_to_unsigned_char},
    {"test_to_short", &test_to_short},
    {"test_to_unsigned_short", &test_to_unsigned_short},
    {"test_to_int", &test_to_int},
    {"test_to_unsigned_int", &test_to_unsigned_int},
    {"test_to_long", &test_to_long},
    {"test_to_unsigned_long", &test_to_unsigned_long},
    {"test_to_long_long", &test_to_long_long},
    {"test_to_unsigned_long_long", &test_to_unsigned_long_long},
    {"test_to_bool", &test_to_bool},
    {"test_to_float", &test_to_float},
    {"test_to_double", &test_to_double},
    {"test_to_long_double", &test_to_long_double},
    {"test_to_base", &test_to_base},
    {"test_ostream", &test_ostream},
    {"test_istream", &test_istream},
    {"test_create_with_raw", &test_create_with_raw},
    {"test_fract", &test_fract},
};

class Main
{
public:
    Main(int argc, char* argv[])
        : tests_to_run(toSet(argc, argv))
    {
    }

    int run()
    {
        std::cout << "#### START TESTS. ####" << std::endl;
        int failed_count = 0;
        for (const auto & [test_name, test_func] : ALL_TESTS)
        {
            if (tests_to_run.count(test_name) > 0u || tests_to_run.empty())
            {
                std::cout << "TEST '" << test_name << "':" << std::endl;
                if (!test_func())
                {
                    std::cout << "  FAIL" << std::endl;
                    failed_count++;
                }
                else
                {
                    std::cout << "  PASS" << std::endl;
                }
            }
        }

        const size_t tests_count = tests_to_run.empty() ? ALL_TESTS.size() : tests_to_run.size();
        std::cout
            << "#### FINISH. ####\n"
            << "Run: " << tests_count << "/" << ALL_TESTS.size()
            << ", Fail: " << failed_count
            << ", Pass: " << tests_count - static_cast<size_t>(failed_count)
            << std::endl;

        return failed_count;
    }

protected:

    std::unordered_set<std::string> toSet(int argc, char* argv[])
    {
        std::unordered_set<std::string> set;
        for (int i = 1 /* Skip exe file. */; i < argc; ++i)
            set.emplace(std::string(argv[i]));

        return set;
    }

private:
    std::unordered_set<std::string> tests_to_run;
};

int main(int argc, char* argv[])
{
    Main m(argc, argv);
    return m.run();
}
