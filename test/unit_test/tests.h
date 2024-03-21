#pragma once

#include <string>
#include <cstdint>

#ifdef TH_ASSERT
#undef TH_ASSERT
#endif
#define TH_ASSERT(V_A, V_B, MSG)                                        \
    if (!thAssert(V_A == V_B, #V_A ## " == " ## #V_B, MSG, __LINE__))   \
        return false

#ifdef TH_N_ASSERT
#undef TH_N_ASSERT
#endif
#define TH_N_ASSERT(V_A, V_B, MSG)                                      \
    if (!thAssert(V_A != V_B, #V_A ## " != " ## #V_B, MSG, __LINE__))   \
        return false

#ifdef TH_ASSERT_TRUE
#undef TH_ASSERT_TRUE
#endif
#define TH_ASSERT_TRUE(V, MSG)                                          \
    if (!thAssert(V, #V, MSG, __LINE__))                                \
        return false

#ifdef TH_MESSAGE
#undef TH_MESSAGE
#endif
#define TH_MESSAGE(MSG)                                     \
    thMessage(MSG, __LINE__)

#ifdef TH_ASSERT_F
#undef TH_ASSERT_F
#endif
#define TH_ASSERT_F(V_A, V_B, MSG)                                      \
    if (!thAssertF(V_A, V_B, #V_A ## " == " ## #V_B, MSG, __LINE__))    \
        return false

bool thAssert(bool v, const std::string & msg, const std::string & additional_msg, uint32_t line);
bool thAssertF(float a, float b, const std::string & msg, const std::string & additional_msg, uint32_t line);
void thMessage(const std::string & msg, uint32_t line);
