// Host fake of <Fw/FPrimeBasicTypes.hpp>: the integer aliases UspRadio uses.
#pragma once
#include <cstddef>
#include <cstdint>
using U8 = uint8_t;
using U16 = uint16_t;
using U32 = uint32_t;
using U64 = uint64_t;
using I8 = int8_t;
using I16 = int16_t;
using I32 = int32_t;
using I64 = int64_t;
using F32 = float;
using FwSizeType = U64;
using FwIndexType = I16;
using FwOpcodeType = U32;
using FwAssertArgType = I32;
#define FW_ASSERT(cond, ...)                                                        \
    do {                                                                            \
        if (!(cond)) {                                                              \
            std::fprintf(stderr, "FW_ASSERT %s:%d (%s)\n", __FILE__, __LINE__, #cond); \
            std::abort();                                                           \
        }                                                                           \
    } while (0)
#include <cstdio>
#include <cstdlib>
