// ======================================================================
// \title  config/ZephyrAllocator.hpp
// \brief  Zephyr memory allocator using k_malloc/k_aligned_alloc
// ======================================================================
#ifndef CONFIG_ZEPHYR_ALLOCATOR_HPP
#define CONFIG_ZEPHYR_ALLOCATOR_HPP

#include <cstddef>

#include <Fw/Types/MemAllocator.hpp>
#include <zephyr/kernel.h>

namespace Fw {

class ZephyrKmallocAllocator final : public MemAllocator {
  public:
    ZephyrKmallocAllocator() = default;
    ~ZephyrKmallocAllocator() override = default;

    void* allocate(const FwEnumStoreType identifier,
                   FwSizeType& size,
                   bool& recoverable,
                   FwSizeType alignment = alignof(std::max_align_t)) override {
        static_cast<void>(identifier);
        recoverable = false;

        // k_aligned_alloc requires alignment to be a power of two.
        // Round up if needed to prevent __ASSERT crash.
        FwSizeType safeAlignment = alignment;
        if (safeAlignment == 0U) {
            safeAlignment = 1U;
        }
        if ((safeAlignment & (safeAlignment - 1U)) != 0U) {
            safeAlignment--;
            safeAlignment |= safeAlignment >> 1;
            safeAlignment |= safeAlignment >> 2;
            safeAlignment |= safeAlignment >> 4;
            safeAlignment |= safeAlignment >> 8;
            safeAlignment |= safeAlignment >> 16;
            safeAlignment++;
        }

        // Zephyr documents k_malloc as only guaranteeing pointer-size alignment.
        // Route stricter requests through k_aligned_alloc so fprime's alignment
        // contract remains valid on 32-bit targets where max_align_t > void*.
        const FwSizeType minAlignment = static_cast<FwSizeType>(alignof(void*));
        void* memory = nullptr;
        if (safeAlignment <= minAlignment) {
            memory = k_malloc(static_cast<size_t>(size));
        } else {
            // C11 aligned_alloc requires size to be a multiple of alignment.
            const FwSizeType remainder = size % safeAlignment;
            const FwSizeType allocSize = (remainder == 0U) ? size : (size + safeAlignment - remainder);
            memory = k_aligned_alloc(static_cast<size_t>(safeAlignment), static_cast<size_t>(allocSize));
        }
        if (memory == nullptr) {
            size = 0;
        }
        return memory;
    }

    void deallocate(const FwEnumStoreType identifier, void* ptr) override {
        static_cast<void>(identifier);
        k_free(ptr);
    }
};

}  // namespace Fw

#endif  // CONFIG_ZEPHYR_ALLOCATOR_HPP
