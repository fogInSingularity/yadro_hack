#include <stdint.h>

static uint32_t udivmod32(uint32_t n, uint32_t d, uint32_t *r_out) {
    if (d == 0) {
        if (r_out) *r_out = 0;
        return 0xffffffffu;
    }

    uint32_t q = 0;
    uint32_t r = 0;

    for (int i = 31; i >= 0; --i) {
        r = (r << 1) | ((n >> i) & 1u);
        if (r >= d) {
            r -= d;
            q |= (1u << i);
        }
    }

    if (r_out) *r_out = r;
    return q;
}

static uint64_t udivmod64(uint64_t n, uint64_t d, uint64_t *r_out) {
    if (d == 0) {
        if (r_out) *r_out = 0;
        return 0xffffffffffffffffull;
    }

    uint64_t q = 0;
    uint64_t r = 0;

    for (int i = 63; i >= 0; --i) {
        r = (r << 1) | ((n >> i) & 1ull);
        if (r >= d) {
            r -= d;
            q |= (1ull << i);
        }
    }

    if (r_out) *r_out = r;
    return q;
}

/* GCC helper: signed 32-bit multiply */
int __mulsi3(int a, int b) {
    uint32_t ua = (uint32_t)a;
    uint32_t ub = (uint32_t)b;
    uint32_t res = 0;

    while (ub) {
        if (ub & 1u)
            res += ua;
        ua <<= 1;
        ub >>= 1;
    }

    return (int)res;
}

/* GCC helper: unsigned 32-bit divide */
unsigned int __udivsi3(unsigned int a, unsigned int b) {
    return udivmod32(a, b, 0);
}

/* Optional, often needed sooner or later */
unsigned int __umodsi3(unsigned int a, unsigned int b) {
    uint32_t r;
    (void)udivmod32(a, b, &r);
    return r;
}

/* GCC helper: unsigned 64-bit divide */
unsigned long long __udivdi3(unsigned long long a, unsigned long long b) {
    return udivmod64(a, b, 0);
}

/* Optional, often needed sooner or later */
unsigned long long __umoddi3(unsigned long long a, unsigned long long b) {
    uint64_t r;
    (void)udivmod64(a, b, &r);
    return r;
}
