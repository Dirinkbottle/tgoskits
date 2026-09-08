#define _GNU_SOURCE
#include <elf.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/syscall.h>
#include <unistd.h>

#ifndef PR_GET_AUXV
#define PR_GET_AUXV 0x41555856
#endif

static int passed;
static int failed;

static void check(int condition, const char *name)
{
    if (condition) {
        printf("PASS: %s\n", name);
        passed++;
    } else {
        printf("FAIL: %s\n", name);
        failed++;
    }
}

static long prctl_get_auxv(void *buffer, size_t size,
                           unsigned long arg4, unsigned long arg5)
{
    return syscall(SYS_prctl, PR_GET_AUXV, buffer, size, arg4, arg5);
}

static unsigned long find_auxv_value(const Elf64_auxv_t *entries,
                                     size_t byte_count, unsigned long type,
                                     int *found_null)
{
    size_t count = byte_count / sizeof(*entries);

    for (size_t index = 0; index < count; index++) {
        if (entries[index].a_type == AT_NULL) {
            *found_null = 1;
            return 0;
        }
        if (entries[index].a_type == type)
            return entries[index].a_un.a_val;
    }
    return 0;
}

int main(void)
{
    _Alignas(Elf64_auxv_t) unsigned char full[512];
    _Alignas(Elf64_auxv_t) unsigned char prefix[sizeof(Elf64_auxv_t)];
    int found_null = 0;

    printf("=== bug-prctl-get-auxv ===\n");
    memset(full, 0xa5, sizeof(full));
    errno = 0;
    long ret = prctl_get_auxv(full, sizeof(full), 0, 0);
    check(ret > 0 && (size_t)ret <= sizeof(full),
          "PR_GET_AUXV result fits the supplied buffer");
    const long full_size = ret;

    if (ret > 0 && (size_t)ret <= sizeof(full)) {
        const Elf64_auxv_t *entries = (const Elf64_auxv_t *)full;
        check(find_auxv_value(entries, (size_t)ret, AT_PAGESZ, &found_null) != 0,
              "auxv contains AT_PAGESZ");
        check(find_auxv_value(entries, (size_t)ret, AT_RANDOM, &found_null) != 0,
              "auxv contains a non-null AT_RANDOM pointer");
        check(find_auxv_value(entries, (size_t)ret, AT_EXECFN, &found_null) != 0,
              "auxv contains a non-null AT_EXECFN pointer");
        (void)find_auxv_value(entries, (size_t)ret, UINT64_MAX, &found_null);
        check(found_null, "auxv is terminated by AT_NULL");
    }

    memset(prefix, 0, sizeof(prefix));
    errno = 0;
    ret = prctl_get_auxv(prefix, sizeof(prefix), 0, 0);
    check(full_size > 0 && ret == full_size,
          "a short buffer still returns the full auxv size");
    check(memcmp(prefix, full, sizeof(prefix)) == 0,
          "a short buffer receives an exact prefix");

    errno = 0;
    ret = prctl_get_auxv((void *)(uintptr_t)1, 0, 0, 0);
    check(full_size > 0 && ret == full_size && errno == 0,
          "a zero-length request does not dereference the pointer");

    errno = 0;
    ret = prctl_get_auxv((void *)(uintptr_t)1, 1, 0, 0);
    check(ret == -1 && errno == EFAULT,
          "an invalid non-empty destination returns EFAULT");

    errno = 0;
    ret = prctl_get_auxv((void *)(uintptr_t)1, 1, 1, 0);
    check(ret == -1 && errno == EINVAL,
          "nonzero reserved arguments take EINVAL precedence");

    printf("=== Results: %d passed, %d failed ===\n", passed, failed);
    if (failed == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("SOME TESTS FAILED\n");
    return 1;
}
