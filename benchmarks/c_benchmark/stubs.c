/*
 * stubs.c — Provide missing symbols from file_utils / ARUtil that
 * arPattLoad.c and arUtil.c reference but we don't need for benchmarks.
 */
#include <stddef.h>

char *cat(const char *file, size_t *bufSize_p)
{
    (void)file; (void)bufSize_p;
    return NULL;
}

int test_d(const char *dir)
{
    (void)dir;
    return -1;
}

int mkdir_p(const char *path)
{
    (void)path;
    return -1;
}
