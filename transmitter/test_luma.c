/*
 * test_luma — Read AE luminance via dlopen/dlsym to avoid link-time issues.
 */

#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <unistd.h>
#include <time.h>

typedef int (*GetAeLuma_fn)(int *);

int main(int argc, char *argv[])
{
    int duration = 5;
    if (argc > 1) duration = atoi(argv[1]);

    void *handle = dlopen("libimp.so", RTLD_LAZY);
    if (!handle) {
        fprintf(stderr, "dlopen libimp.so: %s\n", dlerror());
        return 1;
    }

    GetAeLuma_fn get_luma = (GetAeLuma_fn)dlsym(handle, "IMP_ISP_Tuning_GetAeLuma");
    if (!get_luma) {
        fprintf(stderr, "dlsym GetAeLuma: %s\n", dlerror());
        dlclose(handle);
        return 1;
    }

    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    int count = 0;
    fprintf(stderr, "Reading AE luma for %ds...\n", duration);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec)
                       + (now.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed >= duration) break;

        int luma;
        int ret = get_luma(&luma);
        if (ret == 0) {
            printf("%.3f %d\n", elapsed, luma);
            count++;
        }

        usleep(20000); /* 50Hz polling */
    }

    fprintf(stderr, "Done: %d samples (%.0f Hz)\n", count, (double)count / duration);
    dlclose(handle);
    return 0;
}
