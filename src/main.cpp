#include "sam/samanager.h"
#include <malloc.h>
#include <sys/mman.h>

#define MEMORY_POOL (100 * 1024 * 1024) // 100MB

int main(int, char**)
{
    // Now lock all current and future pages from preventing of being paged
    mlockall(MCL_CURRENT | MCL_FUTURE);
    // Turn off malloc trimming.
    mallopt(M_TRIM_THRESHOLD, -1);
    // Turn off mmap usage.
    mallopt(M_MMAP_MAX, 0);

    void* buf = malloc(MEMORY_POOL);
    free(buf);

    SAManager sam;
    sam.run();

    return 0;
}
