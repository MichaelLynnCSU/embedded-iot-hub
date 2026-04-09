#include "trinity_log.h"
#include <stdlib.h>
void  trinity_log_heap_stats(void)  {}
void  trinity_log_task_stats(void)  {}
void *trinity_malloc(size_t s)      { return malloc(s); }
void  trinity_free(void **pp)       { if(pp&&*pp){free(*pp);*pp=NULL;} }
