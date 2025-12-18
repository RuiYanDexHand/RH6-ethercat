#ifndef SHM_UTILS_H
#define SHM_UTILS_H

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include "rh6_ecat/shared_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// 函数声明
SharedData_t* create_shared_memory(int create_new);
void destroy_shared_memory(SharedData_t* shared_data);
int read_shared_data(SharedData_t* shared_data, SharedData_t* local_copy);
int write_shared_data(SharedData_t* shared_data, const SharedData_t* local_data);

#ifdef __cplusplus
}
#endif

#endif // SHM_UTILS_H
