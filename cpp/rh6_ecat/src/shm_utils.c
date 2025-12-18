#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "rh6_ecat/shared_data.h"

// 创建或打开共享内存
SharedData_t* create_shared_memory(int create_new) {
    int shm_fd;
    SharedData_t* shared_data;
    
    if (create_new) {
        // 创建新的共享内存对象
        shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            perror("shm_open create failed");
            return NULL;
        }
        
        // 设置共享内存大小
        if (ftruncate(shm_fd, SHM_SIZE) == -1) {
            perror("ftruncate failed");
            close(shm_fd);
            return NULL;
        }
        
        // 额外设置文件权限，确保其他用户可以访问
        char shm_path[256];
        snprintf(shm_path, sizeof(shm_path), "/dev/shm%s", SHM_NAME);
        chmod(shm_path, 0666);
        
    } else {
        // 打开已存在的共享内存对象
        shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
        if (shm_fd == -1) {
            perror("shm_open open failed");
            return NULL;
        }
    }
    
    // 映射共享内存到进程地址空间
    shared_data = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_data == MAP_FAILED) {
        perror("mmap failed");
        close(shm_fd);
        return NULL;
    }
    
    close(shm_fd);
    
    if (create_new) {
        // 初始化共享内存
        memset(shared_data, 0, sizeof(SharedData_t));
        
        // 初始化互斥锁和条件变量（用于进程间同步）
        pthread_mutexattr_t mutex_attr;
        pthread_mutexattr_init(&mutex_attr);
        pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
        pthread_mutex_init(&shared_data->mutex, &mutex_attr);
        pthread_mutexattr_destroy(&mutex_attr);
        
        pthread_condattr_t cond_attr;
        pthread_condattr_init(&cond_attr);
        pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
        pthread_cond_init(&shared_data->data_ready, &cond_attr);
        pthread_condattr_destroy(&cond_attr);
        
        shared_data->data_valid = 0;
        shared_data->shutdown_flag = 0;
        
        printf("Shared memory created and initialized\n");
    }
    
    return shared_data;
}

// 销毁共享内存
void destroy_shared_memory(SharedData_t* shared_data) {
    if (shared_data) {
        // 设置关闭标志
        pthread_mutex_lock(&shared_data->mutex);
        shared_data->shutdown_flag = 1;
        pthread_cond_broadcast(&shared_data->data_ready);
        pthread_mutex_unlock(&shared_data->mutex);
        
        // 解除映射
        munmap(shared_data, SHM_SIZE);
    }
    
    // 删除共享内存对象
    shm_unlink(SHM_NAME);
    printf("Shared memory destroyed\n");
}

// 安全地从共享内存读取数据
int read_shared_data(SharedData_t* shared_data, SharedData_t* local_copy) {
    if (!shared_data || !local_copy) {
        return -1;
    }
    
    pthread_mutex_lock(&shared_data->mutex);
    
    // 等待数据准备好
    while (!shared_data->data_valid && !shared_data->shutdown_flag) {
        pthread_cond_wait(&shared_data->data_ready, &shared_data->mutex);
    }
    
    if (shared_data->shutdown_flag) {
        pthread_mutex_unlock(&shared_data->mutex);
        return -1; // 收到关闭信号
    }
    
    // 复制数据
    memcpy(local_copy, shared_data, sizeof(SharedData_t));
    
    pthread_mutex_unlock(&shared_data->mutex);
    return 0;
}

// 安全地向共享内存写入数据
int write_shared_data(SharedData_t* shared_data, const SharedData_t* local_data) {
    if (!shared_data || !local_data) {
        return -1;
    }
    
    pthread_mutex_lock(&shared_data->mutex);
    
    // 复制数据（不包括同步原语）
    shared_data->timestamp_ns = local_data->timestamp_ns;
    shared_data->master_status = local_data->master_status;
    memcpy(shared_data->joints, local_data->joints, sizeof(shared_data->joints));
    memcpy(shared_data->ryhand, local_data->ryhand, sizeof(shared_data->ryhand));
    
    // 标记数据有效并通知等待的进程
    shared_data->data_valid = 1;
    pthread_cond_broadcast(&shared_data->data_ready);
    
    pthread_mutex_unlock(&shared_data->mutex);
    return 0;
}
