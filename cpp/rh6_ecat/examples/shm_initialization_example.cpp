#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <cstring>

// 简化的共享数据结构
struct SimpleSharedData {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int data;
    bool ready;
};

#define SHM_NAME "/my_shared_memory"
#define SHM_SIZE sizeof(SimpleSharedData)

int main() {
    std::cout << "=== 共享内存初始化过程详解 ===" << std::endl;
    
    // 步骤1: 创建或打开共享内存对象
    std::cout << "\n1. 创建共享内存对象..." << std::endl;
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open failed");
        return -1;
    }
    std::cout << "   ✓ 共享内存对象创建成功，文件描述符: " << shm_fd << std::endl;
    
    // 步骤2: 设置共享内存大小
    std::cout << "\n2. 设置共享内存大小..." << std::endl;
    std::cout << "   需要的大小: " << SHM_SIZE << " 字节" << std::endl;
    if (ftruncate(shm_fd, SHM_SIZE) == -1) {
        perror("ftruncate failed");
        close(shm_fd);
        return -1;
    }
    std::cout << "   ✓ 共享内存大小设置成功" << std::endl;
    
    // 步骤3: 映射共享内存到进程地址空间
    std::cout << "\n3. 映射共享内存到进程地址空间..." << std::endl;
    SimpleSharedData* shared_data = (SimpleSharedData*)mmap(
        NULL,           // 让系统自动选择地址
        SHM_SIZE,       // 映射的大小
        PROT_READ | PROT_WRITE,  // 可读可写
        MAP_SHARED,     // 进程间共享
        shm_fd,         // 文件描述符
        0               // 偏移量
    );
    
    if (shared_data == MAP_FAILED) {
        perror("mmap failed");
        close(shm_fd);
        return -1;
    }
    std::cout << "   ✓ 共享内存映射成功，地址: " << shared_data << std::endl;
    
    // 步骤4: 关闭文件描述符（不再需要）
    close(shm_fd);
    std::cout << "   ✓ 文件描述符已关闭" << std::endl;
    
    // 步骤5: 初始化共享内存内容
    std::cout << "\n4. 初始化共享内存内容..." << std::endl;
    memset(shared_data, 0, SHM_SIZE);
    std::cout << "   ✓ 内存清零完成" << std::endl;
    
    // 步骤6: 初始化进程间共享的互斥锁
    std::cout << "\n5. 初始化进程间共享的互斥锁..." << std::endl;
    
    // 6.1 创建互斥锁属性对象
    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    std::cout << "   ✓ 互斥锁属性对象创建成功" << std::endl;
    
    // 6.2 设置进程间共享属性
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    std::cout << "   ✓ 设置进程间共享属性成功" << std::endl;
    
    // 6.3 使用属性初始化互斥锁
    pthread_mutex_init(&shared_data->mutex, &mutex_attr);
    std::cout << "   ✓ 互斥锁初始化成功" << std::endl;
    
    // 6.4 销毁属性对象
    pthread_mutexattr_destroy(&mutex_attr);
    std::cout << "   ✓ 属性对象已销毁" << std::endl;
    
    // 步骤7: 初始化进程间共享的条件变量
    std::cout << "\n6. 初始化进程间共享的条件变量..." << std::endl;
    
    // 7.1 创建条件变量属性对象
    pthread_condattr_t cond_attr;
    pthread_condattr_init(&cond_attr);
    std::cout << "   ✓ 条件变量属性对象创建成功" << std::endl;
    
    // 7.2 设置进程间共享属性
    pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
    std::cout << "   ✓ 设置进程间共享属性成功" << std::endl;
    
    // 7.3 使用属性初始化条件变量
    pthread_cond_init(&shared_data->cond, &cond_attr);
    std::cout << "   ✓ 条件变量初始化成功" << std::endl;
    
    // 7.4 销毁属性对象
    pthread_condattr_destroy(&cond_attr);
    std::cout << "   ✓ 属性对象已销毁" << std::endl;
    
    // 步骤8: 设置初始状态
    shared_data->data = 0;
    shared_data->ready = false;
    std::cout << "   ✓ 初始状态设置完成" << std::endl;
    
    std::cout << "\n=== 共享内存初始化完成 ===" << std::endl;
    std::cout << "共享内存地址: " << shared_data << std::endl;
    std::cout << "共享内存大小: " << SHM_SIZE << " 字节" << std::endl;
    
    // 演示使用
    std::cout << "\n=== 使用示例 ===" << std::endl;
    pthread_mutex_lock(&shared_data->mutex);
    shared_data->data = 42;
    shared_data->ready = true;
    std::cout << "写入数据: " << shared_data->data << std::endl;
    pthread_cond_signal(&shared_data->cond);
    pthread_mutex_unlock(&shared_data->mutex);
    
    // 清理
    std::cout << "\n=== 清理资源 ===" << std::endl;
    munmap(shared_data, SHM_SIZE);
    shm_unlink(SHM_NAME);
    std::cout << "✓ 资源清理完成" << std::endl;
    
    return 0;
}

