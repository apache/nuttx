#ifndef __INCLUDE_NUTTX_DMA_DMA_ALIGN_MANAGER_H
#define __INCLUDE_NUTTX_DMA_DMA_ALIGN_MANAGER_H

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>

#include <debug.h>


struct dma_align_allocator_s
{
    CODE uint8_t*   (*alloc_align_buffer)(FAR void *dev,size_t buflen);
    CODE int (*free_align_buffer)(FAR void *dev,uint8_t *buf);
};

struct dma_align_manager_s
{
    uint8_t allocated;
    uint8_t *align_buffer; //from malloc  or  up layout original buffer,like:fat layout  
    struct dma_align_allocator_s  *allocator;
    void *device;
};
struct dma_align_manager_init_s
{
    void *dev;
    struct dma_align_allocator_s *allocator;
    uint8_t *original_buffer;
    uint32_t original_buffer_len;
};


#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int dma_align_manager_init(struct dma_align_manager_s *manager,struct dma_align_manager_init_s *initcfg);
uint8_t *dma_align_manager_get_align_buffer(struct dma_align_manager_s *manager);
int dma_align_manager_finish(struct dma_align_manager_s *manager);

#undef EXTERN
#ifdef __cplusplus
}
#endif


#endif