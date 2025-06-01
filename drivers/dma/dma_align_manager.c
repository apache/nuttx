#include <nuttx/dma/dma_align_manager.h>
#include <sys/types.h>
int dma_align_manager_init(struct dma_align_manager_s *manager,struct dma_align_manager_init_s *initcfg)
{
    manager->device=initcfg->dev;
    manager->allocator=initcfg->allocator;
    manager->align_buffer=manager->allocator->alloc_align_buffer(manager->device,initcfg->original_buffer_len);
    if(manager->align_buffer==NULL)
    {
        return -ENOMEM;
    }
    manager->allocated=true;
    return OK;
}
uint8_t *dma_align_manager_get_align_buffer(struct dma_align_manager_s *manager)
{
    return manager->align_buffer;
}
int dma_align_manager_finish(struct dma_align_manager_s *manager)
{
    if(manager->allocated)
    {
        if(manager->align_buffer)
        {
            manager->allocator->free_align_buffer((void *)manager->device,manager->align_buffer);
        }
    }
    manager->allocated=false;
    return OK;
}