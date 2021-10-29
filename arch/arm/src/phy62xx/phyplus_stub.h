#ifndef __INCLUDE_NUTTX_PHYPLUS_STUB_H
#define __INCLUDE_NUTTX_PHYPLUS_STUB_H



enum phyplus_stub_e
{
  PHYPLUS_GPIO_REGISTER =0 ,
  PHYPLUS_GPIO_UNREGISTER,
  PHYPLUS_TIMER_REGISTER,
  PHYPLUS_TIMER_UNREGISTER,  
  PHYPLUS_MAX
};

/*
struct stub_operations_s
{

  CODE int (*go_read)(FAR struct stub_dev_s *dev, FAR bool *value);
  CODE int (*go_write)(FAR struct stub_dev_s *dev, bool value);
  CODE int (*go_attach)(FAR struct stub_dev_s *dev, pin_interrupt_t callback);
  CODE int (*go_enable)(FAR struct stub_dev_s *dev, bool enable);
  CODE int (*go_setpintype)(FAR struct stub_dev_s *dev, enum stub_pintype_e pintype);
};

struct stub_dev_s
{
  FAR const struct stub_operations_s *gp_ops;
};


#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


int stub_pin_register(FAR struct stub_dev_s *dev);



void stub_pin_unregister(FAR struct stub_dev_s *dev);


#ifdef __cplusplus
}
#endif

#endif
*/

//int phyplus_stub_init(void);

#endif 
