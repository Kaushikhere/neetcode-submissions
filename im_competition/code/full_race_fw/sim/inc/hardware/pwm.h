#pragma once
typedef struct { int x; } pwm_config;
#define GPIO_FUNC_PWM 4
inline void gpio_set_function(int,int){} inline unsigned pwm_gpio_to_slice_num(int){return 5;}
inline pwm_config pwm_get_default_config(){return {0};} inline void pwm_config_set_clkdiv(pwm_config*,float){}
inline void pwm_config_set_wrap(pwm_config*,int){} inline void pwm_init(unsigned,pwm_config*,bool){} inline void pwm_set_gpio_level(int,unsigned){}
