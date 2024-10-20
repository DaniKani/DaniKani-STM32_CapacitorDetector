#ifndef LCD_CONFIG_H_
#define LCD_CONFIG_H_

/* CONFIG FOR LIBRARY USER */
#define D4_GPIO_Port GPIOB
#define RS_GPIO_Port GPIOB

//4 pin mode -> pins
#define D4_Pin GPIO_PIN_10
#define D5_Pin GPIO_PIN_4
#define D6_Pin GPIO_PIN_5
#define D7_Pin GPIO_PIN_3

#define RS_Pin GPIO_PIN_15
#define E_Pin  GPIO_PIN_14
//RW Pin not used,connect to GND

//if you want to work with 8 bit mode uncomment the area which is given below

/*
#define LCD8Bit
#define DATA1_Pin GPIO_PIN_1
#define DATA2_Pin GPIO_PIN_2
#define DATA3_Pin GPIO_PIN_3
#define DATA4_Pin GPIO_PIN_4
*/


#endif /* LCD_CONFIG_H_ */
