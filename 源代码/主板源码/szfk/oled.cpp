#include "oled.h"

/*
 *IIC构造函数
 *@param  rotation：    U8G2_R0 不旋转，横向，绘制方向从左到右
                        U8G2_R1 顺时针旋转90度，绘制方向从上到下
                        U8G2_R2 顺时针旋转180度，绘制方向从右到左
                        U8G2_R3 顺时针旋转270度，绘制方向从下到上
                        U8G2_MIRROR 正常显示镜像内容（v2.6.x版本以上使用)   注意:U8G2_MIRROR需要与setFlipMode（）配搭使用.
 *@param reset：U8x8_PIN_NONE 表示引脚为空，不会使用复位引脚
 *
*/

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0,/* reset=*/ U8X8_PIN_NONE ,22 ,19 );  //M0/ESP32/ESP8266/mega2560/Uno/Leonardo




//oled初始化
void OledInit(void)
{
  u8g2.begin();    //初始化函数
  u8g2.enableUTF8Print();        // 为Arduino print（）函数启用UTF8支持 
  /*
   *字库占用的内存较大，谨慎使用。若只需显示固定几个字，可自行获得汉字编码，
   *通过drawXBM的方法去显示;或者使用内存更大的主控
   *中文字库：需要使用比Leonardo内存更大的主控
   *日文字库：需要使用比UNO内存更大的主控
   *韩文字库：由于Arduino现有版本的INO文件无法支持显示韩文字符，但是在屏幕上可以正常显示
  */
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);  //对“你好世界”的所有字形使用chinese2
  //u8g2.setFont(u8g2_font_b10_t_japanese1);  // 日语中已经包含了所有“こんにちは世界”的字形1：Lerning 1-6级 
  //u8g2.setFont(u8g2_font_unifont_t_korean1);  // 日语中已经包含了所有“안녕하세요세계”的字形：Lerning 1-2级

  /*@brief 设置所有字符串或字形的绘制方向setFontDirection(uint8_t dir)
   *@param dir=0，旋转0度
                 dir=1，旋转90度
                 dir=2，旋转180度
                 dir=3，旋转270度
   *@param 设置字体方向后，要重新设置光标位置才能正常显示；如果不懂可以参考API的解释
  */
  u8g2.setFontDirection(0);  

  /*
   * firstPage方法会把当前页码位置变成0
   * 修改内容处于firstPage和nextPage之间，每次都是重新渲染所有内容
   * 该方法消耗的ram空间，比sendBuffer消耗的ram空间要少
  */

  u8g2.firstPage();
  do {
      u8g2.setCursor(/* x=*/20, /* y=*/15);    //定义打印功能的光标,打印 功能的任何输出都将在此位置开始
      u8g2.print("Hello World!");
      u8g2.setCursor(33, 30);
      u8g2.print("国炫学长");        // Chinese "Hello World"  
  } while ( u8g2.nextPage() );
}

//oled显示
void oled_show(void)
{
    static float x=0;
    u8g2.firstPage();
    do {
          if(battery_voltage>=14.2)
          {
              u8g2.setCursor(/* x=*/0, /* y=*/15);    //定义打印功能的光标,打印 功能的任何输出都将在此位置开始
              u8g2.print(battery_voltage); 
              u8g2.print("V");             
          }
          else
          {
              u8g2.setCursor(/* x=*/0, /* y=*/15);    //定义打印功能的光标,打印 功能的任何输出都将在此位置开始
              u8g2.print("电压低");                
          }
          
          u8g2.setCursor(33, 30);
          u8g2.print("国炫学长");        // Chinese 
      
    } while ( u8g2.nextPage() );  
}
