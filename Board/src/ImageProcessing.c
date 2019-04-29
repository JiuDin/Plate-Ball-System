#include "common.h"
#include "ImageProcessing.h"
#include "include.h"

uint8 Image_Data_Black_spots[CAMERA_H];   
int    Axis_X = 0;   //��ȡ����������X�е� ���ƶ��
int    Axis_Y = 0;    //��ȡ����������Y�е� ���Ƶ��
extern uint8 imgbuff[CAMERA_SIZE];      //imgbuff�ǲɼ��Ļ�������img�ǽ�ѹ��Ļ�������
                                  
extern uint8 img[CAMERA_H*CAMERA_W]; //imgbuff���ڲɼ�ͼ��img����ͼ����. 

int gete_X[5]={0};
int gete_Y[5]={0};
//
//void IMAGE_PRO(void)
//{
//        unsigned int i=0;  
//	unsigned char *p=NULL;
//	int n=0;
//      
//        int j,num=0,Y=0;
//        int sum=0;
//      // Axis_Y=0;
//        img_extract((uint8 *)img,(uint8 *) imgbuff, CAMERA_H*CAMERA_W/8);    //��ѹ
//       
////        for(j=3;j>0;j--)
////        {
////        gete_Y[j]=gete_Y[j-1];
////   
////        }
////	for (i = 0;i < CAMERA_H;i++)                                         //�˵��������ĵ�
////        {               
////		p = &img[i][1];
////		while (p < &img[i][135-1])
////                {
////		  if (*(p-1) == 0xff && *p == 0x00 && *(p+1) ==0xff)
////                  {
////			  *p = 0xff;
////                  }
////                  else if (*(p-1) == 0x00 && *p ==0xff && *(p+1) ==0x00)
////                  {
////			*p = 0x00;
////                  }
////                  p++;
////		}
////	}
//        
//
//
//         //----------------------------------------��ȡ�ű��Y����------------------------------------------------//
//        for(i=0;i<CAMERA_H;i++) 
//        {
//           Image_Data_Black_spots[i] = 0;//����ֵ
//        }
//         //Axis_Y=0;
//        for (i = 0;i < CAMERA_H;i++)
//        {               
//          p = &img[i][1];
//          while (p < &img[i][135-1])
//            {
//		  if (*p == 0xff && *(p+1) ==0xff)
//                  {
//			 Image_Data_Black_spots[i]++;  //�����߲�Ϊ0��
//                         //����Image_Data_Black_spots[i]�Ĵ�С�жϿ��
//                  }
//                  p++;
//	    }
//          
//           if(i == 0)
//           {
//             n=Image_Data_Black_spots[0];//==0
//            }
//            else if(Image_Data_Black_spots[i] > n )
//              {
//                  n=Image_Data_Black_spots[i];  //   nΪ�����
//                  Y = i;                   // i Ӧ�����������ĵ�������
//              }
//             
//	}  
//        Axis_Y = Y;
//
//}
//
////----------------------------------------��ȡX������------------------------------------------------//
//void Get_X(void)
//  {
//    int i,j,num=0;
// 
//    int sum=0;
//    //Axis_X=0;
//    int start=0,end=0;
//    for(i=0;i<135;i++)// ��ͼ���������߱���ʺϼ��� �����BUG
//      {
//          img[i][0] =img[i][135-1]=0x00;  
//      }
//    for(i = 0; i < 135; i++ )
//      {
//        if(img[Axis_Y][i] == 0x00 && img[Axis_Y][i+1] ==0xff)
//        start=i;
//        if(img[Axis_Y][i] == 0xff && img[Axis_Y][i+1] ==0x00)
//        end=i;
//    
//      }
//      Axis_X=(start+end)/2;
//
//}
//
//






/*****************************************
 *  @brief      ����۵Ķ�ֵ��ͼ���ѹ
 *  @since      v5.0 //  img_extract_haomi(img,imgbuff,CAMERA_SIZE);
 ****************************************/
void img_extract_haomi(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {1, 0};
    uint8 tmpsrc;
    while(srclen --)
    {
        tmpsrc = *src++;
        *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
    }
}

extern Ball_Position Ball;

/*************************************************************************
*  �������ƣ�Position_Calculate 
*  ����˵����С��λ�ü���
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2017-8-03    �Ѳ���
*************************************************************************/
void Position_Calculate()
{
  int sumX = 0,sumY = 0;
  int i,j,cnt=0;
  for(i=3;i<CAMERA_H-3;i++)
  {
    for(j=2;j<CAMERA_W-2;j++)
    {
      if(img[i*CAMERA_W+j]==0)   //���Ϊ��
      {
        sumX+=i;
        sumY+=j;
        cnt++;
      }
    }
  }
  if(sumX <5&&sumY<5)
  {
      Ball.now_x = 46;//�е�����
      Ball.now_y = 6;
  }
  else
  {
      Ball.now_x = (int)(sumX /(float)cnt);
      Ball.now_y = (int)(sumY /(float)cnt);
  }
}





