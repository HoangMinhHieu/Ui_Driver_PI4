#include "widget.h"
#include "ui_widget.h"
#include "ui_opendevice.h"
#include <QApplication>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <dlfcn.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/time.h>
#include <iconv.h>
#include <iconv.h>
#define CONF_FILE_PATH  "Config.ini"
#ifdef WIN32
#include <Windows.h>
#else
#include <stdarg.h>
#endif
#define FALSE  -1
#define TRUE   0
int l_speed_arr[] = {B115200,B57600,B38400,B19200,B9600,B4800,B2400,B1200,B57600,B38400,B19200,B9600,B4800,B2400,B1200};
int l_name_arr[] = {115200,57600,38400,19200,9600,4800,2400,1200,57600,38400,19200, 9600,4800, 2400,1200};
int bPortOpen=0;
void *handle;
int CpuCardType; //current cpu Card type
int menu();
int port_fd;
unsigned char Rdata[256]={0};
unsigned int Rlen=0;

void OpenDeviceFunction();
void CloseDeviceFunction();
void InitDeviceFunction();
void CallEntryFunction();
void DisEntryFunction();
void EjectFunction();
void CaptureFunction();
Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
}

Widget::~Widget()
{
    delete ui;
}
void Widget::on_pushButton_clicked()
{
    close();
}


/**
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*/
int l_Set_ComPort(int fd, int speed)
{
    int   i;
    struct termios newtio;
    if  ( tcgetattr( fd,&newtio)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }
    fcntl(fd, F_SETFL, 0);
    //tcgetattr(fd,oldtio); //save current serial port settings
    bzero(&newtio, sizeof(newtio)); // clear struct for new port settings
    //configure the serial port;
        for ( i= 0;  i < (int) (sizeof(l_speed_arr) / sizeof(int));  i++)
    {
        if  (speed == l_name_arr[i])
        {
                cfsetispeed(&newtio, l_speed_arr[i]);
                cfsetospeed(&newtio, l_speed_arr[i]);
            }
    }

    newtio.c_cflag |=CLOCAL|CREAD;
    /*8N1*/
    newtio.c_cflag &= ~CSIZE; /* Mask the character size bits */
    newtio.c_cflag |= CS8; /* Select 8 data bits */
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CRTSCTS;//disable hardware flow control;
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);/*raw input*/
    newtio.c_oflag &= ~OPOST; /*raw output*/
    tcflush(fd,TCIFLUSH);//clear input buffer
    newtio.c_cc[VTIME] = 1; /* inter-character timer unused */
    newtio.c_cc[VMIN] = 0; /* blocking read until 0 character arrives */
    if (tcsetattr(fd,TCSANOW,&newtio) != 0)
    {
         perror("Cannot set the serial port parameters");
         return (FALSE);
    }
    return (TRUE);
}



/***@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return  void*/
void l_set_speed(int fd, int speed)
{
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);
        for ( i= 0;  i < (int) (sizeof(l_speed_arr) / sizeof(int));  i++)
    {
        if  (speed == l_name_arr[i])
        {
                tcflush(fd, TCIOFLUSH);
                    cfsetispeed(&Opt, l_speed_arr[i]);
                    cfsetospeed(&Opt, l_speed_arr[i]);
                //调试输出
                    status = tcsetattr(fd, TCSANOW, &Opt);
                    if  (status != 0)
                        perror("tcsetattr fd1");
                    return;
            }
        tcflush(fd,TCIOFLUSH);
    }
}
/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄*
*@param  databits 类型  int 数据位   取值 为 7 或者8*
*@param  stopbits 类型  int 停止位   取值为 1 或者2*
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int l_set_Parity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;

    if  ( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |=CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |=CREAD;
    //===========设置数据位============
    options.c_cflag &=~CSIZE;
    switch (databits) //设置数据位数
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            options.c_cflag |= CS8;
            break;
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;   // Clear parity enable
            break;
        case 'o':
        case 'O':
            options.c_cflag |= PARENB;     // Enable parity
            options.c_cflag |= PARODD;  // 设置为奇效验
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;     // Enable parity
            options.c_cflag &=~PARODD;   // 转换为偶效验
            break;
        case 'S':
        case 's':  //as no parity
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            options.c_cflag &= ~PARENB;   // Clear parity enable
            break;
    }
    // 设置停止位
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            options.c_cflag &= ~CSTOPB;
            break;
    }
    //Set input parity option
    if (parity != 'n' && parity != 'N')
        options.c_iflag |= INPCK;
    //修改输出模式，原始数据输出
    options.c_oflag &=~OPOST;
    //不使用流控制
    options.c_oflag &=~CRTSCTS;

    //修改控制字符，读取字符的最少个数为1
    options.c_cc[VMIN]=1;
    //修改控制字符，读取第一个字符等待1*（1/10）s
    options.c_cc[VTIME]=1;

    //如果发生数据溢出，接收数据，但是不再读取
    tcflush(fd,TCIFLUSH); // Update the options and do it NOW
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("Cannot set the serial port parameters");
        return (FALSE);
    }
    return (TRUE);
}

//关闭指定串口
int close_port(int port_fd)
{
    close(port_fd);
    port_fd=0;
    return 0 ;

}
//打开指定的串口
int open_port( char* port)
{
    unsigned int InBauteRate=9600;
     int port_fd;
    unsigned char ReData[256];
    memset(ReData,0x00,sizeof(ReData));

    port_fd =  open(port,O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (l_Set_ComPort(port_fd,InBauteRate)== FALSE)
    {
       close_port(port_fd);
        return -1;
    }
    return port_fd;
}
//用指定的波特率打开指定的串口
int open_portWithBaut( char* port, unsigned int _data)
{
   int port_fd;
    unsigned int InBauteRate;
    switch (_data)
    {
        case 1200:
            InBauteRate=1200;
            break;
        case 2400:
            InBauteRate=2400;
            break;
        case 4800:
            InBauteRate=4800;
            break;
        case 9600:
            InBauteRate=9600;
            break;
        case 19200:
            InBauteRate=19200;
            break;
        case 38400:
            InBauteRate=38400;
            break;
        case 57600:
            InBauteRate=57600;
            break;
        case 115200:
            InBauteRate=115200;
            break;
        default:
            InBauteRate=9600;
            break;
    }


    port_fd =  open(port,O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (l_Set_ComPort(port_fd,InBauteRate)== FALSE)
    {
        close_port(port_fd);
        return -1;
    }

    return port_fd;
}


int l_send_data(int fd,unsigned char *data,unsigned int data_len)
{
    unsigned int len=0;
    len=write(fd,data,data_len);
    if(len==data_len)
    {
        return len;
        }
        else
        {
        //如果出现溢出的情况
        tcflush(fd,TCOFLUSH);
        return -1;
    }
}

int l_recv_data(int fd,unsigned char *data,unsigned int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec=5;
    time.tv_usec=0;

    //使用select实现串口的多路通信
    fs_sel=select(fd+1,&fs_read,NULL,NULL,&time);

    time.tv_sec=0;
    time.tv_usec=50000;
    select(0,NULL,NULL,NULL,&time);

    if(fs_sel)
    {
        len=read(fd,data,data_len);
        return len;
    }
    else
    {
        return -1;
    }
}

int l_Firstrecv_data(int fd,unsigned char *data,unsigned int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec=5;
    time.tv_usec=0;

    //使用select实现串口的多路通信
    fs_sel=select(fd+1,&fs_read,NULL,NULL,&time);

    time.tv_sec=0;
    time.tv_usec=2000;
    select(0,NULL,NULL,NULL,&time);

    if(fs_sel)
    {
        len=read(fd,data,data_len);
        return len;
    }
    else
    {
        return -1;
    }
}


unsigned char l_CalXOR(unsigned char strOrder[],unsigned int strLen)
{
    unsigned char XORValue=0;
    unsigned int ii=0;
    for(ii=0;ii<strLen;ii++)
    {
        XORValue^=strOrder[ii];
    }
    return XORValue;
}

//////////////////////////////////////////////////////////////////////////


int RS232_ExeCommand(int fd,unsigned char addr,unsigned char TxData[], unsigned int TxDataLen,unsigned char RxData[], unsigned int *RxDataLen)
{
    unsigned char SendBuf[1024];
    unsigned char ReceiveBuf[1024];
    unsigned char SendACK[2]={0x06,0xFF};
    unsigned char ACK[2]={0xFF,0xFF};
    int  i,j;
    int CRTBackDataLen=0;


    int len,lenMore;
    if (TxDataLen<2)
    {
        return -1001;
    }

    //tcflush(fd,TCIOFLUSH);
    tcflush(fd,TCIFLUSH);

    memset(SendBuf,0x00,sizeof(SendBuf));
    memset(ReceiveBuf,0x00,sizeof(ReceiveBuf));

    SendBuf[0]=0xf2;   //发送包赋值开始
    SendBuf[1]=addr;
    SendBuf[2]=(TxDataLen)/256;
    SendBuf[3]=(TxDataLen)%256;
    if (TxDataLen >=2)
    {
        memcpy(&SendBuf[4],TxData,TxDataLen);
    }
    SendBuf[4+TxDataLen]=0x03;
    SendBuf[5+TxDataLen]=l_CalXOR(SendBuf,TxDataLen+6);



    len=l_send_data(fd,SendBuf,TxDataLen+6);	//发送[COMMAND],执行动作.
    len=l_Firstrecv_data(fd,ACK,1); //接收[ACK]。

    if(len<=0)
    {
        return -1002;
    }


    if(ACK[0]==0x15) //Negative acknowledge
    {
        return -1003;
    }

    if (ACK[0]!=0x06) //ACK应答错误。
    {
        return -1004;
    }



    len=l_recv_data(fd,ReceiveBuf,4);   //   Recv  4 unsigned ints =接收[包头][长度低字节][长度高字节]。


    if(len<=0)
    {
        return -1005;
    }
    if(ReceiveBuf[0]!=0xf2)  //[包头]格式错误。
    {
        return -1006;
    }

    CRTBackDataLen=(ReceiveBuf[2]<<8) | ReceiveBuf[3];//[报文长度]。

    lenMore=0;
    for (j=0;j<20;j++)
    {
        len=l_recv_data(fd,&ReceiveBuf[4+lenMore],CRTBackDataLen+2-lenMore);   //   再读余下数据

        lenMore=lenMore+len;
        if(len<=0)
        {
            return -1007;
        }
        if (lenMore>=CRTBackDataLen+2)
        {
            break;
        }
    }

        if (CRTBackDataLen+2!=lenMore)
    {
        return -1008;
    }

    *RxDataLen=0;

     len=l_send_data(fd,SendACK,1);	//发送[ENQ],执行动作.

    if (CRTBackDataLen>2 )
    {
        *RxDataLen=CRTBackDataLen;
        for (i=0;i<CRTBackDataLen;i++)
        RxData[i]=ReceiveBuf[4+i];
    }
    return 0;
}

//====================================CRT310DEVICE=========================
//int OpenDevice(char *sPort, char* sMes)
void OpenDeviceFunction()
{
    char devicename[50]="/dev/ttyS0";
    int Reback;


    printf("Sample of the device name  \n");
    printf("format of COM1 PORT is /dev/ttyS0  etc.\n");
    printf("format of USB PORT 1 is /dev/ttyUSB0  etc.\n");
    printf("Please input the device name(/dev/ttyS0):\n");
    Reback=scanf("%s",devicename);

    port_fd=open_port(devicename);
    if (port_fd>0)
    {
       printf("Open device OK,fd is:%d\n",port_fd);

       bPortOpen=1;
    }
    else
    {
        printf("Open device Error,fd is %d\n",port_fd);
    }
}

//int InitDevice (int Option,char* sMes);
void InitDeviceFunction()
{
    unsigned char Tdata[]={0x43,0x30,0x33};
    int Resuilt=-1;
    int i;

    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("InitDevice OK\n");
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    {
        printf("InitDevice Error,error code is %d\n",Resuilt);
    }
}

//int CloseDevice (char* sMes);
void CloseDeviceFunction()
{
    int Resuilt=-1;

    Resuilt=close_port(port_fd);
    if (Resuilt==0)
    {
       printf("CloseDevice OK\n");
    }
    else
    {
        printf("CloseDevice Error,error code is %d\n",Resuilt);
    }
}


//int CallEntry(char* sMes);
void CallEntryFunction()
{
    unsigned char Tdata[]={0x43,0x33,0x30};
    int Resuilt=-1;
    int i;

    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("CallEntry OK\n");
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    {
        printf("CallEntry Error,error code is %d\n",Resuilt);
    }
}
//int DisEntry (char* sMes);
void DisEntryFunction()
{
    unsigned char Tdata[]={0x43,0x33,0x31};
    int Resuilt=-1;
    int i;

    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("DisEntry OK\n");
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    {
        printf("DisEntry Error,error code is %d\n",Resuilt);
    }
}

//int Eject(char* sMes);
void EjectFunction()
{
    unsigned char Tdata[]={0x43,0x32,0x30};
    int Resuilt=-1;
    int i;

    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("Eject OK\n");
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    {
        printf("Eject Error,error code is %d\n",Resuilt);
    }
}

//int Capture(char* sMes);  capture to 1#
void CaptureFunction()
{
    unsigned char Tdata[]={0x43,0x32,0x33};
    int Resuilt=-1;
    int i;

    if (bPortOpen==0)
    {
       printf("Open the Comm. port first\n\n\n");
        return;
    }

    Resuilt=RS232_ExeCommand(port_fd,0,Tdata,3,Rdata,&Rlen);
    if (Resuilt==0)
    {
       printf("Capture OK\n");
       printf("Rlen=%d\n",Rlen);
       for(i=0;i<Rlen;i++)
       {
           printf(" %02X ",Rdata[i]);
       }
       printf("\n");
    }
    else
    {
        printf("Capture Error,error code is %d\n",Resuilt);
    }
}


int menu()
{
    int i;
        int Reback;
    i=0;

    //CPUPowerOnFunction
  //CPUChipIoFunction
    //CPUPowerOffFunction
    while(1)
    {


        printf("Please select a menu(0~9):\n");
        printf("    ---------Communication Settings----------------------\n");
        printf("    ---------Reader Operation----------------------------\n");
        printf("    1:  OpenDevice.\n");
        printf("    2:  InitDevice.\n");
        printf("    3:  CallEntry.\n");
        printf("    4:  DisEntry.\n");
        printf("    5:  Eject.\n");
        printf("    6:  Capture.\n");
        printf("    ----------Exit -----------------------------\n");
        printf("    0:  Exit (Close the Comm. port).\n");
        Reback=scanf("%d",&i);
        printf("\n");
            printf("You have chosen: %d\n",i);
        if (i==0)
        {
                if (bPortOpen==1)
                {
                CloseDeviceFunction();//关闭串口
                }
            printf("Byebye.\n\n\n");
            (handle);
            exit(1);
        }
        if (i<0 || i>13)
        {
           printf("Error.\n\n");
            continue;
        }
        else
        {
            return(i);
        }
    }
}




void Widget::on_pushButton_2_clicked()
{
    //lay lua chon
    if(ui->OpenDevice->isChecked())
    {
        OpenDeviceFunction();
        QWidget *opendevice = new QWidget;
        opendevice->show();
    }
    else if(ui->InitDevice->isChecked())
    {
        InitDeviceFunction();
    }
    else if(ui->CallEntry->isChecked())
    {
        CallEntryFunction();
    }
    else if(ui->DisEntry->isChecked())
    {
        DisEntryFunction();
    }
    else if(ui->Eject->isChecked())
    {
        EjectFunction();
    }
    else if(ui->Capture->isChecked())
    {
        CaptureFunction();
    }
}
