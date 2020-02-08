#include <termio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
//rawモードにするために追加したヘッダとdefine
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1 /* POSIX 準拠のソース */
#define BYTE unsigned char
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define NUM_of_SERVO 3

//jp関連
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <stdlib.h>
    #include <stdio.h>
    #include <stdint.h>

    #include <linux/joystick.h>

    #define JOYDEVNAME "/dev/input/js0"

    #define JOYAXIS_NUM (4)

    // #define JS_EVENT_BUTTON 0x01
    // #define JS_EVENT_AXIS 0x02

    enum { JOYBTN_0=0, JOYBTN_1, JOYBTN_2, JOYBTN_3, 
        JOYBTN_4, JOYBTN_5, JOYBTN_6, JOYBTN_7, JOYBTN_NUM };

    const uint8_t joybtn_map[JOYBTN_NUM] = { 2-1, 3-1, 1-1, 4-1, 10-1, 7-1, 8-1, 9-1};

    struct {
    int16_t axis[JOYAXIS_NUM];
    uint8_t btn_state[JOYBTN_NUM];
    uint8_t btn_edge[JOYBTN_NUM];
    } joyState;
//

void comm_conf(int fd); //通信設定のための関数
int RSTorqueOnOff(int SERVO_ID, int fd, short sMode);
int AllRSTorqueOnOff(int fd, short sMode);
int RSMove(int SERVO_ID, int fd, short sPos, unsigned short sTime);
int AllRSMove(int fd, short sPos1, short sPos2, short sPos3, unsigned short sTime);
int safe_judge(short sPos1, short sPos2, short sPos3);
short RSGetAngle(int fd, int ID);
int RSTorqueOnOff(int SERVO_ID, int fd, short sMode)
{
    unsigned char sendbuf[28];
    unsigned char sum;
    int i;
    int ret;

    // ハンドルチェック
    if (!fd)
    {
        return -1;
    }

    // バッファクリア
    memset(sendbuf, 0x00, sizeof(sendbuf));

    // パケット作成
    sendbuf[0] = (unsigned char)0xFA;             // ヘッダー1
    sendbuf[1] = (unsigned char)0xAF;             // ヘッダー2
    sendbuf[2] = (unsigned char)SERVO_ID;         // サーボID
    sendbuf[3] = (unsigned char)0x00;             // フラグ
    sendbuf[4] = (unsigned char)0x24;             // アドレス(0x24=36)
    sendbuf[5] = (unsigned char)0x01;             // 長さ(4byte)
    sendbuf[6] = (unsigned char)0x01;             // 個数
    sendbuf[7] = (unsigned char)(sMode & 0x00FF); // ON/OFFフラグ
    // チェックサムの計算
    sum = sendbuf[2];
    for (i = 3; i < 8; i++)
    {
        sum = (unsigned char)(sum ^ sendbuf[i]);
    }
    sendbuf[8] = sum; // チェックサム

    // 通信バッファクリア
    // PurgeComm( hComm, PURGE_RXCLEAR );
    tcflush(fd, TCIFLUSH);
    // 送信
    ret = write(fd, &sendbuf, 9);

    return ret;
}

int main()
{
    //宣言
        int fd;
        int ret = 0;
        
        float pi = 3.14;
        float L1, L2, L3;         // Link length [m]
        float Ja1, Ja2, Ja3;      // joint angle [deg]
        float th1, th2, th3 ;     // theta [rad]
        float X2, X3, Xe;         // 各関節のx座標[m] *e=end effector
        float Y2, Y3, Ye;         // 各関節のy座標[m]
        
        //float phi;                //手先姿勢[deg]...入力パラメータ
        float psi=pi/2;                //手先姿勢[rad]...phiより計算  

        L1=0.090, L2=0.090, L3=0.160;  //リン長入力,単位[m], L3(手先長さ)は仮
    //

    //openしてファイルディスクリプタを獲得
        fd = open(MODEMDEVICE, O_RDWR);
        /*-------------説明-------------------
        * 対象のデバイスを読み書き可能なモードでオープンする．
        * O_RDONLY 読み込み専用でオープン
        * O_WRONLY 書き込み専用でオープンb
        * O_RDWR 読み込みと書き込み用にオープン
        * O_APPEND 書き込みのたびに末尾に追加する
        * O_CREAT ファイルが存在しない場合、新たなファイルを作成する
        * O_TRUNC サイズを 0 バイトに切り捨てる
        -------------------------------------*/
        printf("fd=%d\n", fd); //ファイルディスクリプタに3が割り当てられる。usbが繋がっていないと-1 (エラー)がfdに入る
        comm_conf(fd);         //通信設定をする
    //

      printf( "SEND Torque ON\n" );
        ret = RSTorqueOnOff(2,fd, 1 );
        if( ret < 0 ){
            printf( "ERROR:Torque ON failed[%x]\n", ret );
            return -1;
    }
}
void comm_conf(int fd) //通信設定のための関数
{
    // 4_22 int status;
    //4_22  struct termio tio;
    struct termios tio; //4_22追加

    //4_22コメントアウト status = ioctl (fd, TCGETA, &tio);//TCGETAで現在のシリアルポートの設定を取得する
    //4_22コメントアウト　if (status < 0) perror("TCGETA");

    bzero(&tio, sizeof(tio)); //4_22追加　termio構造体のtioの全てを0にリセット

    // tio.c_cflag = 0;//一度0にセットして全てのフラグをリセットする．
    tio.c_cflag |= (BAUDRATE | CS8 | CLOCAL | CREAD);
    //制御モードの指定。termios構造体member c_cflagでflagを指定
    //c_cflag フィールドターミナルのハードウェア制御の設定を行う．
    /*
    BAUDRATE: ボーレートの設定．cfsetispeed と cfsetospeed も使用できる．
    CRTSCTS : 出力のハードウェアフロー制御 (必要な結線が全てされているケー
    ブルを使う場合のみ．Serial-HOWTO の7章を参照のこと)
    CS8     :送受信8ビットを使う。 8n1 (8 ビット，ノンパリティ，ストップビット 1)
    CLOCAL  : modem stetus lineを無視する。ローカル接続，モデム制御なし
    CREAD   : 文字の受信を可能にする。受信文字(receiving characters)を有効にする．
  */

    tio.c_cc[VTIME] = 10; //バーストで短期のデータ伝送を
                          //タイムアウトするために使用する0.10秒単位のタイマ 単位は 100[msec]
    tio.c_cc[VMIN] = 0;   //受信すべき最小の文字数
    //特殊制御文字は c_cc 配列で定義される．

    tio.c_iflag = 0; //入力モードの設定。Rawモードでの入力
    //c_iflag フィールドは基本的なターミナル入力の制御の設定に使われる．

    tio.c_oflag = 0; //出力モードの設定。Rawモードでの出力
    //c_oflag フィールドはシステムの出力処理の取り扱いの設定に使われる．

    tio.c_lflag = 0; //ローカルモードの設定．端末の特性を制御
    //c_lflag フィールドはターミナルの制御に使用される．

    tcflush(fd, TCIFLUSH); //通信バッファクリア

    tcsetattr(fd, TCSANOW, &tio); //4_22追加

    //  status = ioctl (fd, TCSETA, &tio);//TCSETSで現在のシリアルポートの設定をセットする
    //if (status < 0) perror("TCSETA");
}