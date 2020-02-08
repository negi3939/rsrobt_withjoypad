//20191213作成
#include <stdio.h>
#include <math.h>

int main(void)
{

    //各パラメータの宣言
    float pi=3.14;
    float phi;             //姿勢[°]...入力パラメータ
    float psi;           //姿勢[rad]...phiより計算
    float Xe,Ye;           //手先座標...入力パラメータ
    float L1,L2,L3;        //リンク長...プリセット値
    float th1,th2,th3;     //各関節角度...出力されるパラメータ  [rad?]
    float ja1,ja2,ja3;     //[deg]
    float X3,Y3;           //第3関節の座標...入力値とリンク長から求まる値

    //パラメータへの数値代入
        //プリセット値
        L1=0.090, L2=0.090, L3=0.160;    //リンク長は変更する予定なので仮の値

        //入力値
        printf("input Xe[m]\n Xe=");
        scanf("%f",&Xe);
        printf("input Ye[m]\n Ye=");
        scanf("%f",&Ye);
        printf("input phi[deg]\n phi=");
        scanf("%f",&phi);

    //計算
        //psi[rad]を求める
        psi=phi*(pi/180);

        //X3,Y3を求める
        X3=Xe-L3*cos(psi);
        Y3=Ye-L3*sin(psi);

        //各関節角度を求める
        float Z=sqrtf((X3*X3)+(Y3*Y3));

            //[rad]
        th1=atan(Y3/X3)-acos( ( (L1*L1)-(L2*L2)+( (X3*X3)+(Y3*Y3) ) ) / (2*L1*Z) );
        th2=acos( ( (L1*L1)-(L2*L2)+( (X3*X3)+(Y3*Y3) ) / (2*L1*Z) ) ) + acos( ( (L2*L2)-(L1*L1)+( (X3*X3)+(Y3*Y3) ) ) / (2*L2*Z));   
        th3=psi-th1-th2;
            //[deg]
        ja1=th1*(180/pi);
        ja2=th2*(180/pi);
        ja3=th3*(180/pi);

            //範囲外の場合、全体にマイナス掛ける
        if(ja1<-30){
            th1=-th1;
            th2=-th2;
            th3=-th3;
            ja1=th1*(180/pi);
            ja2=th2*(180/pi);
            ja3=th3*(180/pi);
        }

    //計算結果の表示
    // printf("\n目標関節角度[rad]\n");
    // printf("th1=%f[rad]\n",th1);
    // printf("th2=%f[rad]\n",th2);
    // printf("th3=%f[rad]\n",th3);
    
    //確かめ(X3,Y3)
    printf("\nX3=%f\n",X3);
    printf("Y3=%f\n",Y3);

    printf("\n目標関節角度[deg]\n");
    printf("ja1=%f[deg]\n",ja1);
    printf("ja2=%f[deg]\n",ja2);
    printf("ja3=%f[deg]\n",ja3);

    //nan=not a number: 非数...主に浮動小数点演算の結果が正常な結果でないことを表す特殊な表現形式


    //FKの確かめ
    float X2,Y2;

    X2=L1*cos(th1);
    Y2=L1*sin(th1);

    X3=X2+L2*cos(th1+th2);
    Y3=Y2+L2*sin(th1+th2);
    
    Xe=X3+L3*cos(th1+th2+th3);
    Ye=Y3+L3*sin(th1+th2+th3);

   //それぞれの関節位置表示（入力に対する答え）
    printf("\nそれぞれの関節の座標は\n");
    printf("  (X2,Y2)=(%f,%f)\n",X2,Y2);
    printf("  (X3,Y3)=(%f,%f)\n",X3,Y3);
    printf("  (Xe,Ye)=(%f,%f)\n",Xe,Ye);


    return 0;
        
}